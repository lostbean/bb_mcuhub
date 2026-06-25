defmodule SegbyV1.Sim.MujocoPlant do
  @moduledoc """
  The MuJoCo `BBMCUHub.Sim.Plant` for segby_v1 (ADR-0008) — the EXAMPLE's dynamics
  seam, owning a `Port` to a headless Python/MuJoCo child.

  The library ships the engine-agnostic sim building blocks (`BBMCUHub.Sim.Plant`
  behaviour + `Sim.Transport` + `Sim.Driver`, ADR-0003); this consumer module is one
  `Plant` implementation. It translates between segby's wire slots and MuJoCo's
  arrays:

    * the two `:effort` wheel commands → MuJoCo's `data.ctrl` (index 0 = left,
      index 1 = right — this MUST match the actuator order in `sim/segby.xml`),
    * MuJoCo's IMU site sensors → the blaster `pose` port's 10-field `:imu` value
      (quaternion from `framequat`, angular velocity from `gyro`, linear accel from
      `accelerometer`),
    * a per-wheel `applied_seq` counter → the two wheel `:status` values (in sim the
      wheel is *applied*, never *floored*).

  ## The Port wire (JSON-per-line, ADR-0008)

  The child is command-driven over a `line:`-framed Port. The plant writes ONE JSON
  line per `step/3` and reads back ONE JSON line:

      → {"op":"set_ctrl_and_step","ctrl":[left, right],"n":N}
      ← {"event":"state","sensordata":[...],"qpos":[...],"qvel":[...],
         "framequat":[w,x,y,z],"gyro":[gx,gy,gz],"acc":[ax,ay,az]}

  `n` is the number of MuJoCo `mj_step`s to advance per host tick, derived from the
  driver's `dt_s` and the MJCF timestep so one host tick advances the same simulated
  time regardless of the MJCF substep size. `quit` closes the child.

  ## The child seam (testability, ADR-0008)

  `init/1` accepts an injected `:child` module (default `SegbyV1.Sim.MujocoPlant.Port`,
  the real Port wrapper) implementing the tiny `SegbyV1.Sim.MujocoPlant.Child`
  behaviour (`open/1`, `send_line/2`, `recv_line/1`, `close/1`). A test passes a FAKE
  child that feeds canned lines and records what was written, so the plant's command
  mapping + sensor parsing are unit-tested with NO MuJoCo and NO Port.
  """
  @behaviour BBMCUHub.Sim.Plant

  require Logger

  alias BBMCUHub.Contract.PortIndex
  alias SegbyV1.Sim.MujocoPlant.Child

  # The default MJCF, relative to the example app root (the consumer's sim model).
  @default_mjcf "sim/segby.xml"

  # The MJCF integration step (must match `<option timestep>` in sim/segby.xml).
  @mjcf_timestep_s 0.002

  defmodule Child do
    @moduledoc """
    The tiny child seam the plant talks to (ADR-0008): a line-framed transport to
    the MuJoCo process. The real impl wraps an OS `Port`; a test impl feeds canned
    lines so the plant runs with NO MuJoCo.
    """
    @typedoc "The opaque child handle (a Port, or a test pid/ref)."
    @type t :: term()

    @doc "Open the child, returning its handle. `opts` carries `:exe` + `:args`."
    @callback open(opts :: keyword()) :: {:ok, t()}
    @doc "Write one already-encoded line (no trailing newline) to the child."
    @callback send_line(t(), iodata()) :: :ok
    @doc "Read the next line (without the trailing newline) from the child."
    @callback recv_line(t()) :: {:ok, binary()} | {:error, term()}
    @doc "Close the child, releasing the Port. Idempotent."
    @callback close(t()) :: :ok
  end

  defmodule Port do
    @moduledoc """
    The real `Child`: an OS `Port` to the Python/MuJoCo child, framed line-by-line
    (`Port.open(..., [:binary, line: 8192])`). segby's vectors are tiny, so 8 KB is
    ample (ADR-0008).
    """
    @behaviour Child

    @line 8192

    @impl Child
    def open(opts) do
      exe = Keyword.fetch!(opts, :exe)
      args = Keyword.get(opts, :args, [])

      port =
        Elixir.Port.open(
          {:spawn_executable, exe},
          [:binary, :exit_status, {:line, @line}, {:args, args}]
        )

      {:ok, port}
    end

    @impl Child
    def send_line(port, line) do
      true = Elixir.Port.command(port, [line, "\n"])
      :ok
    end

    @impl Child
    def recv_line(port) do
      # The Port is opened with `{:line, n}`, so a complete line arrives as
      # `{:eol, data}` and an over-long fragment as `{:noeol, data}`; reassemble.
      recv_line(port, [])
    end

    defp recv_line(port, acc) do
      receive do
        {^port, {:data, {:eol, data}}} ->
          {:ok, IO.iodata_to_binary(Enum.reverse([data | acc]))}

        {^port, {:data, {:noeol, data}}} ->
          recv_line(port, [data | acc])

        {^port, {:exit_status, status}} ->
          {:error, {:exit_status, status}}
      after
        10_000 -> {:error, :timeout}
      end
    end

    @impl Child
    def close(port) do
      try do
        Elixir.Port.close(port)
      rescue
        ArgumentError -> :ok
      end

      :ok
    end
  end

  # --- Plant behaviour ---

  @impl BBMCUHub.Sim.Plant
  def init(opts) do
    child_mod = Keyword.get(opts, :child, Port)
    mjcf = Keyword.get(opts, :mjcf, @default_mjcf)

    child_opts = Keyword.get(opts, :child_opts, port_child_opts(mjcf))

    {:ok, child} = child_mod.open(child_opts)

    # The child's first line is `{"event":"ready", ...}`; consume it.
    {:ok, ready_line} = child_mod.recv_line(child)
    %{"event" => "ready"} = Jason.decode!(ready_line)

    # Resolve segby's wire slots once (PortIndex must be built for @robot first; the
    # driver/host build it at boot, tests build it in setup).
    {:ok, pose_slot} = PortIndex.resolve(:blaster, :pose)
    {:ok, status_left_slot} = PortIndex.resolve(:wheels, :status_left)
    {:ok, status_right_slot} = PortIndex.resolve(:wheels, :status_right)
    {:ok, motor_left_slot} = PortIndex.resolve(:wheels, :motor_left)
    {:ok, motor_right_slot} = PortIndex.resolve(:wheels, :motor_right)
    {:ok, vel_left_slot} = PortIndex.resolve(:wheels, :vel_left)
    {:ok, vel_right_slot} = PortIndex.resolve(:wheels, :vel_right)

    state = %{
      child_mod: child_mod,
      child: child,
      mjcf_timestep_s: Keyword.get(opts, :mjcf_timestep_s, @mjcf_timestep_s),
      # ctrl index 0 = left, 1 = right (MUST match sim/segby.xml actuator order).
      motor_left_slot: motor_left_slot,
      motor_right_slot: motor_right_slot,
      pose_slot: pose_slot,
      status_left_slot: status_left_slot,
      status_right_slot: status_right_slot,
      vel_left_slot: vel_left_slot,
      vel_right_slot: vel_right_slot,
      # per-wheel applied_seq counters echoed in the status values.
      applied_left: 0,
      applied_right: 0
    }

    {:ok, state}
  end

  @impl BBMCUHub.Sim.Plant
  def step(commands, dt_s, state) do
    left = effort_for(commands, state.motor_left_slot)
    right = effort_for(commands, state.motor_right_slot)

    # Advance the same simulated time per host tick regardless of MJCF substep size:
    # n = round(dt_s / mjcf_timestep), at least 1.
    n = max(1, round(dt_s / state.mjcf_timestep_s))

    cmd = %{"op" => "set_ctrl_and_step", "ctrl" => [left, right], "n" => n}
    :ok = state.child_mod.send_line(state.child, Jason.encode!(cmd))

    {:ok, line} = state.child_mod.recv_line(state.child)
    %{"event" => "state"} = reply = Jason.decode!(line)

    applied_left = wrap_u16(state.applied_left + 1)
    applied_right = wrap_u16(state.applied_right + 1)

    {vel_left_rad_s, vel_right_rad_s} = wheel_velocities(reply)

    sensors = [
      pose_sensor(state.pose_slot, reply),
      status_sensor(state.status_left_slot, applied_left),
      status_sensor(state.status_right_slot, applied_right),
      wheel_speed_sensor(state.vel_left_slot, vel_left_rad_s),
      wheel_speed_sensor(state.vel_right_slot, vel_right_rad_s)
    ]

    {sensors, %{state | applied_left: applied_left, applied_right: applied_right}}
  end

  @impl BBMCUHub.Sim.Plant
  def close(state) do
    _ = safe_send_quit(state)
    state.child_mod.close(state.child)
    :ok
  end

  # --- internals ---

  # The latest effort (N·m) captured for a wheel slot this tick, or 0.0 if none.
  defp effort_for(commands, slot) do
    case Map.get(commands, slot) do
      %{nm: nm} -> nm * 1.0
      _ -> 0.0
    end
  end

  # Build the blaster `pose` :imu value from the child's framequat/gyro/acc arrays.
  # framequat is [w, x, y, z]; gyro is angular velocity (rad/s); acc is linear
  # acceleration (m/s²). All floats.
  defp pose_sensor({node, port_id}, reply) do
    [qw, qx, qy, qz] = floats(reply["framequat"], 4)
    [wx, wy, wz] = floats(reply["gyro"], 3)
    [ax, ay, az] = floats(reply["acc"], 3)

    value = %{
      qw: qw,
      qx: qx,
      qy: qy,
      qz: qz,
      wx: wx,
      wy: wy,
      wz: wz,
      ax: ax,
      ay: ay,
      az: az
    }

    {node, port_id, :imu, value}
  end

  # In sim the wheel is APPLIED, not floored: floored: false, applied_seq echoes a
  # per-wheel u16 counter.
  defp status_sensor({node, port_id}, applied_seq) do
    {node, port_id, :status, %{applied_seq: applied_seq, floored: false}}
  end

  # The measured wheel shaft angular velocity (rad/s) reported as the
  # `SegbyV1.ValueTypes.WheelSpeed` value-type (layout `{rad_s: :f32}`, ADR-0009).
  # The `type` is the value-type MODULE (not a stock atom like :imu/:status): the
  # codec resolves it via `BBMCUHub.ValueType.resolve/1`, which passes a module
  # through unchanged — exactly how the wheels hub names the port
  # (`type: SegbyV1.ValueTypes.WheelSpeed`).
  defp wheel_speed_sensor({node, port_id}, rad_s) do
    {node, port_id, SegbyV1.ValueTypes.WheelSpeed, %{rad_s: rad_s}}
  end

  # The left/right wheel hinge angular velocities (rad/s) from MuJoCo's `qvel`.
  #
  # For segby's freejoint-base model the generalized-velocity (`qvel`) layout is:
  # the `root` freejoint's 6 DOF first (3 linear + 3 angular, dofadr 0..5), THEN the
  # two wheel hinges — `wheel_l_joint` at dofadr 6, `wheel_r_joint` at dofadr 7
  # (verified against sim/segby.xml: nv=8, jnt_dofadr [0, 6, 7]). So the wheel
  # velocities are `qvel[6]` (left) and `qvel[7]` (right). The child already ships
  # the full `qvel` in its `state` reply; we read these two indices directly.
  defp wheel_velocities(reply) do
    qvel = reply["qvel"]
    {float_at(qvel, 6), float_at(qvel, 7)}
  end

  # The qvel element at `i` coerced to float (the child `.tolist()`s numpy arrays,
  # so a whole value can arrive as an int — normalize). 0.0 if the index is absent.
  defp float_at(list, i) when is_list(list) do
    case Enum.at(list, i) do
      n when is_number(n) -> n * 1.0
      _ -> 0.0
    end
  end

  # Coerce a JSON number list to exactly `n` floats (the child `.tolist()`s numpy
  # arrays, so ints can arrive for whole values — normalize to float).
  defp floats(list, n) when is_list(list) and length(list) == n do
    Enum.map(list, &(&1 * 1.0))
  end

  defp wrap_u16(seq), do: seq |> rem(0x1_0000)

  # The Python child script (the MuJoCo stepper) that the interpreter runs, beside
  # the MJCF in the example's sim/ dir.
  @child_script "segby_sim.py"

  defp port_child_opts(mjcf) do
    [exe: exe(mjcf), args: child_args(mjcf)]
  end

  @doc """
  The interpreter args for an `mjcf` path: `[<script.py>, <mjcf>]`. Exposed (and
  kept separate from interpreter resolution) so it is unit-testable without a real
  interpreter on the box. Guards the smoke-run bug: the interpreter is invoked
  `<exe> <script.py> <mjcf>`, so the args MUST lead with the Python script, not the
  MJCF — passing only the MJCF made the interpreter try to run `segby.xml` as Python.
  """
  @spec child_args(binary()) :: [binary()]
  def child_args(mjcf) do
    script = Elixir.Path.join(Elixir.Path.dirname(mjcf), @child_script)
    # `SEGBY_SIM_HEADLESS=1` runs the child without the MuJoCo viewer window — for
    # automated balance/regression tests on a box with no display (where
    # launch_passive segfaults). Physics is identical; only rendering is skipped.
    if System.get_env("SEGBY_SIM_HEADLESS") == "1" do
      [script, "--headless", mjcf]
    else
      [script, mjcf]
    end
  end

  # The interpreter that runs the child. macOS needs `mjpython` (the GLFW
  # main-thread rule); Linux uses `python`. `:erlang.open_port({:spawn_executable,
  # …})` needs a REAL PATH, not a bare name to search — a bare name throws an opaque
  # `:enoent`. So resolve a concrete path: prefer the sim's own `uv` venv (it sits at
  # `<mjcf dir>/.venv/bin/`, where `uv sync` installs mjpython), then PATH, and if
  # neither exists raise an actionable error naming the `uv sync` step.
  defp exe(mjcf) do
    name =
      case :os.type() do
        {:unix, :darwin} -> "mjpython"
        _ -> "python"
      end

    venv = Elixir.Path.join(Elixir.Path.dirname(mjcf), ".venv")
    venv_bin = Elixir.Path.join([venv, "bin", name])

    cond do
      Elixir.File.exists?(venv_bin) ->
        ensure_mjpython_libpython(venv, name)
        venv_bin

      path = System.find_executable(name) ->
        path

      true ->
        raise_no_interpreter(name, venv_bin)
    end
  end

  # macOS `mjpython` re-execs the venv python from inside a Cocoa .app bundle and
  # dlopens `libpython3.12.dylib` via `@executable_path/../lib/`. uv's venv layout
  # does not ship that dylib, so without a symlink at `<venv>/lib/` the load fails.
  # The flake's shellHook creates it — but only on devShell ENTRY, so a venv made
  # by `uv sync` AFTER entering the shell is missed. Create it here too (idempotent,
  # venv-local), so the sim works regardless of shellHook timing. No-op off macOS or
  # if the dylib can't be located.
  defp ensure_mjpython_libpython(_venv, "python"), do: :ok

  defp ensure_mjpython_libpython(venv, _mjpython) do
    link = Elixir.Path.join([venv, "lib", "libpython3.12.dylib"])

    unless Elixir.File.exists?(link) do
      with real_python when is_binary(real_python) <- venv_real_python(venv),
           lib_dir =
             Elixir.Path.join(Elixir.Path.dirname(Elixir.Path.dirname(real_python)), "lib"),
           dylib = Elixir.Path.join(lib_dir, "libpython3.12.dylib"),
           true <- Elixir.File.exists?(dylib) do
        Elixir.File.mkdir_p(Elixir.Path.dirname(link))
        # Best-effort: a failure here just surfaces later as the dlopen error.
        _ = Elixir.File.ln_s(dylib, link)
        :ok
      else
        _ -> :ok
      end
    end

    :ok
  end

  # The real interpreter the venv `bin/python` symlink points at (uv's managed
  # CPython), whose sibling `lib/` holds the dylib.
  defp venv_real_python(venv) do
    venv_python = Elixir.Path.join([venv, "bin", "python"])

    case Elixir.File.read_link(venv_python) do
      {:ok, target} -> target
      _ -> if Elixir.File.exists?(venv_python), do: venv_python
    end
  end

  defp raise_no_interpreter(name, venv_bin) do
    raise """
    The sim interpreter `#{name}` was not found.

    Looked for the sim's uv venv at:
        #{venv_bin}
    and on PATH (`#{name}` is not there either).

    Install the MuJoCo deps once (from the example app root):
        cd sim && uv sync

    On macOS re-enter the devShell afterwards so the shellHook symlinks
    libpython for `mjpython` (see sim/README.md).
    """
  end

  defp safe_send_quit(state) do
    state.child_mod.send_line(state.child, Jason.encode!(%{"op" => "quit"}))
  rescue
    e ->
      Logger.debug("sim plant quit send failed (child likely gone): #{Exception.message(e)}")
      :ok
  end
end
