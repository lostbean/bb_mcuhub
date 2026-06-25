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

    sensors = [
      pose_sensor(state.pose_slot, reply),
      status_sensor(state.status_left_slot, applied_left),
      status_sensor(state.status_right_slot, applied_right)
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

  # Coerce a JSON number list to exactly `n` floats (the child `.tolist()`s numpy
  # arrays, so ints can arrive for whole values — normalize to float).
  defp floats(list, n) when is_list(list) and length(list) == n do
    Enum.map(list, &(&1 * 1.0))
  end

  defp wrap_u16(seq), do: seq |> rem(0x1_0000)

  defp port_child_opts(mjcf) do
    [exe: exe(), args: [mjcf]]
  end

  # macOS needs `mjpython` (the GLFW main-thread rule); Linux uses `python`.
  defp exe do
    case :os.type() do
      {:unix, :darwin} -> System.find_executable("mjpython") || "mjpython"
      _ -> System.find_executable("python") || "python"
    end
  end

  defp safe_send_quit(state) do
    state.child_mod.send_line(state.child, Jason.encode!(%{"op" => "quit"}))
  rescue
    e ->
      Logger.debug("sim plant quit send failed (child likely gone): #{Exception.message(e)}")
      :ok
  end
end
