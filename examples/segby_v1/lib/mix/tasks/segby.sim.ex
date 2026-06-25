defmodule Mix.Tasks.Segby.Sim do
  @shortdoc "Run segby_v1 virtually — the real host stack over a MuJoCo physics plant (no hardware)"

  @moduledoc """
  Stand up the **whole virtual segby_v1** in one command (ADR-0008): the real
  host control stack — codec, freshness monitor, floor semantics, views, the
  balance + teleop laws — running unchanged over a MuJoCo physics model, with a
  native MuJoCo viewer window rendering the bot in 3D. **No hardware.**

  The transport is the one and only hardware boundary (ADR-0008). This task swaps
  it for the library's `BBMCUHub.Sim.Transport` and closes the loop with a
  `BBMCUHub.Sim.Driver` driving the example's `SegbyV1.Sim.MujocoPlant`. Every box
  above the transport is shipped code — the same path a real board lights up.

  ## What it wires (the three moving parts)

    1. **`SegbyV1.Host`** with the SIM transport injected — this stands up the BB
       supervision tree (views, balance, teleop) plus the `LinkOwner`, which owns
       the transport. The transport is started by the `LinkOwner` (its owner) and
       given a known `:name` so the driver can find it:

           SegbyV1.Host.start_link(
             transport: BBMCUHub.Sim.Transport,
             transport_opts: [name: SegbyV1.Sim.Transport]
           )

    2. **`BBMCUHub.Sim.Driver`** — the ~50 Hz real-time loop. It reads the newest
       per-slot commands the transport captured, steps the plant, and injects the
       resulting sensor readings back UP the real stack as wire bodies to the
       `LinkOwner` (the transport's owner):

           BBMCUHub.Sim.Driver.start_link(
             owner: BBMCUHub.Host.LinkOwner,     # the registered LinkOwner name
             transport: SegbyV1.Sim.Transport,   # same name as step 1
             plant: SegbyV1.Sim.MujocoPlant,
             plant_opts: [mjcf: "…/sim/segby.xml"],
             tick_ms: 20
           )

    3. **The plant's Port** to the MuJoCo Python child (owned by the plant), which
       runs the passive viewer window.

  The host is started FIRST so the `LinkOwner` and the named transport exist; the
  driver is started only AFTER the transport name is registered (it references
  both). Then the task launches the **bb_tui dashboard in this same node, in the
  foreground** (`BB.TUI.run/2`) — which blocks until you quit, keeping the whole
  supervised tree up.

  **Why one node, not two panes.** `bb_tui` attaches to a *running* robot tree over
  that tree's PubSub registry (`SegbyV1.Robot.PubSub`). A separate `mix bb.tui`
  invocation is a *different* BEAM node with no distribution to this one, so it would
  not find the tree (`unknown registry: SegbyV1.Robot.PubSub`). Running the dashboard
  here, in the node that owns the tree, is the same-node attach the host design
  assumes (see `SegbyV1.Host`). (To attach a dashboard from a *separate* workstation
  you'd start this node named and use `mix bb.tui --node ...` — see `BB.TUI`.)

  ## Prerequisites

  The Python/MuJoCo deps must be installed once (needs network), and on macOS the
  child runs under `mjpython` (the GLFW main-thread rule):

      cd examples/segby_v1/sim && uv sync

  See `sim/README.md` for the full setup (devShell, the macOS `libpython` symlink).

  ## Operator flow (one command)

      $ mix segby.sim

  This brings up the robot tree + the sim loop + the MuJoCo viewer window, then opens
  the bb_tui dashboard in the terminal. In the dashboard: **arm** the robot (`a`),
  then run the `:teleop` command in the Commands panel with `forward` / `turn` to
  drive — the bot you see in the MuJoCo window moves. Quit the dashboard (`q`) to
  stop everything. (Enable the balance loop live from IEx — start with
  `iex -S mix segby.sim` — via `SegbyV1.Balance.enable(SegbyV1.Robot)`.)
  """

  use Mix.Task

  # The known name the SIM transport is registered under, so the driver can locate
  # it (`take_commands/1`) — the `LinkOwner` starts it and holds its pid privately
  # (ADR-0008: `Sim.Transport` accepts an optional `:name` for exactly this).
  @robot SegbyV1.Robot

  @transport_name SegbyV1.Sim.Transport

  # The driver injects sensor bodies to the owner's mailbox. The owner MUST be the
  # `LinkOwner` (so the real decode → registry → view path runs). It is registered
  # under its default name; the driver's `Kernel.send/2` accepts a registered name.
  @owner_name BBMCUHub.Host.LinkOwner

  @plant SegbyV1.Sim.MujocoPlant

  # 10 ms = 100 Hz, matching the pose sensor's declared rate (blaster `pose` is
  # 100 Hz) and its view `beat_ms: 10`. The driver steps physics + emits one fresh
  # pose per tick, so ticking at the sensor rate means EVERY balance compute sees
  # fresh data with a correct dt — ticking slower (e.g. 50 Hz under a 100 Hz view)
  # makes half the balance computes run on duplicated sensor data with a too-small
  # dt, silently mis-scaling ki/kd.
  @tick_ms 10

  @impl Mix.Task
  def run(_args) do
    # Start the app so deps (bb, jason, …) and the OTP tree are up.
    Mix.Task.run("app.start")

    mjcf = mjcf_path()
    unless File.exists?(mjcf), do: Mix.raise(missing_mjcf_message(mjcf))

    # 1) The host, with the SIM transport injected + named. This stands up the BB
    #    tree and the LinkOwner; the LinkOwner starts the named transport (owner =
    #    the LinkOwner) and registers under its default name.
    {:ok, _host} =
      SegbyV1.Host.start_link(
        transport: BBMCUHub.Sim.Transport,
        transport_opts: [name: @transport_name]
      )

    # The host's start_link returns once the LinkOwner child has started, which is
    # what registers the named transport — but wait explicitly so the driver never
    # races ahead of `take_commands/1` resolving the name.
    await_registered!(@transport_name)
    await_registered!(@owner_name)

    # 2) The driver, closing the loop. owner = the registered LinkOwner name (a
    #    valid send target); transport = the same name from step 1; plant = the
    #    example's MuJoCo plant, pointed at the MJCF.
    {:ok, _driver} =
      BBMCUHub.Sim.Driver.start_link(
        owner: @owner_name,
        transport: @transport_name,
        plant: @plant,
        plant_opts: [mjcf: mjcf],
        tick_ms: @tick_ms
      )

    # 2b) ARM the robot. It boots :disarmed (the safe default), and the balance
    #     controller publishes NOTHING while disarmed (ADR-0010), so the wheels stay
    #     silent until armed. The sim is meant to show the bot balancing and to let
    #     the operator arm/disarm from the dashboard, so we arm at boot — the panel
    #     then matches the live, balancing bot. (Disarm from the dashboard makes the
    #     wheels go silent → the bot goes limp, the real floor safe-state.)
    BB.Safety.arm(@robot)

    # 2c) Enable the balance loop. The robot's own default is DISABLED (so a real
    #     board boots passive even when armed — see SegbyV1.Robot), but the WHOLE
    #     POINT of the sim is to watch the bot balance, so the sim turns it on.
    SegbyV1.Balance.enable(@robot)

    # 2d) Start the dashboard Observer (ADR-0004): a host-side reader that samples
    #     segby's produced slots at its OWN 10 Hz and republishes `Observer.Sample`s
    #     on `[:observe]`. The TUI subscribes to `[:observe]` (not the control
    #     firehose) and `SegbyV1.ObserveRenderer` renders the samples — so the
    #     observer-plane view is live in the sim, exactly as on the Nerves host.
    #     Best-effort: if it can't start, the dashboard still runs (just without the
    #     observer panel). Started after the host so PortIndex is built + slots fill.
    start_observer()

    banner(mjcf)

    # 3) Launch the bb_tui dashboard IN THIS NODE, in the foreground. The robot tree
    #    and the dashboard MUST share a BEAM node — `mix segby.sim` and a separate
    #    `mix bb.tui` would be two nodes with no distribution, so the dashboard would
    #    not see SegbyV1.Robot's PubSub. Running it here attaches it to the
    #    already-running tree over the standard BB PubSub paths. `BB.TUI.run/2` owns
    #    stdin/stdout and blocks until the dashboard exits — which keeps the whole
    #    supervised tree (host + driver + MuJoCo Port) up until you quit.
    #
    #    subscribe_paths feeds the dashboard the slow `[:observe]` stream (the
    #    Observer above) plus the safety/state/command paths it needs — NOT the full
    #    `[:sensor]`/`[:actuator]` control firehose. The `:renderers` map teaches the
    #    TUI to render our `Observer.Sample`s via `SegbyV1.ObserveRenderer` without
    #    bb_tui knowing that struct (ADR-0004).
    BB.TUI.run(@robot,
      subscribe_paths: [
        [:observe],
        [:state_machine],
        [:param],
        [:command],
        [:safety]
      ],
      renderers: %{[:observe] => SegbyV1.ObserveRenderer}
    )
  end

  # Start the dashboard Observer that samples segby's produced slots and republishes
  # `Observer.Sample`s on `[:observe]` (ADR-0004), mirroring the Nerves host. The
  # observed slots are the OUT ports worth a glance: chassis pose + forward range,
  # both wheel statuses (applied/floored), and both wheel speeds (the ADR-0009 vel
  # sensors — handy while driving). Best-effort: a failure logs and the dashboard
  # still runs.
  defp start_observer do
    opts = [
      name: SegbyV1.DashboardObserver,
      robot: @robot,
      slots: [
        {:blaster, :pose},
        {:blaster, :range_front},
        {:wheels, :status_left},
        {:wheels, :status_right},
        {:wheels, :vel_left},
        {:wheels, :vel_right}
      ],
      sample_ms: 100,
      sink: BBMCUHub.Observer.Sink.PubSub.new(robot: @robot, topic: [:observe])
    ]

    case BBMCUHub.Observer.start_link(opts) do
      {:ok, _} -> :ok
      {:error, reason} -> Mix.shell().info("  (observer not started: #{inspect(reason)})")
    end
  end

  # The MJCF is resolved against the cwd `mix` runs in (the example app root), so
  # `mix segby.sim` from examples/segby_v1/ finds sim/segby.xml. (It is not in
  # priv/, so Application.app_dir wouldn't find it.)
  defp mjcf_path, do: Path.join(File.cwd!(), "sim/segby.xml")

  # Poll until `name` is registered (the host's children start synchronously, so
  # this returns almost immediately — it's a guard against any future async start
  # ordering, not a real wait).
  defp await_registered!(name, tries \\ 200)

  defp await_registered!(name, 0),
    do: Mix.raise("#{inspect(name)} was not registered after starting the host")

  defp await_registered!(name, tries) do
    if Process.whereis(name) do
      :ok
    else
      Process.sleep(10)
      await_registered!(name, tries - 1)
    end
  end

  defp banner(mjcf) do
    shell = Mix.shell()
    shell.info("")
    shell.info("  segby_v1 — virtual robot is UP")
    shell.info("  ─────────────────────────────────────────")
    shell.info("  plant     : #{inspect(@plant)}")
    shell.info("  mjcf      : #{mjcf}")
    shell.info("  transport : #{inspect(@transport_name)} (sim)")
    shell.info("  loop      : #{@tick_ms} ms (~#{round(1000 / @tick_ms)} Hz)")
    shell.info("  balance   : ENABLED (the sim turns it on; hardware default is off)")
    shell.info("")
    shell.info("  Starting the bb_tui dashboard in THIS terminal (same node as the")
    shell.info("  robot tree). A MuJoCo viewer window opens separately.")
    shell.info("")
    shell.info("  In the dashboard: ARM (a), then run the :teleop command (Commands")
    shell.info("  panel) with forward/turn to drive — the bot moves in the MuJoCo")
    shell.info("  window. Quit the dashboard (q) to stop everything.")
    shell.info("")
  end

  defp missing_mjcf_message(mjcf) do
    """
    Could not find the MuJoCo model at:

        #{mjcf}

    Run `mix segby.sim` from the example app root (examples/segby_v1/), where
    sim/segby.xml lives. If the deps are not installed yet:

        cd sim && uv sync
    """
  end
end
