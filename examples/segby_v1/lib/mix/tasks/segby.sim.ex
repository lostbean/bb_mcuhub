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
  both). Then the task blocks (`Process.sleep(:infinity)`) so the supervised tree
  stays up until Ctrl-C, like a server.

  ## Prerequisites

  The Python/MuJoCo deps must be installed once (needs network), and on macOS the
  child runs under `mjpython` (the GLFW main-thread rule):

      cd examples/segby_v1/sim && uv sync

  See `sim/README.md` for the full setup (devShell, the macOS `libpython` symlink).

  ## Operator flow (two panes)

      # pane 1 — the virtual robot (this task; long-running, owns the MuJoCo window)
      $ mix segby.sim

      # pane 2 — the dashboard (attaches over BB PubSub, exactly as on a real bot)
      $ mix bb.tui --robot SegbyV1.Robot

  In `bb.tui`: **arm** the robot (`a`), then run the `:teleop` command in the
  Commands panel with `forward` / `turn` to drive — the bot you see in the MuJoCo
  window moves. (Enable the balance loop live from IEx with
  `SegbyV1.Balance.enable(SegbyV1.Robot)`.)
  """

  use Mix.Task

  # The known name the SIM transport is registered under, so the driver can locate
  # it (`take_commands/1`) — the `LinkOwner` starts it and holds its pid privately
  # (ADR-0008: `Sim.Transport` accepts an optional `:name` for exactly this).
  @transport_name SegbyV1.Sim.Transport

  # The driver injects sensor bodies to the owner's mailbox. The owner MUST be the
  # `LinkOwner` (so the real decode → registry → view path runs). It is registered
  # under its default name; the driver's `Kernel.send/2` accepts a registered name.
  @owner_name BBMCUHub.Host.LinkOwner

  @plant SegbyV1.Sim.MujocoPlant

  # ~50 Hz, matching the balance/control cadence (the driver's default, made
  # explicit here so the operator sees the loop rate).
  @tick_ms 20

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

    banner(mjcf)

    # 3) Block forever so the supervised tree (host + driver + MuJoCo Port) stays
    #    up until Ctrl-C, like a server.
    Process.sleep(:infinity)
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
    shell.info("  segby_v1 — virtual robot is UP (ADR-0008)")
    shell.info("  ─────────────────────────────────────────")
    shell.info("  plant     : #{inspect(@plant)}")
    shell.info("  mjcf      : #{mjcf}")
    shell.info("  transport : #{inspect(@transport_name)} (sim)")
    shell.info("  loop      : #{@tick_ms} ms (~#{round(1000 / @tick_ms)} Hz)")
    shell.info("")
    shell.info("  Next — open a SECOND terminal and attach the dashboard:")
    shell.info("")
    shell.info("      mix bb.tui --robot SegbyV1.Robot")
    shell.info("")
    shell.info("  Then in bb.tui: ARM (a), run the :teleop command (Commands panel)")
    shell.info("  with forward/turn to drive — the bot moves in the MuJoCo window.")
    shell.info("  (Enable balance live: SegbyV1.Balance.enable(SegbyV1.Robot))")
    shell.info("")
    shell.info("  Ctrl-C twice to stop.")
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
