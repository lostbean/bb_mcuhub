defmodule BBMCUHub.Sim.Driver do
  @moduledoc """
  The real-time loop that **closes the sim loop** for a virtual robot (ADR-0008).

  The transport is the one and only hardware boundary: above it is the real host
  stack, below it the sim stands in for the wire, the hubs, the floors, the
  silicon, and the physical plant. `BBMCUHub.Sim.Transport` captures the outbound
  per-slot commands (a pure capture — no clock); **this Driver owns the loop** that
  reads them, advances the plant, and injects the resulting sensors back up the
  real stack as wire bodies. The loop lives here, NOT in the transport, so the
  transport's `send/2` stays a pure capture out of the `LinkOwner`'s call path and
  the loop+plant are independently restartable (ADR-0008).

  ## The tick (default ~20 ms / 50 Hz)

      cmds          = Sim.Transport.take_commands(transport)   # newest per slot
      {sensors, st} = plant.step(cmds, dt_s, plant_state)      # advance physics
      for {node, port_id, type, value} <- sensors:
        body = Codec.encode_body(node, port_id, seq, t_dev, type, value, stamped?)
        send(owner, {:circuits_uart, :sim, body})              # inject up the stack

  `{:circuits_uart, :sim, body}` is **exactly** the inbound message shape the UART
  transport delivers, so the real `LinkOwner` decodes it unchanged: it
  `Codec.decode_body/1`s the body, writes the registry slot, the sensor view
  witnesses a fresh `seq` advance, and the published `BB.Message` flows to `bb_tui`
  and the control laws — every box above the transport is shipped code.

  ## Why `seq` must strictly advance (born-stale freshness)

  The host's freshness monitor is **born-stale**: a sensor view publishes NOTHING
  until it witnesses `seq` ADVANCE since its own boot (the floor design / ADR-0008).
  So the Driver keeps a **per-slot `seq` counter that strictly increases** across
  ticks — if it reused a `seq`, the view would treat the reading as stale and the
  bot would never appear to move. The counter wraps `0xFFFF → 0` (the advance test
  is plain inequality, and a wrap-around step from `0xFFFF` to `0` is still an
  advance for the inequality check at the boundary the view actually uses — it
  compares against the last seq it saw, not a global monotone, so successive sends
  always look fresh as long as they differ from the immediately preceding one).

  ## Determinism: `dt`, `t_dev`, and `seq` are derived from the tick count

  The live loop uses `Process.send_after` for **wall-clock pacing** (a human is
  watching), but every *simulated* quantity is derived from the tick count and the
  tick period, **never** the wall clock:

    * `dt_s` is the fixed tick period in seconds (`tick_ms / 1000`).
    * `t_dev` for a stamped port is the accumulated simulated time in microseconds
      (`tick_count * tick_us`), so it advances monotonically and reproducibly. An
      unstamped port passes `0` (the codec ignores it).
    * `seq` is a per-slot counter incremented each time that slot is emitted.

  So the *sequence* of physics inputs is reproducible; only *when* ticks fire is
  wall-clock. A future deterministic test mode can drive the same plant on an
  explicit step count without changing the recipe.

  ## Robustness

  A malformed value-type value (an `encode_fields` `Map.fetch!` raising on a
  missing layout field) is caught **per sensor**, logged, and skipped — never
  crashing the loop, mirroring `LinkOwner.send_value/6`. On terminate the Driver
  calls `plant.close/1` so a plant releasing a `Port` or file shuts down cleanly.
  """
  use GenServer

  require Logger

  alias BBMCUHub.Contract.PortIndex
  alias BBMCUHub.Wire.Codec

  @default_tick_ms 20

  # The inbound tag the owner sees on each injected body. The UART transport tags
  # its inbound `{:circuits_uart, port, body}` with the serial port; the sim uses
  # the fixed `:sim` atom — the `LinkOwner` ignores the tag, so any atom decodes
  # the same, but `:sim` makes the source legible.
  @inbound_tag :sim

  # --- API ---

  @doc """
  Start the sim loop.

  Options:
    * `:owner` — the pid that receives injected sensor bodies (the `LinkOwner`, or
      `BBMCUHub.Sim.Transport.owner/1`). Required.
    * `:transport` — the `BBMCUHub.Sim.Transport` to read captured commands from.
      Required.
    * `:plant` — the `BBMCUHub.Sim.Plant` module to drive. Required.
    * `:plant_opts` — keyword passed to `plant.init/1` (default `[]`).
    * `:tick_ms` — the loop period in milliseconds (default `#{@default_tick_ms}`).
    * `:name` — an optional process name.
  """
  @spec start_link(keyword()) :: GenServer.on_start()
  def start_link(opts) do
    {name, opts} = Keyword.pop(opts, :name)

    if name,
      do: GenServer.start_link(__MODULE__, opts, name: name),
      else: GenServer.start_link(__MODULE__, opts)
  end

  # --- GenServer ---

  @impl true
  def init(opts) do
    owner = Keyword.fetch!(opts, :owner)
    transport = Keyword.fetch!(opts, :transport)
    plant = Keyword.fetch!(opts, :plant)
    plant_opts = Keyword.get(opts, :plant_opts, [])
    tick_ms = Keyword.get(opts, :tick_ms, @default_tick_ms)

    {:ok, plant_state} = plant.init(plant_opts)

    schedule_tick(tick_ms)

    {:ok,
     %{
       owner: owner,
       transport: transport,
       plant: plant,
       plant_state: plant_state,
       tick_ms: tick_ms,
       dt_s: tick_ms / 1000,
       tick_us: tick_ms * 1000,
       tick_count: 0,
       seqs: %{}
     }}
  end

  @impl true
  def handle_info(:tick, st) do
    commands = BBMCUHub.Sim.Transport.take_commands(st.transport)
    {sensors, plant_state} = st.plant.step(commands, st.dt_s, st.plant_state)

    # t_dev is the accumulated simulated time at this tick (in microseconds), so it
    # advances monotonically and is derived from the tick count, never the clock.
    tick_count = st.tick_count + 1
    t_dev = tick_count * st.tick_us

    seqs = Enum.reduce(sensors, st.seqs, &inject_sensor(&1, &2, st.owner, t_dev))

    schedule_tick(st.tick_ms)

    {:noreply, %{st | plant_state: plant_state, tick_count: tick_count, seqs: seqs}}
  end

  @impl true
  def terminate(_reason, st) do
    st.plant.close(st.plant_state)
    :ok
  end

  # --- internals ---

  defp schedule_tick(tick_ms), do: Process.send_after(self(), :tick, tick_ms)

  # Encode one sensor into a wire body and inject it to the owner, advancing this
  # slot's seq. DEFENSIVE (mirrors LinkOwner.send_value/6): a malformed value-type
  # value makes encode_fields' Map.fetch! raise — it is caught, logged, and the
  # sensor skipped, so one bad reading never crashes the loop. The slot's seq still
  # advances so the NEXT good reading on that slot is witnessed as fresh.
  defp inject_sensor({node, port_id, type, value}, seqs, owner, t_dev) do
    slot = {node, port_id}
    seq = Map.get(seqs, slot, 0)
    stamped? = PortIndex.stamped?(node, port_id)
    t_dev = if stamped?, do: t_dev, else: 0

    try do
      body = Codec.encode_body(node, port_id, seq, t_dev, type, value, stamped?)
      Kernel.send(owner, {:circuits_uart, @inbound_tag, body})
    rescue
      e ->
        Logger.warning(
          "sim sensor for #{inspect(slot)} could not be encoded " <>
            "(#{Exception.message(e)}) — skipped this tick; check the plant's value"
        )
    end

    Map.put(seqs, slot, next_seq(seq))
  end

  # Strictly-increasing per-slot seq, wrapping 0xFFFF -> 0. The born-stale view
  # only needs each emission to differ from the immediately preceding one, which
  # holds across the wrap boundary too.
  defp next_seq(0xFFFF), do: 0
  defp next_seq(seq), do: seq + 1
end
