# Test-only plants for the Driver, defined as top-level modules (above the test
# module) so they resolve unambiguously when passed as the `:plant` argument. The
# shared `BBMCUHub.Test.SimStubPlant`'s hard-coded slot {0x02, 0x00} does NOT
# resolve in the fixture PortIndex (pose is hash-assigned at {2, 113}), so to
# exercise the REAL codec round-trip these plants emit on a caller-chosen,
# fixture-real slot/type. See ADR-0008.

defmodule BBMCUHub.Sim.DriverTest.TestPlant do
  @moduledoc false
  # Emits one sensor on a caller-chosen slot/type, folding the summed command
  # signal into `ax` so a test can assert commands flow. Carries a step counter so
  # its `wx` output advances tick-over-tick.
  @behaviour BBMCUHub.Sim.Plant

  @impl true
  def init(opts) do
    {:ok, %{step: 0, slot: Keyword.fetch!(opts, :slot), type: Keyword.fetch!(opts, :type)}}
  end

  @impl true
  def step(commands, dt_s, %{step: n, slot: {node, port}, type: type} = st) do
    cmd_signal = commands |> Map.values() |> Enum.flat_map(&Map.values/1) |> Enum.sum()

    value = %{
      qw: 1.0,
      qx: 0.0,
      qy: 0.0,
      qz: 0.0,
      wx: n * dt_s,
      wy: 0.0,
      wz: 0.0,
      ax: cmd_signal,
      ay: 0.0,
      az: 0.0
    }

    {[{node, port, type, value}], %{st | step: n + 1}}
  end

  @impl true
  def close(_state), do: :ok
end

defmodule BBMCUHub.Sim.DriverTest.BadThenGoodPlant do
  @moduledoc false
  # Emits TWO sensors per step: one malformed (an :imu value missing every field,
  # so encode raises) and one good — both on the good slot/type. Proves the
  # per-sensor rescue isolates the failure without losing the good reading.
  @behaviour BBMCUHub.Sim.Plant

  @impl true
  def init(opts) do
    {:ok,
     %{good_slot: Keyword.fetch!(opts, :good_slot), good_type: Keyword.fetch!(opts, :good_type)}}
  end

  @impl true
  def step(_commands, _dt_s, %{good_slot: {gn, gp}, good_type: gtype} = st) do
    good = %{
      qw: 1.0,
      qx: 0.0,
      qy: 0.0,
      qz: 0.0,
      wx: 0.0,
      wy: 0.0,
      wz: 0.0,
      ax: 0.0,
      ay: 0.0,
      az: 0.0
    }

    # The bad sensor targets the SAME slot/type but with an empty value map, so
    # encode_fields' Map.fetch! raises mid-loop — the loop must rescue and skip it
    # while still delivering the good one.
    sensors = [
      {gn, gp, gtype, %{}},
      {gn, gp, gtype, good}
    ]

    {sensors, st}
  end

  @impl true
  def close(_state), do: :ok
end

defmodule BBMCUHub.Sim.DriverTest.ClosingPlant do
  @moduledoc false
  # Records its close/1 call by messaging a test pid, so a test can assert the
  # Driver calls plant.close on terminate.
  @behaviour BBMCUHub.Sim.Plant

  @impl true
  def init(opts) do
    {:ok,
     %{
       notify: Keyword.fetch!(opts, :notify),
       slot: Keyword.fetch!(opts, :slot),
       type: Keyword.fetch!(opts, :type)
     }}
  end

  @impl true
  def step(_commands, _dt_s, %{slot: {node, port}, type: type} = st) do
    value = %{
      qw: 1.0,
      qx: 0.0,
      qy: 0.0,
      qz: 0.0,
      wx: 0.0,
      wy: 0.0,
      wz: 0.0,
      ax: 0.0,
      ay: 0.0,
      az: 0.0
    }

    {[{node, port, type, value}], st}
  end

  @impl true
  def close(%{notify: notify} = state) do
    Kernel.send(notify, {:plant_closed, state})
    :ok
  end
end

defmodule BBMCUHub.Sim.DriverTest do
  use ExUnit.Case, async: false

  # The Driver is the real-time loop that closes the sim loop (ADR-0008): it reads
  # the transport's captured commands, steps a plant, and injects the returned
  # sensors back up the REAL host stack as wire bodies — `{:circuits_uart, :sim,
  # body}` to the owner, exactly the shape the UART transport delivers inbound, so
  # `LinkOwner`/`Codec.decode_body` consume them unchanged.
  #
  # These tests observe the loop through that injected-body channel: the test
  # process is the owner, so it receives the injected bodies directly and decodes
  # them with the real codec.

  alias BBMCUHub.Contract.PortIndex
  alias BBMCUHub.Sim.{Driver, Transport}
  alias BBMCUHub.Sim.DriverTest.{BadThenGoodPlant, ClosingPlant, TestPlant}
  alias BBMCUHub.Wire.Codec

  setup_all do
    PortIndex.build(BBMCUHub.Test.Fixtures.Robot)
    :ok
  end

  setup do
    {:ok, t} = Transport.start_link(self(), [])
    on_exit(fn -> if Process.alive?(t), do: Transport.close(t) end)
    {:ok, transport: t}
  end

  # The fixture's real STAMPED sensor slot (an :imu carrying t_dev).
  defp pose_slot do
    {:ok, slot} = PortIndex.resolve(:sensor_hub, :pose)
    slot
  end

  defp effort_slot do
    {:ok, slot} = PortIndex.resolve(:act_hub, :effort_cmd)
    slot
  end

  # Start a Driver whose owner is the test process, on a small tick so wall-clock
  # pacing delivers messages promptly without flaking (we only assert "a message
  # arrived", which `assert_receive` with a timeout handles robustly).
  defp start_driver(transport, plant, plant_opts, tick_ms \\ 5) do
    {:ok, d} =
      Driver.start_link(
        owner: self(),
        transport: transport,
        plant: plant,
        plant_opts: plant_opts,
        tick_ms: tick_ms
      )

    on_exit(fn -> if Process.alive?(d), do: GenServer.stop(d) end)
    d
  end

  test "injects each plant sensor as a decodable wire body to the owner", %{transport: t} do
    {node, port_id} = pose_slot()
    start_driver(t, TestPlant, slot: {node, port_id}, type: :imu)

    assert_receive {:circuits_uart, :sim, body}, 500

    assert {:ok, decoded} = Codec.decode_body(body)
    assert decoded.node == node
    assert decoded.port_id == port_id
    assert decoded.type == :imu
    assert is_map(decoded.value)
    # the value round-trips through the codec layout
    assert Map.has_key?(decoded.value, :wx)
  end

  test "per-slot seq strictly advances across ticks (born-stale freshness passes)", %{
    transport: t
  } do
    start_driver(t, TestPlant, slot: pose_slot(), type: :imu)

    assert_receive {:circuits_uart, :sim, body1}, 500
    assert_receive {:circuits_uart, :sim, body2}, 500
    assert_receive {:circuits_uart, :sim, body3}, 500

    {:ok, d1} = Codec.decode_body(body1)
    {:ok, d2} = Codec.decode_body(body2)
    {:ok, d3} = Codec.decode_body(body3)

    # plain inequality / strict increase — the freshness monitor only needs to
    # witness seq ADVANCE since its boot to publish.
    assert d2.seq > d1.seq
    assert d3.seq > d2.seq
  end

  test "stamped port carries a monotonically increasing, non-wall-clock t_dev", %{transport: t} do
    start_driver(t, TestPlant, slot: pose_slot(), type: :imu)

    assert_receive {:circuits_uart, :sim, body1}, 500
    assert_receive {:circuits_uart, :sim, body2}, 500

    {:ok, d1} = Codec.decode_body(body1)
    {:ok, d2} = Codec.decode_body(body2)

    # pose is stamped, so t_dev is present and advances tick-over-tick.
    assert is_integer(d1.t_dev)
    assert d2.t_dev > d1.t_dev
  end

  test "commands flow transport -> plant -> sensors", %{transport: t} do
    {node, port_id} = pose_slot()
    start_driver(t, TestPlant, slot: {node, port_id}, type: :imu)

    # Capture an idle reading before any command is in flight.
    assert_receive {:circuits_uart, :sim, idle_body}, 500
    {:ok, idle} = Codec.decode_body(idle_body)

    # Send a real encoded effort command into the transport (newest captured wins).
    {enode, eport} = effort_slot()
    cmd_body = Codec.encode_body(enode, eport, 1, 0, :effort, %{nm: 7.0})
    assert :ok = Transport.send(t, cmd_body)

    # On a later tick the plant must have seen the command and folded it into its
    # output: TestPlant places the summed command signal into `ax`.
    driven =
      Enum.reduce_while(1..50, nil, fn _i, _acc ->
        receive do
          {:circuits_uart, :sim, body} ->
            {:ok, d} = Codec.decode_body(body)
            if d.value.ax != idle.value.ax, do: {:halt, d}, else: {:cont, nil}
        after
          500 -> {:halt, nil}
        end
      end)

    assert driven, "expected a sensor reflecting the injected command"
    assert_in_delta driven.value.ax, 7.0, 1.0e-4
  end

  test "encode failures are caught and skipped, never crashing the loop", %{transport: t} do
    # A malformed value (missing layout fields) makes encode_fields' Map.fetch!
    # raise; the loop must rescue+skip and keep ticking. Mix one bad sensor with a
    # good one and assert the good one still arrives, and the driver stays alive.
    {node, port_id} = pose_slot()

    d = start_driver(t, BadThenGoodPlant, good_slot: {node, port_id}, good_type: :imu)

    assert_receive {:circuits_uart, :sim, body}, 500
    assert {:ok, decoded} = Codec.decode_body(body)
    assert decoded.node == node
    assert Process.alive?(d)
  end

  test "close/terminate stops cleanly and calls plant.close", %{transport: t} do
    # ClosingPlant records its close by sending a message to the test process.
    d = start_driver(t, ClosingPlant, notify: self(), slot: pose_slot(), type: :imu)

    assert_receive {:circuits_uart, :sim, _body}, 500

    :ok = GenServer.stop(d)
    refute Process.alive?(d)
    assert_receive {:plant_closed, _state}, 500
  end
end
