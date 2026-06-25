defmodule BBMCUHub.Sim.PlantTest do
  use ExUnit.Case, async: true

  # Drives the deterministic stub plant THROUGH the behaviour contract
  # (BBMCUHub.Sim.Plant), never through stub internals — see ADR-0008. The plant
  # speaks value-type values keyed by wire slot (ADR-0005), and step/3 returns
  # `{node, port_id, type, value}` sensor tuples — exactly the shape the future
  # Sim.Driver hands to Codec.encode_body.
  @plant BBMCUHub.Test.SimStubPlant

  test "init/1 returns {:ok, state} whose state is steppable" do
    assert {:ok, state} = @plant.init([])
    # The state is opaque to us; the contract is only that it can be threaded into
    # step/3. Exercising it (rather than poking at internals) proves init produced
    # a usable plant state.
    assert {_sensors, _next} = @plant.step(%{}, 0.02, state)
  end

  test "step/3 returns sensors in the {node, port_id, type, value} wire shape" do
    {:ok, state} = @plant.init([])

    {sensors, _state} = @plant.step(%{}, 0.02, state)

    assert is_list(sensors)
    # The driver injects one wire body per returned sensor, so a step must produce
    # at least one reading for the loop below to witness anything.
    assert length(sensors) >= 1

    for sensor <- sensors do
      assert {node, port_id, type, value} = sensor
      assert node in 0..255
      assert port_id in 0..255
      assert is_atom(type)
      assert is_map(value)
      assert Enum.all?(Map.keys(value), &is_atom/1)
      assert Enum.all?(Map.values(value), &is_number/1)
    end
  end

  test "step/3 accepts per-slot commands keyed by {node, port_id}" do
    {:ok, state} = @plant.init([])

    commands = %{{0x02, 0} => %{nm: 0.5}}

    assert {sensors, _state} = @plant.step(commands, 0.02, state)
    assert is_list(sensors)
  end

  test "state advances across successive steps (the plant carries state forward)" do
    {:ok, state0} = @plant.init([])

    {sensors1, state1} = @plant.step(%{}, 0.02, state0)
    {sensors2, _state2} = @plant.step(%{}, 0.02, state1)

    # Stepping the SAME advanced state must differ from re-stepping the initial
    # state — the contract is that step/3 carries simulated state forward, so a
    # second step is not identical to the first.
    {sensors_replay, _} = @plant.step(%{}, 0.02, state0)

    assert sensors1 == sensors_replay
    assert sensors2 != sensors1
  end

  test "the latest command value is reflected in the sensors the plant produces" do
    {:ok, state} = @plant.init([])

    {sensors_idle, _} = @plant.step(%{}, 0.02, state)
    {sensors_driven, _} = @plant.step(%{{0x02, 0} => %{nm: 1.0}}, 0.02, state)

    # Commands flow in and influence the produced sensors (so a later chunk can
    # assert a command had an effect). The exact mapping is a stub detail; that
    # idle and driven differ is the contract we lean on.
    assert sensors_idle != sensors_driven
  end

  test "close/1 returns :ok and is idempotent" do
    {:ok, state} = @plant.init([])

    assert :ok = @plant.close(state)
    assert :ok = @plant.close(state)
  end
end
