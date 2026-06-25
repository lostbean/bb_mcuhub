defmodule BBMCUHub.Sim.TransportTest do
  use ExUnit.Case, async: true

  alias BBMCUHub.Contract.PortIndex
  alias BBMCUHub.Sim.Transport
  alias BBMCUHub.Wire.Codec

  setup_all do
    PortIndex.build(BBMCUHub.Test.Fixtures.Robot)
    :ok
  end

  setup do
    {:ok, t} = Transport.start_link(self(), [])
    on_exit(fn -> if Process.alive?(t), do: Transport.close(t) end)
    {:ok, t: t}
  end

  defp effort_body(value, seq \\ 1) do
    {:ok, {node, port_id}} = PortIndex.resolve(:act_hub, :effort_cmd)
    {{node, port_id}, Codec.encode_body(node, port_id, seq, 0, :effort, value)}
  end

  test "records the owner pid at start_link", %{t: t} do
    assert Transport.owner(t) == self()
  end

  test "captures a single command keyed by {node, port_id}", %{t: t} do
    {slot, body} = effort_body(%{nm: 0.25})
    assert :ok = Transport.send(t, body)

    commands = Transport.take_commands(t)
    assert Map.keys(commands) == [slot]
    assert_in_delta commands[slot].nm, 0.25, 1.0e-6
  end

  test "newest-per-slot wins: a later send for the same slot overwrites", %{t: t} do
    {slot, body1} = effort_body(%{nm: 0.10}, 1)
    {^slot, body2} = effort_body(%{nm: 0.90}, 2)

    assert :ok = Transport.send(t, body1)
    assert :ok = Transport.send(t, body2)

    commands = Transport.take_commands(t)
    assert Map.keys(commands) == [slot]
    assert_in_delta commands[slot].nm, 0.90, 1.0e-6
  end

  test "captures distinct slots independently", %{t: t} do
    {effort_slot, effort} = effort_body(%{nm: 0.5})

    {:ok, {pnode, pport}} = PortIndex.resolve(:sensor_hub, :pose)
    pose_slot = {pnode, pport}

    pose_value = %{
      qw: 1.0,
      qx: 0.0,
      qy: 0.0,
      qz: 0.0,
      wx: 0.0,
      wy: 0.0,
      wz: 0.0,
      ax: 0.0,
      ay: 0.0,
      az: 9.81
    }

    pose = Codec.encode_body(pnode, pport, 1, 1234, :imu, pose_value, true)

    assert :ok = Transport.send(t, effort)
    assert :ok = Transport.send(t, pose)

    commands = Transport.take_commands(t)
    assert MapSet.new(Map.keys(commands)) == MapSet.new([effort_slot, pose_slot])
    assert_in_delta commands[effort_slot].nm, 0.5, 1.0e-6
    assert_in_delta commands[pose_slot].az, 9.81, 1.0e-5
  end

  test "take_commands is a non-clearing snapshot: a slot persists across reads", %{t: t} do
    {slot, body} = effort_body(%{nm: 0.42})
    assert :ok = Transport.send(t, body)

    first = Transport.take_commands(t)
    second = Transport.take_commands(t)

    assert Map.keys(first) == [slot]
    assert Map.keys(second) == [slot]
    assert_in_delta second[slot].nm, 0.42, 1.0e-6
  end

  test "undecodable body does not crash and adds no slot", %{t: t} do
    assert :ok = Transport.send(t, <<0, 0, 0>>)
    assert Transport.take_commands(t) == %{}

    # the transport is still alive and functioning after the garbage
    {slot, body} = effort_body(%{nm: 0.33})
    assert :ok = Transport.send(t, body)
    assert Map.keys(Transport.take_commands(t)) == [slot]
  end

  test "close/1 stops the transport", %{t: t} do
    assert :ok = Transport.close(t)
    refute Process.alive?(t)
  end

  test "an optional :name registers the transport so it can be reached by name" do
    name = :"sim_transport_#{System.unique_integer([:positive])}"
    {:ok, t} = Transport.start_link(self(), name: name)
    on_exit(fn -> if Process.alive?(t), do: Transport.close(t) end)

    # the named transport answers the sim seam by its name, not just its pid
    assert Transport.owner(name) == self()
    assert Transport.take_commands(name) == %{}
  end
end
