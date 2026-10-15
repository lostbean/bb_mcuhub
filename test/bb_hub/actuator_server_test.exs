defmodule BBMCUHub.BBHub.ActuatorServerTest do
  use ExUnit.Case, async: false

  alias BBMCUHub.Contract.PortIndex
  alias BBMCUHub.Host.NodeRegistry
  alias BBMCUHub.Test.Fixtures.Robot

  @path [:base_link, :drive_joint, :drive]
  @has_epochs Code.ensure_loaded?(BB.Safety) and function_exported?(BB.Safety, :epoch, 1)

  setup do
    NodeRegistry.reset()
    PortIndex.build(Robot)
    start_supervised!(%{id: BB.Supervisor, start: {BB.Supervisor, :start_link, [Robot]}})
    {:ok, slot} = PortIndex.resolve(:act_hub, :effort_cmd)
    %{slot: slot}
  end

  test "each accepted broadcast writes the command slot exactly once", %{slot: slot} do
    :ok = BB.Safety.arm(Robot)

    assert :ok = BB.Actuator.set_effort(Robot, :drive, 0.25, delivery: :broadcast)
    flush_commands()
    assert {%{nm: 0.25}, 1, _} = read_slot(slot)

    assert :ok = BB.Actuator.set_effort(Robot, :drive, -0.5, delivery: :broadcast)
    flush_commands()
    assert {%{nm: -0.5}, 2, _} = read_slot(slot)
  end

  test "an unsupported payload never reaches the effort port", %{slot: slot} do
    :ok = BB.Safety.arm(Robot)

    assert :ok =
             BB.Actuator.set_position(Robot, :drive, 0.5, delivery: :broadcast)

    flush_commands()
    assert read_slot(slot) == nil
    assert Process.alive?(BB.Process.whereis(Robot, :drive))
  end

  test "a disarmed robot leaves the command slot untouched", %{slot: slot} do
    :ok = BB.Actuator.set_effort(Robot, :drive, 0.25, delivery: :broadcast)
    flush_commands()
    assert read_slot(slot) == nil

    :ok = BB.Safety.arm(Robot)
    :ok = BB.Actuator.set_effort(Robot, :drive, 0.5, delivery: :broadcast)
    flush_commands()
    accepted = read_slot(slot)

    :ok = BB.Safety.disarm(Robot)
    :ok = BB.Actuator.set_effort(Robot, :drive, -0.5, delivery: :broadcast)
    flush_commands()
    assert read_slot(slot) == accepted
  end

  # Arm epochs were introduced after the minimum supported bb release.
  @tag skip: not @has_epochs
  test "re-arm rejects an old command and accepts a fresh one exactly once", %{slot: slot} do
    BB.subscribe(Robot, [:actuator | @path])
    :ok = BB.Safety.arm(Robot)
    :ok = BB.Actuator.set_effort(Robot, :drive, 0.25, delivery: :broadcast)
    assert_receive {:bb, [:actuator | @path], %BB.Message{} = previous}
    flush_commands()
    assert {%{nm: 0.25}, 1, _} = accepted = read_slot(slot)

    :ok = BB.Safety.disarm(Robot)
    :ok = BB.Safety.arm(Robot)
    BB.publish(Robot, [:actuator | @path], previous)
    flush_commands()
    assert read_slot(slot) == accepted

    :ok = BB.Actuator.set_effort(Robot, :drive, -0.5, delivery: :broadcast)
    flush_commands()
    assert {%{nm: -0.5}, 2, _} = read_slot(slot)
  end

  @tag skip: not @has_epochs
  test "an unstamped command cannot write an armed robot's slot", %{slot: slot} do
    :ok = BB.Safety.arm(Robot)

    BB.publish(Robot, [:actuator | @path], %BB.Message{
      payload: %BB.Message.Actuator.Command.Effort{effort: 0.25}
    })

    flush_commands()
    assert read_slot(slot) == nil
  end

  defp read_slot({node, port}), do: NodeRegistry.get(node, port)

  # PubSub delivers from the publishing process. This call waits for its earlier
  # messages to be handled; assertions use the public command slot, not server state.
  defp flush_commands, do: :sys.get_state(BB.Process.whereis(Robot, :drive))
end
