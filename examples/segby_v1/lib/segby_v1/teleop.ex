defmodule SegbyV1.Teleop do
  @moduledoc """
  The segby_v1 operator-teleop command (§09) — the bridge from `bb_tui`'s
  operator surface into the balance controller's teleop loop.

  `bb_tui` has no built-in "teleop" concept; its write surface is the declared
  `commands` panel (run via `BB.Robot.Runtime.execute/3`), the joints panel
  (`BB.Actuator.set_position!/3`), and arm/disarm. Driving the wheels straight
  from the joints panel would fight `SegbyV1.Balance`, which is the *sole*
  commander of the wheel actuators (the balance loop owns the wheels, §04
  single-writer). So operator drive is surfaced as THIS command instead.

  When `bb_tui`'s Commands panel runs `:teleop` with `forward` / `turn` floats
  (each clamped to `[-1.0, 1.0]` by the controller), this handler publishes a
  `BB.Message.Geometry.Twist` (`linear.x` = forward, `angular.z` = turn, the
  ROS-style convention) onto the balance controller's teleop topic
  (`[:teleop, :segby]`). The balance controller consumes that Twist
  (ADR-0009 + amendment): `forward` is a per-wheel **target SPEED** for its inner
  velocity loop (roll forward at a bounded rate, scaled by `max_speed`), and `turn`
  is a **target YAW RATE** (chassis yaw rate, scaled by `max_yaw_rate`) closed-loop
  on the IMU's measured yaw — NOT a torque bias. The controller drives the forward
  velocity term `kv·(target − measured)` and the differential yaw term
  `±kyaw·(target_yaw_rate − gyro_z)` onto the balance torque each pose tick. The
  command completes immediately — it is a single fire-and-forget intent update, not
  a long-running motion.

  Wired into segby's `commands do` block with `allowed_states [:*]` so an
  operator can teleop in any non-disarmed operational state.
  """
  use BB.Command

  alias BB.Math.Vec3
  alias BB.Message.Geometry.Twist

  # The topic the balance controller subscribes to for teleop intent. Kept in
  # one place so the command and the controller agree (the controller's default
  # `:teleop_topic`).
  @teleop_topic [:teleop, :segby]

  @doc "The teleop intent topic this command publishes onto."
  @spec teleop_topic() :: [atom()]
  def teleop_topic, do: @teleop_topic

  @impl BB.Command
  def handle_command(goal, context, state) do
    forward = (goal[:forward] || 0.0) * 1.0
    turn = (goal[:turn] || 0.0) * 1.0

    {:ok, msg} =
      Twist.new(:teleop, Vec3.new(forward, 0.0, 0.0), Vec3.new(0.0, 0.0, turn))

    BB.publish(context.robot_module, @teleop_topic, msg)

    {:stop, :normal, %{state | result: {:ok, %{forward: forward, turn: turn}}}}
  end

  @impl BB.Command
  def result(%{result: result}), do: result
end
