defmodule SegbyV1.ValueTypes.WheelSpeed do
  @moduledoc """
  The `wheel_speed` value-type (ADR-0009) — a wheel's MEASURED shaft angular
  velocity, a single `:f32` in rad/s.

  This is a CONSUMER-defined value-type (`use BBMCUHub.ValueType`): the library
  ships only `imu`/`effort`/`status` as stock, and segby adds its own wire
  vocabulary here with no library edit — the extension seam ADR-0003 records, the
  same one `Range`/`Led` ride. A port names it BY MODULE
  (`type: SegbyV1.ValueTypes.WheelSpeed`), which `BBMCUHub.ValueType.resolve/1`
  passes through unchanged. This exercises the library's extensibility; it does
  not change it (ADR-0009: no `lib/bb_mcuhub` edit).

  It is a MEASUREMENT, not a command: on the real bot it is the wheels' FOC
  `shaft_velocity` (the closed-loop velocity drive already computes it); in the
  MuJoCo sim it is the wheel hinge `qvel`. It rides the wheels hub's `vel_left`/
  `vel_right` SENSOR ports (`dir: :out`) and is published on `[:sensor | …]` via
  the stock `BBMCUHub.BBHub.Sensor` view — the same seam the IMU pose uses — so
  the host balance loop's inner velocity loop can read measured wheel velocity.

  Unlike `Range` (whose sensor view is never made fresh, so it never actually
  publishes and gets away with an identity-map lift), the velocity sensor view IS
  fresh every tick — so it really publishes, and `BB.PubSub` requires a typed
  `BB.Message` payload **struct**, not a raw map. So `lift/1` produces a
  `BB.Message.Sensor.JointState` — the standard BeamBots joint-state message —
  carrying this one wheel's velocity (`velocities: [rad_s]`). The host balance
  loop's inner velocity loop reads `velocities` from it.
  """
  use BBMCUHub.ValueType

  # The joint name on the lifted JointState. A single-wheel message; the consuming
  # controller reads the (single) velocity, not the name — but JointState requires
  # a name, so carry a stable one.
  @joint_name :wheel

  layout(
    # measured wheel shaft angular velocity — rad/s (ADR-0009)
    rad_s: :f32
  )

  @impl BBMCUHub.ValueType
  def lift(%{rad_s: rad_s}) do
    %BB.Message.Sensor.JointState{
      names: [@joint_name],
      positions: [],
      velocities: [rad_s * 1.0],
      efforts: []
    }
  end

  @impl BBMCUHub.ValueType
  def unlift(%BB.Message.Sensor.JointState{velocities: [rad_s | _]}), do: %{rad_s: rad_s}
end
