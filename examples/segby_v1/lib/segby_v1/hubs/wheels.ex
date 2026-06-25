defmodule SegbyV1.Hubs.Wheels do
  @moduledoc """
  The Wheels hub (§09) — segby_v1's LEAF hub on an MKS Dual FOC v3.2 ESP32.

  ONE node driving BOTH wheels: the Dual FOC board runs SimpleFOC on two
  motors (M0 = left, M1 = right), so a single hub exposes two independent
  command ports and two status ports — each motor its own slot and its own
  on-chip floor (§05).

    * `motor_left` / `motor_right` (in): an `:effort` command at 50 Hz each.
      `has_safe_action: true, safe_action: %{nm: 0.0}` (zero torque, ADR-0005)
      drives each motor's own on-chip floor (§05); the consumer freshness window
      (`fresh_for`) lives on the actuator views in the robot topology, not here.
    * `status_left` / `status_right` (out): each motor's reported truth
      (applied_seq, floored?) at 50 Hz — the source of "is it actually
      driving?", read instead of inferred from "we sent a command".
    * `vel_left` / `vel_right` (out): each wheel's MEASURED shaft angular
      velocity (`SegbyV1.ValueTypes.WheelSpeed`, rad/s) at 50 Hz — a CONSUMER
      value-type named BY MODULE (the extension seam, ADR-0003). A SENSOR stream
      (not `:status`, which stays the pure liveness flag, ADR-0009): on hardware
      it is the FOC `shaft_velocity`, in the sim the wheel hinge `qvel`. The host
      balance loop reads it to close an inner velocity loop on top of balance.

  The declared `step` / `sample` MFA refs are data only; the authoritative
  floors and the on-device sense hooks live on the Dual FOC board's own chip
  (`firmware/mcu/wheels.cpp`).
  """
  use BBMCUHub.Hub

  ports do
    port(:motor_left,
      dir: :in,
      type: :effort,
      rate: 50,
      has_safe_action: true,
      safe_action: %{nm: 0.0},
      step: {SegbyV1.Hubs.Wheels.Floor, :step}
    )

    port(:motor_right,
      dir: :in,
      type: :effort,
      rate: 50,
      has_safe_action: true,
      safe_action: %{nm: 0.0},
      step: {SegbyV1.Hubs.Wheels.Floor, :step}
    )

    port(:status_left,
      dir: :out,
      type: :status,
      rate: 50
    )

    port(:status_right,
      dir: :out,
      type: :status,
      rate: 50
    )

    port(:vel_left,
      dir: :out,
      type: SegbyV1.ValueTypes.WheelSpeed,
      rate: 50,
      sample: {SegbyV1.Hubs.Wheels.SampleVelLeft, :sample}
    )

    port(:vel_right,
      dir: :out,
      type: SegbyV1.ValueTypes.WheelSpeed,
      rate: 50,
      sample: {SegbyV1.Hubs.Wheels.SampleVelRight, :sample}
    )
  end
end
