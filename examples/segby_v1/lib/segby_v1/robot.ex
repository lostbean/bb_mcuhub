defmodule SegbyV1.Robot do
  @moduledoc """
  The segby_v1 robot (§09) — a two-wheel self-balancing bot, defined in the
  BeamBots DSL over a UART backplane (no CAN transceiver on hand).

  BeamBots sees ordinary components; it never sees the root hub or the wire tree
  below it. Each hub port surfaces as a thin `BBMCUHub.BBHub.Sensor` /
  `BBMCUHub.BBHub.Actuator` view; below that seam runs our COBS+CRC frame over
  UART, carrying `seq` and (for the IMU) `t_dev`, with the floor on the wheels
  hub's own chip.

  Two hubs (ADR-0006: topology is DECLARED by parent links):

    * the **Blaster** (NODE 0x02) is the root comms hub (`parent: :host`) — it
      owns the host UART and senses pose (the MPU-9250 IMU) and a forward range
      (the HC-SR04), and drives a decorative WS2812 status strip. The wheels leaf
      hangs off it over a UART link.
    * the **Wheels** (NODE 0x05) is the leaf — ONE MKS Dual FOC board driving
      both wheels, so it takes two effort commands (left/right) and reports two
      statuses. Its parent link is UART (`parent: :blaster, uplink: :uart`).

  The host control pipeline lives in `SegbyV1.Balance` — a robot-level
  `BB.Controller` placed in the `controllers do` block below. It consumes the
  `chassis_imu` pose and produces per-wheel effort commands (it is a pure
  consumer+producer across the BeamBots seam; the actuator views remain the
  single writers of the command slots, §04). It starts DISABLED.

  The hub-gateway DSL (`BBMCUHub.Dsl`) composes alongside BeamBots' own: the
  `hubs do` block places each hub on a NODE id and the views in `topology` name
  the hub+port they read.
  """
  use BB, extensions: [BBMCUHub.Dsl]

  hubs do
    # The root (parent: :host) owns the host UART; the wheels leaf hangs off it
    # over a UART link (ADR-0006).
    hub(:blaster, SegbyV1.Hubs.Blaster, node: 0x02, parent: :host)
    hub(:wheels, SegbyV1.Hubs.Wheels, node: 0x05, parent: :blaster, uplink: :uart)
  end

  controllers do
    # The host balance loop (§09). It subscribes to the chassis-IMU pose topic
    # and publishes Effort to BOTH wheel actuator topics — so it needs the two
    # wheel actuator paths, which are the topology nesting (link → joint →
    # actuator). Starts DISABLED; enable live via `SegbyV1.Balance.enable/1`.
    controller(
      :balance,
      {
        SegbyV1.Balance,
        # PID gains tuned against the real closed loop in the MuJoCo sim (ADR-0008):
        # holds the bot upright and recovers from a nudge to <1°. kd stays small — a
        # large derivative destabilizes on the noisy 100 Hz loop with clamp
        # saturation. (Absolute drive sign is set by the sim MJCF actuator gear.)
        #
        # Inner velocity loop (ADR-0009): teleop sets a per-wheel TARGET SPEED
        # (rad/s) and the controller adds kv·(target − measured) to the balance
        # torque, clamped to output_clamp. kv / max_speed / max_turn_speed were
        # tuned HEADLESS against the real SegbyV1.Balance + MuJoCo loop:
        #
        #   * kv is SMALL (0.02) on purpose — the balancer naturally spins the wheels
        #     fast to stay upright, so a large kv makes the velocity term saturate the
        #     ±output_clamp and starve the balance torque (kv≥0.04 topples at
        #     forward=1.0). At 0.02, forward∈[0,1] drives a smooth, DURABLE steady
        #     roll (forward=0.5 → ~4.9 rad/s, forward=1.0 → ~13 rad/s, |pitch|<1°
        #     sustained — replacing the old torque-bias regime where only
        #     forward≈0.001 was usable).
        #   * max_turn_speed is kept LOW (2.0). Turning is the soft spot: the balance
        #     PID only senses PITCH (fore-aft), so a sustained differential (turn)
        #     command slowly pumps yaw-coupled energy the loop can't reject and the
        #     bot eventually topples — independent of how small the gain is. Gentle
        #     turns (turn ≲ 0.3 sustained, or brief taps) yaw cleanly and hold;
        #     hard SUSTAINED full turns are a known limitation (ADR-0009 follow-up:
        #     a yaw-aware term / IMU yaw feedback would be needed to make full
        #     sustained turning stable).
        pose_topic: [:sensor, :base_link, :chassis_imu],
        left_actuator_path: [:base_link, :left_wheel, :left_drive],
        right_actuator_path: [:base_link, :right_wheel, :right_drive],
        kp: 8.0,
        ki: 0.3,
        kd: 0.2,
        target_pitch: 0.0,
        integral_clamp: 1.0,
        output_clamp: 1.0,
        max_speed: 10.0,
        max_turn_speed: 2.0,
        kv: 0.02,
        enabled: false
      }
    )
  end

  commands do
    # Operator teleop from the dashboard (§09). bb_tui's Commands panel runs this
    # via the runtime; its handler publishes a Twist onto the balance controller's
    # teleop topic, which sets a per-wheel TARGET SPEED for the inner velocity loop
    # (ADR-0009 — NOT a torque bias). `allowed_states [:*]` so an operator can
    # teleop in any non-disarmed state.
    command :teleop do
      handler(SegbyV1.Teleop)
      allowed_states([:*])

      argument :forward, :float do
        default(0.0)
        doc("forward TARGET SPEED in [-1.0, 1.0] — roll at a bounded rate (scaled by max_speed)")
      end

      argument :turn, :float do
        default(0.0)
        doc("turn TARGET SPEED in [-1.0, 1.0] — differential yaw rate (scaled by max_turn_speed)")
      end
    end
  end

  topology do
    link :base_link do
      # the chassis IMU — a BB.Sensor view over the blaster hub's pose port
      sensor(
        :chassis_imu,
        {BBMCUHub.BBHub.Sensor, hub: :blaster, port: :pose, fresh_for: 3, beat_ms: 10}
      )

      # the forward rangefinder — a BB.Sensor view over the blaster's range port
      sensor(
        :range_front,
        {BBMCUHub.BBHub.Sensor, hub: :blaster, port: :range_front, fresh_for: 3, beat_ms: 50}
      )

      # measured wheel speeds (ADR-0009) — BB.Sensor views over the wheels hub's
      # velocity ports, published on `[:sensor | …]` exactly like the IMU pose so
      # the balance loop can read them for its inner velocity loop. beat_ms 20 =
      # 50 Hz, matching the ports' rate.
      sensor(
        :vel_left,
        {BBMCUHub.BBHub.Sensor, hub: :wheels, port: :vel_left, fresh_for: 3, beat_ms: 20}
      )

      sensor(
        :vel_right,
        {BBMCUHub.BBHub.Sensor, hub: :wheels, port: :vel_right, fresh_for: 3, beat_ms: 20}
      )

      # left wheel — a BB.Actuator view over the wheels hub's left command port,
      # reading its left status slot for liveness (§05). fresh_for is the
      # command's consumer window (§04); the hub's own floor is the safe state.
      joint :left_wheel do
        type(:continuous)

        axis do
        end

        limit do
          effort(~u(10 newton_meter))
          velocity(~u(20 radian_per_second))
        end

        actuator(
          :left_drive,
          {BBMCUHub.BBHub.Actuator,
           hub: :wheels, port: :motor_left, status_port: :status_left, fresh_for: 5}
        )

        link :left_wheel_link do
        end
      end

      # right wheel — the mirror of the left
      joint :right_wheel do
        type(:continuous)

        axis do
        end

        limit do
          effort(~u(10 newton_meter))
          velocity(~u(20 radian_per_second))
        end

        actuator(
          :right_drive,
          {BBMCUHub.BBHub.Actuator,
           hub: :wheels, port: :motor_right, status_port: :status_right, fresh_for: 5}
        )

        link :right_wheel_link do
        end
      end
    end
  end
end
