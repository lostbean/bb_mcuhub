defmodule SegbyV1.BalanceTest do
  @moduledoc """
  The segby_v1 host balance controller (§09).

  Two layers:

    * the PURE cores — `step/3` (the PID),
      `step_pitch/4` (the accel/gyro complementary filter — the LIVE pitch source),
      `pitch_from_imu/1` (quaternion → pitch — an unused helper), and
      `velocity_mix/4` (the inner velocity loop — teleop target speed + measured
      speed + balance torque, ADR-0009) — exercised directly, no process.
    * an integration layer — the controller driven by synthetic pose messages
      through the real BB PubSub seam, asserting it publishes `Effort` to BOTH
      wheel actuator topics when enabled and zero when disabled (and never writes
      a slot — it only publishes, per §04). The synthetic pose carries the tilt in
      the ACCEL vector (gravity projection), matching the live accel/gyro path.
  """
  use ExUnit.Case, async: false

  alias SegbyV1.Balance
  alias SegbyV1.Balance.Pid
  alias BB.Math.{Quaternion, Vec3}

  # ---------------------------------------------------------------------------
  # Pure PID core
  # ---------------------------------------------------------------------------
  describe "step/3 — pure PID" do
    test "zero gains -> zero output" do
      pid = %Pid{kp: 0.0, ki: 0.0, kd: 0.0, output_clamp: 10.0, integral_clamp: 10.0}
      {out, _} = Balance.step(pid, 0.5, 0.01)
      assert_in_delta out, 0.0, 1.0e-9
    end

    test "zero error -> zero output" do
      pid = %Pid{kp: 2.0, ki: 0.5, kd: 0.1, output_clamp: 10.0, integral_clamp: 10.0}
      {out, _new} = Balance.step(pid, 0.0, 0.01)
      assert_in_delta out, 0.0, 1.0e-9
    end

    test "positive error -> positive (proportional) output" do
      pid = %Pid{kp: 2.0, output_clamp: 10.0, integral_clamp: 10.0}
      {out, _} = Balance.step(pid, 0.5, 0.01)
      assert out > 0.0
      assert_in_delta out, 1.0, 1.0e-6
    end

    test "integral accumulates over time" do
      pid = %Pid{ki: 1.0, output_clamp: 10.0, integral_clamp: 10.0}

      {_, p1} = Balance.step(pid, 1.0, 0.1)
      assert_in_delta p1.integral, 0.1, 1.0e-9

      {_, p2} = Balance.step(p1, 1.0, 0.1)
      assert_in_delta p2.integral, 0.2, 1.0e-9
    end

    test "integral clamp bounds windup" do
      pid = %Pid{ki: 1.0, output_clamp: 1000.0, integral_clamp: 0.5}

      pid =
        Enum.reduce(1..100, pid, fn _i, p ->
          {_, np} = Balance.step(p, 10.0, 0.1)
          np
        end)

      assert pid.integral <= 0.5
      assert pid.integral > 0.0
    end

    test "output clamp bounds the published torque" do
      pid = %Pid{kp: 100.0, output_clamp: 1.0, integral_clamp: 10.0}
      {out, _} = Balance.step(pid, 5.0, 0.01)
      assert out == 1.0
    end

    test "derivative term contributes when error changes" do
      pid = %Pid{kd: 1.0, output_clamp: 1000.0, integral_clamp: 1000.0}
      # First step seeds prev_error.
      {_, p1} = Balance.step(pid, 0.0, 0.01)
      # Error jumps to 1.0 in 0.01 s -> derivative = 100; kd=1 -> out = 100.
      {out, _} = Balance.step(p1, 1.0, 0.01)
      assert_in_delta out, 100.0, 1.0e-6
    end

    test "dt_s == 0 does not advance integral or derivative" do
      pid = %Pid{kp: 1.0, ki: 1.0, kd: 1.0, output_clamp: 10.0, integral_clamp: 10.0}
      {out, new_pid} = Balance.step(pid, 0.5, 0.0)
      assert new_pid.integral == 0.0
      # With dt=0, derivative is 0 — output is just kp * error.
      assert_in_delta out, 0.5, 1.0e-9
    end
  end

  # ---------------------------------------------------------------------------
  # Pure pitch extraction — from the orientation quaternion
  # ---------------------------------------------------------------------------
  describe "pitch_from_imu/1 — quaternion pitch" do
    test "a level IMU (identity orientation) -> ~0 pitch" do
      imu = imu(Quaternion.identity())
      assert_in_delta Balance.pitch_from_imu(imu), 0.0, 1.0e-9
    end

    test "a nose-up tilt -> positive pitch of the expected magnitude" do
      # rotation of +0.2 rad about body Y is a +0.2 rad pitch
      q = Quaternion.from_euler(0.0, 0.2, 0.0, :xyz)
      assert_in_delta Balance.pitch_from_imu(imu(q)), 0.2, 1.0e-6
    end

    test "a nose-down tilt -> negative pitch" do
      q = Quaternion.from_euler(0.0, -0.35, 0.0, :xyz)
      pitch = Balance.pitch_from_imu(imu(q))
      assert pitch < 0.0
      assert_in_delta pitch, -0.35, 1.0e-6
    end

    test "pure roll does not bleed into pitch" do
      q = Quaternion.from_euler(0.3, 0.0, 0.0, :xyz)
      assert_in_delta Balance.pitch_from_imu(imu(q)), 0.0, 1.0e-6
    end
  end

  # ---------------------------------------------------------------------------
  # Pure complementary filter — the LIVE pitch source (accel + gyro)
  # ---------------------------------------------------------------------------
  describe "step_pitch/4 — accel/gyro complementary filter" do
    test "a level IMU (accel = {0,0,g}, gyro 0) -> ~0 pitch" do
      imu = imu_accel_gyro({0.0, 0.0, 9.81}, {0.0, 0.0, 0.0})
      # From rest at 0, a level accel keeps pitch at 0 regardless of dt/alpha.
      assert_in_delta Balance.step_pitch(0.0, imu, 0.01), 0.0, 1.0e-9
    end

    test "a static tilt accel anchors pitch toward the gravity-projection angle" do
      # nose-up tilt theta: gravity projects to ax = -g·sin(theta), az = g·cos(theta).
      theta = 0.3

      imu =
        imu_accel_gyro({-9.81 * :math.sin(theta), 0.0, 9.81 * :math.cos(theta)}, {0.0, 0.0, 0.0})

      # On the FIRST tick (dt=0) the gyro term is the prior pitch unchanged (0), so
      # the blend lands at (1-alpha)*accel_pitch = 0.02 * theta.
      first = Balance.step_pitch(0.0, imu, 0.0)
      assert_in_delta first, 0.02 * theta, 1.0e-6
      assert first > 0.0

      # Iterating with the SAME static accel + zero gyro converges to theta.
      converged =
        Enum.reduce(1..2000, 0.0, fn _i, p -> Balance.step_pitch(p, imu, 0.01) end)

      assert_in_delta converged, theta, 1.0e-3
    end

    test "a nose-down tilt -> negative pitch" do
      theta = -0.25

      imu =
        imu_accel_gyro({-9.81 * :math.sin(theta), 0.0, 9.81 * :math.cos(theta)}, {0.0, 0.0, 0.0})

      converged = Enum.reduce(1..2000, 0.0, fn _i, p -> Balance.step_pitch(p, imu, 0.01) end)
      assert converged < 0.0
      assert_in_delta converged, theta, 1.0e-3
    end

    test "gyro integration advances pitch over dt (rad/s, no deg conversion)" do
      # level accel (so accel_pitch = 0) + a +0.5 rad/s body-Y rate over 0.1 s.
      imu = imu_accel_gyro({0.0, 0.0, 9.81}, {0.0, 0.5, 0.0})
      # gyro_pitch = 0 + 0.5*0.1 = 0.05; pitch' = 0.98*0.05 + 0.02*0 = 0.049.
      assert_in_delta Balance.step_pitch(0.0, imu, 0.1), 0.98 * 0.05, 1.0e-9
    end

    test "alpha weights the gyro vs accel terms" do
      # disagreeing sources: gyro says +0.1 (from prev 0 + rate*dt), accel says +0.3.
      theta = 0.3

      imu =
        imu_accel_gyro({-9.81 * :math.sin(theta), 0.0, 9.81 * :math.cos(theta)}, {0.0, 1.0, 0.0})

      dt = 0.1
      # gyro_pitch = 0 + 1.0*0.1 = 0.1; accel_pitch ~ 0.3.
      gyro_pitch = 0.1
      accel_pitch = :math.atan2(-(-9.81 * :math.sin(theta)), 9.81 * :math.cos(theta))

      # alpha 0.98 (default): gyro-dominated.
      hi = Balance.step_pitch(0.0, imu, dt, 0.98)
      assert_in_delta hi, 0.98 * gyro_pitch + 0.02 * accel_pitch, 1.0e-6

      # alpha 0.5: even blend pulls harder toward accel.
      lo = Balance.step_pitch(0.0, imu, dt, 0.5)
      assert_in_delta lo, 0.5 * gyro_pitch + 0.5 * accel_pitch, 1.0e-6
      assert lo > hi
    end
  end

  # ---------------------------------------------------------------------------
  # Pure inner velocity loop (ADR-0009)
  # ---------------------------------------------------------------------------
  describe "velocity_mix/4 — the inner velocity loop" do
    # max_speed 10, max_turn_speed 4, kv 0.1, output_clamp big enough not to bite.
    @vp %{max_speed: 10.0, max_turn_speed: 4.0, kv: 0.1, output_clamp: 100.0}
    @zero %{left: 0.0, right: 0.0}

    test "zero teleop + zero measured -> just the balance torque on both wheels" do
      mixed = Balance.velocity_mix(0.3, %{forward: 0.0, turn: 0.0}, @zero, @vp)
      assert_in_delta mixed.left, 0.3, 1.0e-9
      assert_in_delta mixed.right, 0.3, 1.0e-9
    end

    test "zero teleop but nonzero measured -> the loop resists drift (holds zero speed)" do
      # measured +2 rad/s on both, target 0 -> kv·(0 - 2) = -0.2 added to balance.
      mixed = Balance.velocity_mix(0.0, %{forward: 0.0, turn: 0.0}, %{left: 2.0, right: 2.0}, @vp)
      assert_in_delta mixed.left, -0.2, 1.0e-9
      assert_in_delta mixed.right, -0.2, 1.0e-9
    end

    test "forward target adds kv·(target - measured) equally to BOTH wheels" do
      # forward 1.0 -> target 10 rad/s; measured 0 -> kv·10 = 1.0 on top of balance.
      mixed = Balance.velocity_mix(0.0, %{forward: 1.0, turn: 0.0}, @zero, @vp)
      assert_in_delta mixed.left, 1.0, 1.0e-9
      assert_in_delta mixed.right, 1.0, 1.0e-9
    end

    test "the velocity term is ADDITIVE to the balance torque (sign preserved)" do
      # a corrective balance torque of -0.5 + a small forward velocity push.
      mixed = Balance.velocity_mix(-0.5, %{forward: 0.2, turn: 0.0}, @zero, @vp)
      # target 2 rad/s, measured 0 -> +0.2; -0.5 + 0.2 = -0.3 (still negative).
      assert_in_delta mixed.left, -0.3, 1.0e-9
      assert_in_delta mixed.right, -0.3, 1.0e-9
    end

    test "the controller adds exactly kv·(target - measured) for a given measured speed" do
      # forward 0.5 -> target 5 rad/s; measured 1.5 -> error 3.5; kv 0.1 -> +0.35.
      mixed = Balance.velocity_mix(0.0, %{forward: 0.5, turn: 0.0}, %{left: 1.5, right: 1.5}, @vp)
      assert_in_delta mixed.left, 0.1 * (5.0 - 1.5), 1.0e-9
      assert_in_delta mixed.right, 0.1 * (5.0 - 1.5), 1.0e-9
    end

    test "turn differentials the target speed (right +, left -)" do
      # turn 1.0 -> target_left -4, target_right +4; measured 0 -> kv·∓4 = ∓0.4.
      mixed = Balance.velocity_mix(0.0, %{forward: 0.0, turn: 1.0}, @zero, @vp)
      assert_in_delta mixed.left, -0.4, 1.0e-9
      assert_in_delta mixed.right, 0.4, 1.0e-9
    end

    test "forward + turn combine into the per-wheel target speed" do
      mixed = Balance.velocity_mix(0.0, %{forward: 1.0, turn: 1.0}, @zero, @vp)
      # left target = 10 - 4 = 6 -> +0.6; right target = 10 + 4 = 14 -> +1.4.
      assert_in_delta mixed.left, 0.6, 1.0e-9
      assert_in_delta mixed.right, 1.4, 1.0e-9
    end

    test "teleop intent is clamped to [-1, 1] before scaling to a target speed" do
      mixed = Balance.velocity_mix(0.0, %{forward: 5.0, turn: -5.0}, @zero, @vp)
      # forward clamps to 1.0 -> target 10; turn clamps to -1.0 -> ±4.
      # left = 10 - (-4) = 14 -> +1.4; right = 10 + (-4) = 6 -> +0.6.
      assert_in_delta mixed.left, 1.4, 1.0e-9
      assert_in_delta mixed.right, 0.6, 1.0e-9
    end

    test "the final per-wheel torque is clamped symmetrically to output_clamp" do
      vp = %{max_speed: 100.0, max_turn_speed: 0.0, kv: 1.0, output_clamp: 1.0}
      # target 100, measured 0 -> +100, plus balance 0.5 -> clamped to +1.0.
      mixed = Balance.velocity_mix(0.5, %{forward: 1.0, turn: 0.0}, @zero, vp)
      assert mixed.left == 1.0
      assert mixed.right == 1.0

      # negative side clamps too.
      mixed2 = Balance.velocity_mix(-0.5, %{forward: -1.0, turn: 0.0}, @zero, vp)
      assert mixed2.left == -1.0
      assert mixed2.right == -1.0
    end
  end

  # ---------------------------------------------------------------------------
  # Integration — the controller across the real BB PubSub seam
  # ---------------------------------------------------------------------------
  describe "the controller commands both wheels via Effort (never writes a slot)" do
    @robot SegbyV1.Robot
    @pose_topic [:sensor, :base_link, :chassis_imu]
    # distinct actuator paths so our test process is the SOLE subscriber and the
    # real views (not started here) can't compete.
    @left [:test, :left_drive]
    @right [:test, :right_drive]
    # the measured-wheel-speed sensor topics (ADR-0009) the velocity loop reads.
    @vel_left_topic [:sensor, :base_link, :vel_left]
    @vel_right_topic [:sensor, :base_link, :vel_right]

    setup do
      # A real BB PubSub registry for this robot, so BB.subscribe / BB.publish
      # work. We do NOT start the full topology (its PortIndex isn't built for
      # segby in test); the controller is driven directly via the ViewHarness.
      start_supervised!(
        {Registry, keys: :duplicate, name: BB.PubSub.registry_name(@robot)},
        id: :pubsub_registry
      )

      :ok
    end

    test "ENABLED: a forward tilt drives non-zero Effort to BOTH wheels" do
      ctrl = start_controller(enabled: true)

      BB.subscribe(@robot, [:actuator | @left])
      BB.subscribe(@robot, [:actuator | @right])

      # First pose seeds the time base (dt=0); second produces the derivative.
      send(ctrl, pose_msg(0.1, 0))
      send(ctrl, pose_msg(0.1, 10_000_000))

      left = drain_last_effort([:actuator | @left])
      right = drain_last_effort([:actuator | @right])

      # target_pitch 0.0, pitch +0.1 -> error -0.1 -> kp 0.5 -> negative torque,
      # commanded to BOTH wheels.
      assert left < 0.0
      assert right < 0.0
      assert_in_delta left, right, 1.0e-9
    end

    test "DISABLED: any tilt commands ZERO Effort to BOTH wheels" do
      ctrl = start_controller(enabled: false)

      BB.subscribe(@robot, [:actuator | @left])
      BB.subscribe(@robot, [:actuator | @right])

      # A big tilt that would otherwise drive a large corrective torque.
      send(ctrl, pose_msg(0.5, 0))
      send(ctrl, pose_msg(0.5, 10_000_000))

      assert drain_last_effort([:actuator | @left]) == 0.0
      assert drain_last_effort([:actuator | @right]) == 0.0
    end

    test "toggling enable on (live) makes a tilt produce corrective Effort again" do
      ctrl = start_controller(enabled: false)

      BB.subscribe(@robot, [:actuator | @left])
      BB.subscribe(@robot, [:actuator | @right])

      send(ctrl, pose_msg(0.3, 0))
      assert drain_last_effort([:actuator | @left]) == 0.0

      send(ctrl, {:balance_enable, true})
      flush(ctrl)
      send(ctrl, pose_msg(0.3, 20_000_000))

      assert drain_last_effort([:actuator | @left]) < 0.0
    end

    test "teleop forward intent drives the velocity loop on top of balance" do
      # max_speed 10, kv 0.1 -> forward 1.0 = target 10 rad/s; measured 0 (no vel
      # stream injected) -> kv·(10 - 0) = +1.0 on top of the (~0) balance torque.
      ctrl = start_controller(enabled: true, max_speed: 10.0, max_turn_speed: 4.0, kv: 0.1)

      BB.subscribe(@robot, [:actuator | @left])
      BB.subscribe(@robot, [:actuator | @right])

      # level pose so the PID torque is ~0; the pure velocity term should show through.
      send(ctrl, {:teleop, %{forward: 1.0, turn: 0.0}})
      send(ctrl, pose_msg(0.0, 0))
      send(ctrl, pose_msg(0.0, 10_000_000))

      left = drain_last_effort([:actuator | @left])
      right = drain_last_effort([:actuator | @right])

      assert_in_delta left, 1.0, 1.0e-6
      assert_in_delta right, 1.0, 1.0e-6
    end

    test "a measured wheel speed bleeds the velocity error (kv·(target - measured))" do
      ctrl = start_controller(enabled: true, max_speed: 10.0, max_turn_speed: 4.0, kv: 0.1)

      BB.subscribe(@robot, [:actuator | @left])
      BB.subscribe(@robot, [:actuator | @right])

      # forward 1.0 -> target 10 rad/s. Report both wheels already at 4 rad/s, so
      # the velocity error is 6 and the contribution is kv·6 = +0.6.
      send(ctrl, {:teleop, %{forward: 1.0, turn: 0.0}})
      send(ctrl, vel_msg(@vel_left_topic, 4.0))
      send(ctrl, vel_msg(@vel_right_topic, 4.0))
      send(ctrl, pose_msg(0.0, 0))
      send(ctrl, pose_msg(0.0, 10_000_000))

      left = drain_last_effort([:actuator | @left])
      right = drain_last_effort([:actuator | @right])

      # kv 0.1 · (10 - 4) = 0.6 on top of the ~0 balance torque.
      assert_in_delta left, 0.6, 1.0e-6
      assert_in_delta right, 0.6, 1.0e-6
    end
  end

  # --- helpers ---------------------------------------------------------------

  # The quaternion-pitch helper still exercises pitch_from_imu/1 (an unused
  # helper) with a tilt orientation + a fixed level accel.
  defp imu(%Quaternion{} = q) do
    %BB.Message.Sensor.Imu{
      orientation: q,
      angular_velocity: Vec3.zero(),
      linear_acceleration: Vec3.new(0.0, 0.0, 9.81)
    }
  end

  # An IMU shaped like the segby MCU ships it: identity orientation, real accel
  # (m/s²) + gyro (rad/s). This is what the live complementary-filter path reads.
  defp imu_accel_gyro({ax, ay, az}, {wx, wy, wz}) do
    %BB.Message.Sensor.Imu{
      orientation: Quaternion.identity(),
      angular_velocity: Vec3.new(wx, wy, wz),
      linear_acceleration: Vec3.new(ax, ay, az)
    }
  end

  # A controller instance wrapped in the ViewHarness (drives init/1 + handle_info/2
  # directly, like the slice test does for the views), wired to the test topics.
  defp start_controller(extra) do
    opts =
      [
        bb: %{robot: @robot, path: [:balance]},
        pose_topic: @pose_topic,
        teleop_topic: [:teleop, :segby_test],
        vel_left_topic: @vel_left_topic,
        vel_right_topic: @vel_right_topic,
        left_actuator_path: @left,
        right_actuator_path: @right,
        kp: 0.5,
        ki: 0.05,
        kd: 0.1,
        target_pitch: 0.0,
        integral_clamp: 1.0,
        output_clamp: 1.0,
        max_speed: 10.0,
        max_turn_speed: 2.0,
        kv: 0.02,
        enabled: false
      ]
      |> Keyword.merge(extra)

    {:ok, pid} = SegbyV1.Test.ViewHarness.start(Balance, opts)
    pid
  end

  # A pose message as the chassis_imu sensor view would publish it. The segby MCU
  # ships an IDENTITY orientation + real accel/gyro, so a body-Y tilt `pitch`
  # (radians) is carried as the gravity projection in the accel vector
  # (ax = -g·sin(pitch), az = g·cos(pitch), gyro zero) — exactly what the live
  # complementary-filter path reads. `mono` is the monotonic timestamp (ns).
  @g 9.81
  defp pose_msg(pitch, mono) do
    accel = {-@g * :math.sin(pitch), 0.0, @g * :math.cos(pitch)}

    msg = %BB.Message{
      monotonic_time: mono,
      wall_time: mono,
      node: Node.self(),
      frame_id: :chassis_imu,
      payload: imu_accel_gyro(accel, {0.0, 0.0, 0.0}),
      robot: @robot
    }

    {:bb, @pose_topic, msg}
  end

  # A measured-wheel-speed message as the vel_* sensor view would publish it
  # (ADR-0009): a `JointState` carrying this one wheel's velocity (rad/s). The
  # velocity loop reads `velocities: [rad_s | _]` from it.
  defp vel_msg(topic, rad_s) do
    msg = %BB.Message{
      monotonic_time: 0,
      wall_time: 0,
      node: Node.self(),
      frame_id: :wheel_speed,
      payload: %BB.Message.Sensor.JointState{
        names: [:wheel],
        positions: [],
        velocities: [rad_s * 1.0],
        efforts: []
      },
      robot: @robot
    }

    {:bb, topic, msg}
  end

  # Wait for at least one Effort published on `topic` and return the last one's
  # effort value (the controller publishes one per pose tick).
  defp drain_last_effort(topic) do
    receive do
      {:bb, ^topic, %BB.Message{payload: %BB.Message.Actuator.Command.Effort{effort: e}}} ->
        drain_more(topic, e)
    after
      500 -> flunk("no Effort received on #{inspect(topic)}")
    end
  end

  defp drain_more(topic, last) do
    receive do
      {:bb, ^topic, %BB.Message{payload: %BB.Message.Actuator.Command.Effort{effort: e}}} ->
        drain_more(topic, e)
    after
      30 -> last
    end
  end

  # Block until the harness has processed prior sends (the call flushes its mailbox).
  defp flush(ctrl), do: SegbyV1.Test.ViewHarness.view_state(ctrl)
end
