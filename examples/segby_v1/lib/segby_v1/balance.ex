defmodule SegbyV1.Balance do
  @moduledoc """
  The segby_v1 host balance controller (§09, Phase) — a `BB.Controller` that
  closes the self-balancing loop on the host.

  This is the host's control pipeline (a PID balance loop + a complementary-filter
  IMU estimator + an **inner velocity loop** that turns operator teleop into a
  bounded per-wheel target speed), wired to the BeamBots
  seam: it is a **consumer** of the chassis IMU pose AND the two measured
  wheel-speed sensor streams, and a **producer** of
  per-wheel effort commands. It NEVER writes a hub command slot — that is the
  actuator view's job (§04 single-writer). It publishes typed
  `BB.Message.Actuator.Command.Effort` messages to each wheel's actuator topic;
  the actuator views turn those into slot writes, and the on-chip floor (§05)
  remains the safe-state guarantee.

  ## Pipeline (per pose tick)

      pose (BB.Message.Sensor.Imu) → step_pitch/4 (complementary filter)
        → PID step/3 → balance_torque
        → velocity_mix/5 (forward SPEED loop: kv·(target − measured) per wheel;
            turn YAW-RATE loop: ±kyaw·(target_yaw_rate − gyro_z) differential)
        → {left, right} clamped to output_clamp → set_effort to both wheels

  ## The inner velocity loop + the yaw-rate turn loop (ADR-0009 + amendment)

  Teleop on a balancing bot CANNOT be a torque bias: torque is acceleration, so a
  forward torque bias accelerates the wheels forever — they run away and the bot
  falls. Instead `forward` sets a per-wheel **target speed** (rad/s) and `turn`
  commands a **target yaw rate** (rad/s), and the controller closes a loop on each
  on top of the balance torque:

      fwd_target      = forward · max_speed                       # forward SPEED loop
      target_yaw_rate = turn · max_yaw_rate                       # turn YAW-RATE loop
      turn_torque     = kyaw · (target_yaw_rate − gyro_z)         # closed on measured yaw
      left  = clamp(balance_torque + kv · (fwd_target − measured_left)  − turn_torque, ±output_clamp)
      right = clamp(balance_torque + kv · (fwd_target − measured_right) + turn_torque, ±output_clamp)

  So `forward = 0.5` means "roll forward at half of `max_speed` rad/s", a bounded
  rate the loop holds by bleeding off the speed error — not a runaway. The measured
  speeds come from the wheels hub's `vel_left` / `vel_right` sensor ports
  (`SegbyV1.ValueTypes.WheelSpeed` → `JointState`), subscribed below exactly as the
  IMU pose is.

  `turn` is **closed-loop on the measured chassis yaw rate** (the amendment): it
  was an open-loop differential of the wheel-speed targets, which has no feedback
  regulating the actual yaw — so a sustained hard turn pumped yaw-coupled energy
  the pitch-only balance PID cannot reject, and the bot eventually toppled (gain
  independent — lowering the turn authority only delayed it). Now `turn` commands a
  yaw RATE and `turn_torque = kyaw · (target_yaw_rate − gyro_z)` drives the
  differential torque so the **measured** yaw (the IMU gyro's body-Z,
  `Vec3.z(angular_velocity)`, already on the pose tick) tracks it. Because the
  differential is regulated by the measured yaw it cannot run away — as the bot
  yaws faster the error shrinks and the differential backs off — so any commanded
  turn rate is self-limiting and a sustained hard turn (turn=1.0) stays stable.

  With zero teleop both targets are 0 and the loop actively holds the wheels at
  zero speed and zero yaw (resisting drift). Both terms are ADDITIVE to the balance
  torque — neither changes the balance sign.

  ## Pitch extraction (accel/gyro complementary filter)

  The segby Blaster MCU does NOT fuse orientation — it packs an IDENTITY
  quaternion and ships the REAL accel (m/s²) + gyro (rad/s) in the
  `BB.Message.Sensor.Imu`'s `linear_acceleration` / `angular_velocity` Vec3s
  (the MPU-9250 read scaled to engineering units in firmware). So pitch is
  recovered HERE by the complementary filter, blending the gyro-integrated pitch
  with the accel-derived absolute pitch:

      accel_pitch = atan2(-ax, sqrt(ay² + az²))     # absolute, drift-free, noisy
      gyro_pitch  = pitch + wy · dt                  # smooth short-term, drifts
      pitch'      = α · gyro_pitch + (1-α) · accel_pitch   # α = 0.98

  `α = 0.98` (gyro-heavy short-term, accel-anchored long-term) is a standard
  complementary-filter blend. The gyro is already in rad/s on the wire, so
  `step_pitch/4` integrates `wy · dt` directly (no deg→rad conversion — that
  scaling happened in firmware). `step_pitch/4` is a pure function; `state.pitch`
  carries the running estimate between ticks. The quaternion is identity now, so
  `pitch_from_imu/1` (the quaternion term) is kept only as an unused helper; the
  live loop reads accel/gyro.

  ## Gains (segby_v1 config)

  `kp 0.5, ki 0.05, kd 0.1, target_pitch 0.0, integral_clamp 1.0, output_clamp
  1.0`. Forward speed loop: `max_speed` (rad/s) bounds the teleop target speed, `kv`
  is the velocity-loop gain. Turn yaw-rate loop: `max_yaw_rate` (rad/s) is the
  commandable yaw rate at `turn = 1`, `kyaw` the yaw-loop gain (torque per rad/s of
  yaw error). The segby_v1 robot passes the values tuned headless against the real
  MuJoCo loop (ADR-0009 + amendment): `max_speed 10.0, kv 0.02, max_yaw_rate 0.5,
  kyaw 0.012` — `kv` deliberately small so the velocity term never saturates the
  clamp and starves the balance torque; `kyaw` deliberately small (the transient
  turn-differential kick is what topples the pitch-only balancer, so the loop gain —
  really the product `kyaw·max_yaw_rate`, the peak turn_torque at zero measured yaw —
  must stay low, ≲0.008). With these, a SUSTAINED full turn (turn=1.0) stays upright
  (|pitch| ~0° over 15 s headless) and yaws at a steady ~0.4 rad/s, scaling linearly
  with `turn`; the old open-loop differential toppled here (see `SegbyV1.Robot`).

  ## Enable / disable

  Starts **DISABLED** (publishes zero BALANCE torque every pose tick so the chassis
  is not actively held upright out of the box). Balance is enabled live by sending
  the controller `{:balance_enable, bool}` — use `enable/1` / `disable/1`, which
  resolve the running controller and cast the toggle. Toggling resets the
  integrator so re-enabling never dumps accumulated windup.

  The **inner velocity loop still runs while disabled**: with no balance torque the
  per-wheel output is `kv · (target − measured)`, so an operator can still drive the
  wheels at a bounded speed with balance off (useful for the bringup bench, where
  you spin the wheels before trusting the balance loop). Disabling stops the
  controller actively *balancing*, not driving.

  ## Teleop (operator input via bb_tui) — a bounded SPEED, not a torque

  Teleop intent (`forward`, `turn`, both in `[-1.0, 1.0]`) arrives on a PubSub
  topic (`:teleop_topic`, default `[:teleop, :segby]`). `forward` sets a per-wheel
  **target speed** (rad/s, scaled by `max_speed`) — "roll forward at a bounded
  rate", NOT a torque bias — and `turn` commands a **target yaw rate** (rad/s,
  scaled by `max_yaw_rate`), closed-loop on the IMU's measured yaw. The
  controller's `velocity_mix/5` adds the forward velocity term `kv · (target −
  measured)` and the yaw differential term `±kyaw · (target_yaw_rate − gyro_z)` to
  the balance torque every pose tick; with zero teleop both targets are 0 and the
  loop holds the wheels at zero speed and zero yaw.

  `bb_tui` has no built-in "teleop" concept — its operator surface is the
  declared-`commands` panel (executed via `BB.Robot.Runtime.execute/3`), the
  joints panel (`BB.Actuator.set_position!/3`), and arm/disarm. Driving the
  wheels directly from the joints panel would fight this controller, which is the
  *sole* actuator commander for the wheels (§04 single-writer + the balance loop
  owns the wheels). So segby surfaces operator drive as a declared command,
  `SegbyV1.Teleop` (`forward`/`turn` float args). Its handler publishes a
  `BB.Message.Geometry.Twist` (`linear.x` = forward, `angular.z` = turn) onto
  this `teleop_topic`; this controller consumes it below and biases the next pose
  tick's output. bb_tui's Commands panel discovers and runs that command — that
  is how an operator teleops segby from the dashboard.

  Two delivery shapes are accepted, both updating `last_teleop`:

    * a `BB.Message` whose payload is a `Twist` (the bb_tui / PubSub path), and
    * a bare `{:teleop, %{forward, turn}}` map (the in-process / test path).

  ## Pure functional cores (tested directly)

    * `step/3` — the PID step.
    * `step_pitch/4` — accel/gyro complementary filter → pitch (radians).
    * `pitch_from_accel/3` — accel-only absolute pitch (the filter's anchor term).
    * `pitch_from_imu/1` — quaternion → pitch (radians); unused helper.
    * `velocity_mix/5` — the inner velocity loop + the yaw-rate turn loop: teleop
      target speed + measured speed + measured yaw rate + the balance torque →
      clamped per-wheel `{left, right}` torque.
  """

  use BB.Controller,
    options_schema: [
      pose_topic: [
        type: {:list, :atom},
        default: [:sensor, :base_link, :chassis_imu],
        doc: "the chassis-IMU pose topic this controller subscribes to"
      ],
      left_actuator_path: [
        type: {:list, :atom},
        required: true,
        doc: "the left wheel actuator path (effort commands publish to [:actuator | path])"
      ],
      right_actuator_path: [
        type: {:list, :atom},
        required: true,
        doc: "the right wheel actuator path"
      ],
      teleop_topic: [
        type: {:list, :atom},
        default: [:teleop, :segby],
        doc: "PubSub topic carrying %{forward, turn} teleop intent"
      ],
      vel_left_topic: [
        type: {:list, :atom},
        default: [:sensor, :base_link, :vel_left],
        doc: "the measured left-wheel-speed sensor topic (JointState, ADR-0009)"
      ],
      vel_right_topic: [
        type: {:list, :atom},
        default: [:sensor, :base_link, :vel_right],
        doc: "the measured right-wheel-speed sensor topic (JointState, ADR-0009)"
      ],
      kp: [type: :float, default: 0.5, doc: "proportional gain"],
      ki: [type: :float, default: 0.05, doc: "integral gain"],
      kd: [type: :float, default: 0.1, doc: "derivative gain"],
      target_pitch: [type: :float, default: 0.0, doc: "upright set-point, radians"],
      integral_clamp: [type: :float, default: 1.0, doc: "symmetric windup clamp"],
      output_clamp: [type: :float, default: 1.0, doc: "symmetric torque clamp"],
      max_speed: [
        type: :float,
        default: 10.0,
        doc: "teleop forward TARGET speed (rad/s) at |forward|=1 (inner velocity loop)"
      ],
      max_yaw_rate: [
        type: :float,
        default: 0.5,
        doc: "commandable yaw rate (rad/s) at turn=1 (closed yaw-rate loop)"
      ],
      kyaw: [
        type: :float,
        default: 0.012,
        doc: "yaw-rate loop gain, torque per rad/s of yaw error"
      ],
      kv: [type: :float, default: 0.02, doc: "inner velocity-loop gain (torque per rad/s error)"],
      enabled: [type: :boolean, default: false, doc: "start enabled? (default DISABLED)"]
    ]

  alias BB.Math.{Quaternion, Vec3}
  alias BB.Message.Geometry.Twist

  # Complementary-filter blend factor (gyro-heavy short-term, accel-anchored
  # long-term) — a standard complementary-filter default.
  @filter_alpha 0.98

  # The PID functional core — its own struct so `step/3` stays pure and testable.
  defmodule Pid do
    @moduledoc "Pure PID state for `SegbyV1.Balance.step/3`."
    defstruct kp: 0.0,
              ki: 0.0,
              kd: 0.0,
              integral: 0.0,
              prev_error: 0.0,
              integral_clamp: 1.0,
              output_clamp: 1.0

    @type t :: %__MODULE__{
            kp: float(),
            ki: float(),
            kd: float(),
            integral: float(),
            prev_error: float(),
            integral_clamp: float(),
            output_clamp: float()
          }
  end

  # ----------------------------------------------------------------------------
  # Pure functional cores
  # ----------------------------------------------------------------------------

  @doc """
  Pure PID step. Given a `%Pid{}`
  state, the current `error` (target - measured), and `dt_s` since the last
  update, return `{output, new_pid}`.

  When `dt_s` is `0.0` the derivative term is zero and the integral is not
  advanced (avoids div-by-zero on the first sample). Output is clamped
  symmetrically; integral is clamped to bound windup.
  """
  @spec step(Pid.t(), float(), float()) :: {float(), Pid.t()}
  def step(%Pid{} = s, error, dt_s)
      when is_float(error) and is_float(dt_s) do
    integral =
      if dt_s > 0.0 do
        s.integral + error * dt_s
      else
        s.integral
      end

    integral = clamp(integral, -s.integral_clamp, s.integral_clamp)

    derivative =
      if dt_s > 0.0 do
        (error - s.prev_error) / dt_s
      else
        0.0
      end

    raw = s.kp * error + s.ki * integral + s.kd * derivative
    output = clamp(raw, -s.output_clamp, s.output_clamp)

    {output, %{s | integral: integral, prev_error: error}}
  end

  @doc """
  Pure accel/gyro complementary-filter pitch step. Given the previous `pitch`
  (radians), a
  `BB.Message.Sensor.Imu` carrying the real accel (m/s²) + gyro (rad/s), the
  time delta `dt_s` (seconds), and the blend factor `alpha`, return the new
  pitch (radians, body-Y rotation, positive = nose up):

      accel_pitch = atan2(-ax, sqrt(ay² + az²))   # absolute, drift-free
      gyro_pitch  = pitch + wy · dt_s             # integrated body-Y rate
      pitch'      = α · gyro_pitch + (1-α) · accel_pitch

  The gyro is already rad/s on the wire (scaled in firmware), so `wy` is
  integrated directly — no deg→rad conversion. With `dt_s = 0.0` (the first
  sample) the gyro term is `pitch` unchanged, so the result anchors fully to the
  accel estimate via the blend. Pure — no process, no I/O.
  """
  @spec step_pitch(float(), BB.Message.Sensor.Imu.t(), float(), float()) :: float()
  def step_pitch(pitch, %BB.Message.Sensor.Imu{} = imu, dt_s, alpha \\ @filter_alpha)
      when is_float(pitch) and is_float(dt_s) and is_float(alpha) do
    ax = Vec3.x(imu.linear_acceleration)
    ay = Vec3.y(imu.linear_acceleration)
    az = Vec3.z(imu.linear_acceleration)
    wy = Vec3.y(imu.angular_velocity)

    accel_pitch = pitch_from_accel(ax, ay, az)
    gyro_pitch = pitch + wy * dt_s
    alpha * gyro_pitch + (1.0 - alpha) * accel_pitch
  end

  @doc """
  Pure accel-only absolute pitch (the complementary filter's drift-free anchor
  term). `pitch = atan2(-ax, sqrt(ay² + az²))`.
  """
  @spec pitch_from_accel(float(), float(), float()) :: float()
  def pitch_from_accel(ax, ay, az) do
    :math.atan2(-ax, :math.sqrt(ay * ay + az * az))
  end

  @doc """
  Extract pitch (radians, body-Y rotation, positive = nose up) from an
  `BB.Message.Sensor.Imu`'s orientation quaternion — the standard aerospace
  (ZYX) pitch term `asin(2*(w*y - z*x))` clamped for gimbal-lock safety.

  UNUSED by the live loop: the segby MCU ships an identity quaternion and the
  real accel/gyro, so the live path runs `step_pitch/4`. Kept as a pure helper
  for a world where a fused orientation IS on the wire.
  """
  @spec pitch_from_imu(BB.Message.Sensor.Imu.t()) :: float()
  def pitch_from_imu(%BB.Message.Sensor.Imu{orientation: %Quaternion{} = q}) do
    w = Quaternion.w(q)
    x = Quaternion.x(q)
    y = Quaternion.y(q)
    z = Quaternion.z(q)

    sin_pitch = clamp(2.0 * (w * y - z * x), -1.0, 1.0)
    :math.asin(sin_pitch)
  end

  @doc """
  Pure inner velocity loop + closed yaw-rate turn loop (ADR-0009 + amendment).
  Given the `balance_torque` (the PID's upright-holding torque, applied to both
  wheels), the teleop intent `%{forward, turn}` (both clamped to `[-1, 1]`), the
  latest measured per-wheel speeds `%{left, right}` (rad/s), the measured chassis
  yaw rate `measured_yaw` (rad/s, the IMU gyro's body-Z = `Vec3.z(angular_velocity)`),
  and the loop params `%{max_speed, max_yaw_rate, kyaw, kv, output_clamp}`, return
  the clamped per-wheel torque `%{left, right}`:

      fwd_target      = forward · max_speed                       # forward SPEED loop
      target_yaw_rate = turn · max_yaw_rate                       # turn commands a YAW RATE
      turn_torque     = kyaw · (target_yaw_rate − measured_yaw)   # closed on measured yaw
      left  = clamp(balance_torque + kv · (fwd_target − measured_left)  − turn_torque, ±output_clamp)
      right = clamp(balance_torque + kv · (fwd_target − measured_right) + turn_torque, ±output_clamp)

  Two host control laws ride on top of the balance torque, both ADDITIVE (neither
  changes the balance sign):

    * **Forward** is a closed wheel-SPEED loop (unchanged from the original
      ADR-0009): `forward` sets a per-wheel target speed and `kv·(target − measured)`
      bleeds the speed error, so "forward 0.3" rolls at a bounded rate, not a
      runaway.
    * **Turn** is a closed YAW-RATE loop (the amendment): `turn` commands a target
      yaw rate and `turn_torque = kyaw·(target_yaw_rate − measured_yaw)` drives a
      DIFFERENTIAL torque (−turn_torque left, +turn_torque right) so the measured
      yaw tracks the command. Because the differential is regulated by the measured
      yaw, it CANNOT run away: as the bot yaws faster the error shrinks and the
      differential backs off — so even a sustained hard turn (turn=1.0) is
      self-limiting and stays stable, where the old open-loop differential of the
      speed targets eventually toppled the pitch-only balancer.

  With zero teleop both targets are 0, so the loop holds the wheels at zero speed
  and zero yaw. The final per-wheel torque is clamped symmetrically to
  `output_clamp` so neither term can blow past the actuator authority.
  """
  @spec velocity_mix(float(), map(), map(), float(), map()) :: map()
  def velocity_mix(
        balance_torque,
        %{forward: fwd, turn: turn},
        %{left: measured_left, right: measured_right},
        measured_yaw,
        %{
          max_speed: max_speed,
          max_yaw_rate: max_yaw_rate,
          kyaw: kyaw,
          kv: kv,
          output_clamp: out_clamp
        }
      ) do
    fwd = clamp(fwd * 1.0, -1.0, 1.0)
    turn = clamp(turn * 1.0, -1.0, 1.0)

    fwd_target = fwd * max_speed

    # Closed yaw-rate loop: turn sets a target yaw rate, the differential torque is
    # regulated by the MEASURED yaw so a sustained turn is self-limiting.
    target_yaw_rate = turn * max_yaw_rate
    turn_torque = kyaw * (target_yaw_rate - measured_yaw * 1.0)

    left = balance_torque + kv * (fwd_target - measured_left) - turn_torque
    right = balance_torque + kv * (fwd_target - measured_right) + turn_torque

    %{
      left: clamp(left * 1.0, -out_clamp, out_clamp),
      right: clamp(right * 1.0, -out_clamp, out_clamp)
    }
  end

  defp clamp(v, lo, _hi) when v < lo, do: lo
  defp clamp(v, _lo, hi) when v > hi, do: hi
  defp clamp(v, _lo, _hi), do: v

  # The latest measured per-wheel speeds, shaped for velocity_mix/5.
  defp measured(%{measured_left: l, measured_right: r}), do: %{left: l, right: r}

  # Seconds since the last pose tick (from monotonic nanoseconds). The first
  # sample (or a non-advancing/backwards clock) yields 0.0 — the filter then
  # anchors fully to the accel estimate and the PID skips its integral/derivative.
  defp dt_since(%{last_mono: nil}, _msg), do: 0.0

  defp dt_since(%{last_mono: prev}, %BB.Message{monotonic_time: now})
       when now > prev,
       do: (now - prev) / 1.0e9

  defp dt_since(_state, _msg), do: 0.0

  # ----------------------------------------------------------------------------
  # Live enable/disable helpers
  # ----------------------------------------------------------------------------

  @doc "Enable balance live on the named controller of `robot` (default `:balance`)."
  @spec enable(module(), atom()) :: :ok
  def enable(robot, name \\ :balance), do: toggle(robot, name, true)

  @doc "Disable balance live (publishes zero torque; resets the integrator)."
  @spec disable(module(), atom()) :: :ok
  def disable(robot, name \\ :balance), do: toggle(robot, name, false)

  defp toggle(robot, name, on?) do
    BB.Process.cast(robot, name, {:balance_enable, on?})
  end

  # ----------------------------------------------------------------------------
  # BB.Controller callbacks
  # ----------------------------------------------------------------------------

  @impl BB.Controller
  def init(opts) do
    bb = Keyword.fetch!(opts, :bb)

    pid = %Pid{
      kp: opts[:kp] * 1.0,
      ki: opts[:ki] * 1.0,
      kd: opts[:kd] * 1.0,
      integral_clamp: opts[:integral_clamp] * 1.0,
      output_clamp: opts[:output_clamp] * 1.0
    }

    state = %{
      bb: bb,
      pose_topic: opts[:pose_topic],
      teleop_topic: opts[:teleop_topic],
      vel_left_topic: opts[:vel_left_topic],
      vel_right_topic: opts[:vel_right_topic],
      left_path: opts[:left_actuator_path],
      right_path: opts[:right_actuator_path],
      pid: pid,
      target_pitch: opts[:target_pitch] * 1.0,
      # inner velocity-loop + yaw-rate-loop params (ADR-0009 + amendment),
      # pre-bundled for velocity_mix/5.
      vel_params: %{
        max_speed: opts[:max_speed] * 1.0,
        max_yaw_rate: opts[:max_yaw_rate] * 1.0,
        kyaw: opts[:kyaw] * 1.0,
        kv: opts[:kv] * 1.0,
        output_clamp: opts[:output_clamp] * 1.0
      },
      enabled: opts[:enabled],
      # Whether the robot is ARMED. The balance loop is the wheels' sole commander,
      # so on disarm it MUST stop publishing — the on-chip floor's safe-state is
      # reached by command-silence (§05), and a controller that keeps commanding
      # defeats disarm (it re-advances the floor's seq every tick). Seeded from the
      # current safety state so a boot-disarmed robot drives nothing until armed;
      # updated by `handle_safety_state_change/2` (disarm) and the `:armed`
      # transition (re-arm). See ADR-0010.
      armed: BB.Safety.state(bb.robot) == :armed,
      last_teleop: %{forward: 0.0, turn: 0.0},
      # latest measured per-wheel speed (rad/s) from the vel_* sensor streams.
      measured_left: 0.0,
      measured_right: 0.0,
      last_mono: nil,
      # running complementary-filter pitch estimate (radians), advanced each tick
      pitch: 0.0
    }

    BB.subscribe(bb.robot, state.pose_topic, message_types: [BB.Message.Sensor.Imu])
    BB.subscribe(bb.robot, state.teleop_topic)

    BB.subscribe(bb.robot, state.vel_left_topic, message_types: [BB.Message.Sensor.JointState])
    BB.subscribe(bb.robot, state.vel_right_topic, message_types: [BB.Message.Sensor.JointState])

    {:ok, state}
  end

  # Disarm (or error): stop commanding the wheels. The framework calls this on a
  # transition to :disarming/:disarmed/:error (BB.Controller, the lostbean fork).
  # We keep running (controllers are long-lived) but flip `armed: false` so
  # `command/2` publishes nothing — the wheels go to command-silence and the floor
  # reaches its safe state (§05 / ADR-0010). The matching re-arm is the `:armed`
  # state-machine transition below.
  @impl BB.Controller
  def handle_safety_state_change(_disarm_state, state) do
    {:continue, %{state | armed: false}}
  end

  # A pose tick while DISABLED: zero BALANCE torque (the chassis is not actively
  # held upright), but the inner velocity loop STILL runs — so an operator can
  # drive the wheels at a bounded target speed with balance off (`balance_torque =
  # 0`, output = `kv·(target − measured)`). Do NOT advance the PID (no windup while
  # off), but DO advance the complementary filter so the pitch estimate stays live
  # for a clean re-enable (no settling jump on the first enabled tick).
  @impl BB.Controller
  def handle_info(
        {:bb, topic, %BB.Message{payload: %BB.Message.Sensor.Imu{} = imu} = msg},
        %{pose_topic: topic, enabled: false} = state
      ) do
    pitch = step_pitch(state.pitch, imu, dt_since(state, msg))
    measured_yaw = Vec3.z(imu.angular_velocity)

    command(
      state,
      velocity_mix(0.0, state.last_teleop, measured(state), measured_yaw, state.vel_params)
    )

    {:noreply, %{state | last_mono: msg.monotonic_time, pitch: pitch}}
  end

  # A pose tick while ENABLED: accel/gyro complementary filter → pitch → PID →
  # balance_torque, then the inner velocity loop (teleop target speed + measured
  # speed) on top, command both wheels.
  def handle_info(
        {:bb, topic, %BB.Message{payload: %BB.Message.Sensor.Imu{} = imu} = msg},
        %{pose_topic: topic} = state
      ) do
    dt_s = dt_since(state, msg)

    pitch = step_pitch(state.pitch, imu, dt_s)
    measured_yaw = Vec3.z(imu.angular_velocity)
    error = state.target_pitch - pitch
    {torque, new_pid} = step(state.pid, error, dt_s)

    command(
      state,
      velocity_mix(torque, state.last_teleop, measured(state), measured_yaw, state.vel_params)
    )

    {:noreply, %{state | pid: new_pid, last_mono: msg.monotonic_time, pitch: pitch}}
  end

  # A measured wheel-speed reading on the left vel topic (ADR-0009): keep the
  # latest measured speed for the inner velocity loop. The lifted JointState
  # carries this one wheel's velocity as `velocities: [rad_s | _]`.
  def handle_info(
        {:bb, topic, %BB.Message{payload: %BB.Message.Sensor.JointState{velocities: [v | _]}}},
        %{vel_left_topic: topic} = state
      ) do
    {:noreply, %{state | measured_left: v * 1.0}}
  end

  def handle_info(
        {:bb, topic, %BB.Message{payload: %BB.Message.Sensor.JointState{velocities: [v | _]}}},
        %{vel_right_topic: topic} = state
      ) do
    {:noreply, %{state | measured_right: v * 1.0}}
  end

  # Teleop intent on the PubSub topic — the bb_tui path. The Teleop command's
  # handler publishes a `BB.Message.Geometry.Twist`; we read forward from
  # `linear.x` and turn from `angular.z` (the ROS-style convention) and keep the
  # latest intent, clamped, to mix into the next pose tick. Any other payload on
  # the topic is ignored (we never guess an intent).
  def handle_info(
        {:bb, topic, %BB.Message{payload: %Twist{} = twist}},
        %{teleop_topic: topic} = state
      ) do
    pad = %{
      forward: clamp(Vec3.x(twist.linear) * 1.0, -1.0, 1.0),
      turn: clamp(Vec3.z(twist.angular) * 1.0, -1.0, 1.0)
    }

    {:noreply, %{state | last_teleop: pad}}
  end

  def handle_info({:bb, topic, %BB.Message{}}, %{teleop_topic: topic} = state) do
    {:noreply, state}
  end

  # Teleop intent delivered as a bare map (the in-process/test path).
  def handle_info({:teleop, %{forward: _, turn: _} = pad}, state) do
    {:noreply, %{state | last_teleop: pad}}
  end

  # Live enable/disable. Resets the integrator on any toggle so re-enabling
  # starts clean (no windup carryover). Accepted both
  # as a `handle_info` (a raw `send`, e.g. in tests) and a `handle_cast` (the
  # `enable/1` / `disable/1` helpers, which `BB.Process.cast`).
  def handle_info({:balance_enable, on?}, state) when is_boolean(on?) do
    {:noreply, set_enabled(state, on?)}
  end

  # Re-arm: the safety state machine transitioned to :armed. The disarm states are
  # handled by `handle_safety_state_change/2` (which the framework dispatches and
  # consumes); the `:armed` transition is NOT a disarm state, so the framework
  # forwards it here. Resume commanding the wheels (see ADR-0010).
  def handle_info(
        {:bb, [:state_machine], %BB.Message{payload: %{to: :armed}}},
        state
      ) do
    {:noreply, %{state | armed: true}}
  end

  def handle_info(_other, state), do: {:noreply, state}

  @impl BB.Controller
  def handle_cast({:balance_enable, on?}, state) when is_boolean(on?) do
    {:noreply, set_enabled(state, on?)}
  end

  def handle_cast(_other, state), do: {:noreply, state}

  defp set_enabled(state, on?) do
    %{state | enabled: on?, pid: %{state.pid | integral: 0.0, prev_error: 0.0}}
  end

  # Command both wheels by publishing a typed Effort to each actuator topic. We
  # do NOT write any slot directly — the actuator view is the single writer
  # (§04); it pattern-matches exactly this `%BB.Message{payload: %Effort{}}` on
  # `[:actuator | path]` and turns it into a slot write.
  #
  # We build the message struct directly rather than via `BB.Actuator.set_effort/4`:
  # that helper passes `duration: nil` to `BB.Message.new!`, which the Effort
  # schema rejects (`duration` is an optional :pos_integer with no default, so the
  # key must be OMITTED, not nil). A bare struct sidesteps that and is exactly the
  # shape the view (and the §02 slice tracer) expects.
  # DISARMED: publish nothing. The wheels' command slots stop advancing, the floor
  # sees command-silence, and the hub reaches its safe state (§05 / ADR-0010). A
  # controller that kept commanding through disarm would re-arm the floor every
  # tick and defeat the disarm — so the safe-state action IS to fall silent.
  defp command(%{armed: false}, _mixed), do: :ok

  defp command(state, %{left: left, right: right}) do
    BB.publish(state.bb.robot, [:actuator | state.left_path], effort_message(left))
    BB.publish(state.bb.robot, [:actuator | state.right_path], effort_message(right))
    :ok
  end

  defp effort_message(nm) do
    %BB.Message{
      monotonic_time: System.monotonic_time(:nanosecond),
      wall_time: System.system_time(:nanosecond),
      node: Node.self(),
      frame_id: :effort,
      payload: %BB.Message.Actuator.Command.Effort{effort: nm * 1.0}
    }
  end
end
