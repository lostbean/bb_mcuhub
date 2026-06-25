# A wheel reports its measured speed as a sensor port, and the host closes a velocity loop on top of balance

Each wheel hub exposes its **measured angular velocity** (rad/s) as a first-class
**sensor port** — `vel_left` / `vel_right`, a new consumer value-type
`WheelSpeed` (one `:f32`), `dir: :out`, surfaced through the **same `BB.Sensor`
view + `[:sensor | …]` topic the IMU pose already uses**. The host balance
controller subscribes to those two topics and runs an **inner velocity loop**:
operator teleop sets a per-wheel _target speed_, and the controller adds
`kv · (target − measured)` to the per-wheel balance torque. The wheel command stays
**torque** end-to-end (faithful to the real MKS Dual FOC's torque-voltage mode); the
velocity loop is a host control law on top, not a change of actuator type.

**Scope: this is an `examples/segby_v1` change only — the `bb_mcuhub` library is NOT
modified.** Everything here is consumer-side and rides the library's _existing_
extension seams (ADR-0003): `WheelSpeed` is a consumer value-type built on the stock
`BBMCUHub.ValueType` behaviour (like the example's own `Range`/`Led`); `vel_left`/
`vel_right` are ports on the example's wheels hub using the stock port DSL; the codec,
the `BB.Sensor` view, the generator, and the drift test are unchanged library
mechanisms that already render and route any consumer's ports. The library learns
nothing new — this exercises its extensibility, it does not change it.

This is recorded because, **within the example**, it adds a new sensor port + value-type
to the v1 robot, it **closes a feedback path the balance loop never had** (today balance
is open-loop on wheel state — it targets pitch only), and it changes how an operator
drives the bot (a bounded _speed_ setpoint instead of an unbounded _torque_ bias) — a
control-law change worth pinning before it is tuned against.

## The problem

Driving the balanced bot is unusable. The teleop command biases the wheels with a
**torque** (`mix/4` adds `forward · max_forward` to each wheel's commanded torque).
But on a balancing bot **torque is acceleration**: any non-trivial forward command
keeps accelerating the wheels, they run away, the chassis pitches hard to chase
them, and the bot rockets off and falls. In the MuJoCo sim only `forward ≈ 0.001`
is usable; anything larger is a runaway. The operator wants "roll forward at a
steady pace," and a torque bias cannot express that — it expresses "accelerate
forever."

The fix every wheeled balancer uses is to command **speed**, not acceleration: a
velocity loop holds the wheel at a target rate, so "forward 0.3" means "roll at a
bounded rate," and the loop bleeds off the error instead of integrating it into a
runaway. The real segby motors already run **SimpleFOC closed-loop velocity mode**
(bench-verified) — they _know_ their shaft velocity. The information exists; it is
simply never reported to the host, and the host never closes a loop on it.

Two facts make this more than "add a field":

- **The host controller has no way to read wheel state today.** `SegbyV1.Balance`
  subscribes to exactly one topic — the IMU pose — and only _publishes_ effort. The
  wheels' `:status` value (`applied_seq`, `floored?`) is **not** a thing the
  controller can read: status is consumed _internally_ by the actuator view's
  born-stale liveness check (read straight from the registry slot, never published
  as a `BB.Message`). So there is **no existing seam** for the controller to learn a
  wheel's speed. A naive "put velocity in `:status`" would force inventing one — and
  would conflate the actuator's _liveness flag_ with _measurement telemetry_.
- **The wheel command must stay torque.** The real MKS Dual FOC is driven in
  torque-voltage mode (effort = q-axis voltage, per BRINGUP). Swapping the sim to a
  MuJoCo `<velocity>` actuator would make the sim a _different plant_ than the
  hardware — defeating the sim's whole purpose (ADR-0008: shrink the sim-to-real
  gap). The velocity loop has to be a **control law on top of torque**, not a
  command-type change.

## The design

### Wheel speed is a sensor port (not a `:status` field)

A wheel's measured speed is a **measurement**, so it rides the **sensor** seam — the
same one pose and range already use — not the actuator's liveness `:status`. The
wheels hub gains two **output ports**:

- `vel_left`, `vel_right` — `dir: :out`, value-type **`WheelSpeed`** (a new
  consumer value-type in the example, layout `{rad_s: :f32}`, lifting to a typed
  `BB.Message` the controller reads), ~50 Hz.

`:status` is **untouched** — it stays `applied_seq + floored`, the pure liveness
flag. Liveness and measurement stay cleanly separated. The per-wheel shape
(`vel_left`/`vel_right`) mirrors the existing `motor_*`/`status_*` pairing — each
wheel its own port, consistent with the hub's per-motor symmetry.

This choice is what makes the host-read seam _free_: a `BBMCUHub.BBHub.Sensor` view
over each velocity port publishes it on `[:sensor | path]`, and the controller
`BB.subscribe`s to it **exactly as it subscribes to the IMU pose** — no new
mechanism, no breaking the status abstraction, no registry back-door.

### Each stratum reports the speed it has

- **Firmware** (real bot): the generated sense-port read hook returns
  `motor.shaft_velocity` (rad/s) per wheel — the FOC already computes it in the
  closed-loop velocity drive. Same hook shape pose/range already use; a motor whose
  encoder did not answer reports 0.0 (it is not driving anyway).
- **Sim plant**: `SegbyV1.Sim.MujocoPlant` maps MuJoCo's `qvel[0]`/`qvel[1]` (the
  wheel hinge angular velocities — already in the child's `state` reply, currently
  dropped) into the two `WheelSpeed` sensor values.
- **Generator / drift**: `mix wire.gen` regenerates the contract, glue, headers,
  and parity vectors for the two new ports; the drift test re-pins. The floor is
  **untouched** — it is command-side; a new _sensor_ port does not touch the
  safe-action machinery.

### The host velocity loop (proposed law — for review)

`SegbyV1.Balance` subscribes to `vel_left` / `vel_right`, keeping the latest
measured speed per wheel. Teleop `forward` / `turn` become a **per-wheel target
speed**, and the velocity error is fed forward as torque on top of the balance
torque:

```
target_left  = forward · max_speed − turn · max_turn_speed
target_right = forward · max_speed + turn · max_turn_speed

left_torque  = balance_torque + kv · (target_left  − measured_left)
right_torque = balance_torque + kv · (target_right − measured_right)
```

- `max_speed` (rad/s) and `max_turn_speed` (rad/s) replace today's
  `max_forward` / `max_turn` torque biases; `kv` is the velocity-loop gain.
- With zero teleop, `target = 0`: the loop actively **holds the wheels at zero
  speed** (resisting drift), which also _helps_ balance hold position rather than
  creeping. Balance torque still dominates the upright dynamics; the velocity term
  is the gentle outer authority that makes driving bounded.
- Concrete numbers (`max_speed`, `max_turn_speed`, `kv`) are **tuned against the
  real closed loop** (real `SegbyV1.Balance` + real MuJoCo, headless) and shown for
  sign-off — not guessed. The control law stays **on the host** (the project's
  pattern: the MCU _reports_, the host _closes the loop_).

### Amendment — turn is a closed yaw-rate loop, not open-loop differential speed

The law above (as first built) makes **forward** a closed wheel-speed loop, but
**turn** an _open-loop_ differential of the wheel-speed targets: it differentials
the targets and hopes the chassis yaw follows. It does, gently — but a **sustained
hard turn** has no feedback regulating the actual yaw, so yaw-coupled energy the
pitch-only balance PID cannot reject builds up and the bot eventually topples. This
is gain-independent: lowering `max_turn_speed` only delays it. (The original ADR
flagged this as a known limit.)

The fix closes a loop on the **measured yaw rate** — which is already on the wire:
the IMU's `angular_velocity.z` is the chassis yaw rate (verified in MuJoCo: a
differential torque spins the gyro's Z axis, X/Y ≈ 0). `turn` now commands a
**target yaw rate** (rad/s), and a yaw-rate controller drives the _differential_
torque so the measured yaw tracks it:

```
fwd_target      = forward · max_speed                 # the (kept) forward speed loop
target_yaw_rate = turn · max_yaw_rate                 # turn commands a YAW RATE now
turn_torque     = kyaw · (target_yaw_rate − gyro_z)   # closed on measured yaw

left_torque  = balance_torque + kv · (fwd_target − measured_left)  − turn_torque
right_torque = balance_torque + kv · (fwd_target − measured_right) + turn_torque
```

Because the differential torque is regulated by the _measured_ yaw, it **cannot run
away**: as the bot yaws faster, `(target_yaw_rate − gyro_z)` shrinks and the
differential backs off. Any commanded turn rate is self-limiting, so a sustained
hard turn stays stable.

- `max_turn_speed` is **replaced** by `max_yaw_rate` (rad/s, the commandable yaw
  rate at `turn = 1`) and `kyaw` (the yaw-loop gain). `max_speed`/`kv` (forward) are
  unchanged. Numbers tuned headless against the real loop.
- This is still **example-only and on the host** — no new port, no firmware change
  (gyro Z already rides the IMU pose). It refines this ADR's host control law; it
  does not change the wire contract.

### What it is, and is not

- **It is** the faithful fix: torque command preserved (matches the FOC), velocity
  _reported_ from where it is truly known (FOC / MuJoCo), loop _closed_ on the host.
  An operator drives a bounded speed; the sim and the hardware see the same plant.
- **It is not** a velocity actuator. The MJCF stays `<motor>` (torque); nothing
  about the wheel command type changes on either stratum.
- **It is not** a `:status` change. Status remains the liveness flag; velocity is a
  separate sensor stream on the existing sensor seam.
- **It is not** a floor change. The floor guards the _command_; a new sensor port is
  orthogonal to the safe-action guarantee.

## Consequences

- **The example's wheels hub grows two output ports + a consumer value-type**
  (`WheelSpeed`). This is a change to the example robot's contract, **not** the
  library's: the (unchanged) codec renders the two new ports, the example's firmware
  grows two sense hooks, the sim plant reports two more values, and the example's
  drift test re-pins. All regenerated, all drift-tested — no hand-edited glue, and no
  `lib/bb_mcuhub` edit.
- **The balance controller gains feedback it never had.** It moves from open-loop on
  wheel state (pitch-only) to reading wheel speed — a strictly larger input surface,
  and the velocity terms must be tuned so they never fight the balance loop.
- **Teleop semantics change** from torque-bias to speed-setpoint. The `:teleop`
  command's `forward`/`turn` now mean target speed, not torque — documented at the
  command and in `SegbyV1.Balance`.
- **Faithful both ways.** Because velocity is reported (not commanded) and the loop
  is host-side, the sim and the real bot run the identical control law over the
  identical (torque) plant — exactly the sim-to-real fidelity ADR-0008 exists to
  protect.

## Open questions (for implementation, not this decision)

- **`kv` and the speed ranges** — tuned against the real loop; the law above is the
  shape, the numbers come from the headless real-loop harness (ADR-0008's
  `SEGBY_SIM_HEADLESS` mode) and are shown for sign-off.
- **Velocity smoothing** — MuJoCo `qvel` is clean; the real FOC `shaft_velocity` is
  noisier. A light host-side filter on the measured speed may be warranted; deferred
  until the real bot is on the bench.
- **`WheelSpeed` placement** — a consumer value-type in `examples/segby_v1` (like
  `Range`/`Led`), not a library stock type, unless a second consumer needs it.
