# A host control loop must fall silent on disarm — it produces the command-silence the floor's safe-state needs

A host **control loop** (a `BB.Controller` that commands actuators every tick) must
**stop publishing commands when the robot is not armed**. On disarm it falls silent;
the actuator command slots stop advancing; the on-chip **floor** sees command-silence
and reaches its safe state (§05). The framework gains the seam to make this possible —
`BB.Controller.handle_safety_state_change/2` (added in the `lostbean/bb` fork, mirroring
`BB.Command`) — and the controller is responsible for gating its own output through it.

This is recorded because it **fixes a real safety bug on both the sim and the hardware**
(disarm did not actually stop a self-balancing robot), it identifies an **unstated
premise in the §05 safety model** that an always-commanding controller violates, and it
adds a **framework seam** (a fork of `bb`) plus a **consumer contract** (controllers must
gate on safety state) that future control loops must honor.

## The problem

Pressing disarm in the dashboard did **not** stop the wheels — on the MuJoCo sim **and**,
by the same root cause, on the real robot.

The §05 safe-state is the on-chip floor, and its mechanism is **command-silence**: if no
fresh command `seq` advances within the floor's window (~100 ms), the floor latches
disarmed and drives the safe action. The broadcast-disarm frame (NODE `0x00`) is — by
deliberate design — an **accelerator, not the mechanism** (the firmware has no active
`0x00` handler; adding one would be a second safety path that could itself fail). So
disarm is _supposed_ to work because the host's command stream stops.

The §05 model carries an **unstated premise: "disarm ⇒ the host stops commanding."** It
holds for every case the design enumerates — a dead parent, a cut bus, a crashed host —
where the stream _physically_ halts. But a **self-balancing controller structurally
violates it**: it must keep commanding every tick to stay upright. `SegbyV1.Balance`
subscribed only to pose/teleop/velocity, knew nothing of the safety state, and published
`Effort` to both wheels on every pose tick **regardless of arm state**. So on disarm it
kept re-advancing the floor's `seq` (~10–20 ms, well inside the 100 ms window), the floor
never saw silence, and the wheels kept running. The broadcast frame lost the race against
the very next pose tick.

Two structural facts made the gap total:

- **No host component was wired to produce the silence on disarm.** BB's disarm fans out
  only per-actuator `disarm/1` callbacks (a one-shot, wire-level "make hardware safe"
  intent — for our actuator view, just the broadcast frame). It never suspends or gates
  the **controllers** that feed those actuators.
- **BB had the gate — but only for `BB.Command`, not `BB.Controller`.** `BB.Command`
  subscribes to `[:state_machine]` and dispatches `handle_safety_state_change/2` (default:
  stop). `BB.Controller` had no such hook. A continuously-publishing controller therefore
  defeats disarm by design, and the framework offered it no way to know it should stop.

And no test caught it because **every disarm test exercised the floor mechanism in
isolation** — injecting commands manually (`VirtualHub.broadcast_disarm` with hand-issued
commands) or running idle — and **never put a live controller behind a disarm**. The test
coverage mirrored the flawed premise, so the test gap could not have caught the design
gap. (Found via a five-whys; three independent chains converged on this root.)

## The design

The §05 floor-silence model is **right and stays unchanged**. The fix supplies the
host-side silence the model always assumed, for a controller that won't go silent on its
own — split across a framework seam and a consumer contract.

### The framework seam — `BB.Controller.handle_safety_state_change/2` (the `lostbean/bb` fork)

`BB.Controller` gains the same safety hook `BB.Command` already has, with the
**controller-correct default**:

- `BB.Controller.Server` subscribes to `[:state_machine]` in `init` and, on a transition
  to `:disarming` / `:disarmed` / `:error`, dispatches the controller's
  `handle_safety_state_change(new_state, state)` (other transitions, including `:armed`,
  fall through to the controller's `handle_info/2`).
- The default is `{:continue, state}` — **not** `BB.Command`'s `{:stop, ...}`. A
  controller is **long-lived**; stopping and restarting it every arm/disarm cycle would be
  wrong. So the default keeps it running, and the controller is responsible for **gating
  its own output**. The framework only _notifies_; it cannot know what "stop output" means
  for an arbitrary controller, so the controller decides.

This is a fork of the upstream `bb` hex package (`beam-bots/bb` → `lostbean/bb`,
`feat/controller-safety-state-hook`), proposed upstream (beam-bots/bb#160); `bb_tui` is
already wired the same way (ADR-0003-style consumer fork). `BB.Command` is untouched.

### The consumer contract — the control loop gates on armed state

`SegbyV1.Balance` (the wheels' sole commander) now:

- **Seeds `armed` from `BB.Safety.state` at init** — so a robot that boots `:disarmed`
  (the safe default) drives nothing until armed.
- **Implements `handle_safety_state_change/2`** → `armed: false` on any disarm transition.
- **Catches the `:armed` transition** in `handle_info` → `armed: true` (re-arm).
- **Gates `command/2`**: `command(%{armed: false}, _) -> :ok` — publishes **nothing** while
  disarmed. The command slots stop advancing → the floor sees silence → safe state. (The
  broadcast frame, still sent, now genuinely accelerates a silence that actually occurs.)

So on disarm the bot's wheels go limp (the real floor safe-state); on re-arm, commanding
resumes. `mix segby.sim` arms at boot, since it stands up a live, balancing bot — the
dashboard's safety panel then matches reality, and an operator's disarm visibly stops it.

### What it is, and is not

- **It is** the host-side producer of the command-silence the §05 floor always assumed —
  the missing half for an always-commanding controller. The floor remains the guarantee;
  this just makes the host actually fall silent.
- **It is not** a change to the floor / born-disarmed / broadcast-accelerator design.
  That is sound; no firmware change, no active `0x00` handler (which §05 deliberately
  forbids).
- **It is not** segby-specific in its framework half: any `BB.Controller` on any robot now
  gets the safety hook and can gate its output. The contract — _a control loop must fall
  silent on disarm_ — is general.

## Consequences

- **A new general contract for control loops:** a `BB.Controller` that commands actuators
  must gate its output on safety state (subscribe via the hook, stop publishing when not
  armed). The next balance/drive/arm controller author must honor this, or it will defeat
  disarm the same way. Captured as a CONTEXT term.
- **A `bb` fork to track.** The dep moves from hex `~> 0.20` to the `lostbean/bb` fork
  branch (`override: true`, since `bb_tui` still wants hex `bb`). Carries until the hook
  lands upstream (beam-bots/bb#160), then the dep returns to hex.
- **The regression seam that was missing now exists:** a test that runs a **live**,
  enabled, armed balance controller, disarms it, and asserts **no** Effort reaches the
  wheels (and resumes on re-arm). It fails on the old code. The ViewHarness was taught to
  mirror `BB.Controller.Server`'s disarm routing so a harness-driven controller exercises
  the real path.
- **A bench step is added** (BRINGUP): with balance ON, press disarm and confirm the wheels
  stop within the floor window — the end-to-end operator check that was asserted but never
  verified.

## Open questions (deferred)

- **Defense-in-depth in the library:** the actuator view (`BBMCUHub.BBHub.Actuator`,
  the sole slot writer) could _also_ refuse writes when disarmed (a fast `BB.Safety.armed?`
  read), so even a misbehaving controller cannot defeat disarm. A worthwhile library
  hardening, but a backstop — the root fix is the controller gating its output. Deferred.
- **Upstream landing:** once beam-bots/bb#160 merges and ships, drop the fork and return
  the dep to hex `bb`.
