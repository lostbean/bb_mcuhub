# Design changelog

Decision history and rationale for the hub-gateway design. The design doc
(`docs/hub-design.html`) is kept **stateless** — it always describes the current
intended design with no history or "we changed X" framing. This file is where the
_why_ and the _when_ live: each entry says what changed in the design and the
reason it changed.

Format: newest first. Dates are absolute.

---

## 2026-06-25 — Turn becomes a closed yaw-rate loop, not open-loop differential speed (ADR-0009 amendment)

ADR-0009's first build made **forward** a closed wheel-speed loop but **turn** an
_open-loop_ differential of the wheel-speed targets — nothing regulated the actual chassis
yaw, so a _sustained hard turn_ pumped yaw-coupled energy the pitch-only balance PID can't
reject and the bot eventually toppled (gain-independent; lowering `max_turn_speed` only
delayed it). The original ADR flagged this as a known limit.

Resolved by closing a loop on the **measured yaw rate**, which is already on the wire: the
IMU's `angular_velocity.z` IS the chassis yaw rate (verified in MuJoCo — a differential
torque spins the gyro Z axis, X/Y ≈ 0). `turn` now commands a _target yaw rate_, and a
yaw-rate controller drives the differential torque so the measured yaw tracks it:
`turn_torque = kyaw · (turn · max_yaw_rate − gyro_z)`, subtracted from the left wheel and
added to the right, on top of the unchanged forward speed loop. Because the differential is
regulated by the measured yaw, it can't run away — any commanded turn rate self-limits, so
sustained hard turns stay stable. `max_turn_speed` is replaced by `max_yaw_rate` + `kyaw`
(tuned headless against the real loop). Still example-only and host-side — no new port, no
firmware change (gyro Z already rides the IMU pose); it refines the ADR-0009 host control
law, not the wire contract. See the ADR-0009 amendment.

---

## 2026-06-25 — A wheel reports its speed as a sensor port; the host closes a velocity loop on top of balance (ADR-0009, design-only)

Driving the (now-balancing) sim bot is unusable: teleop biases the wheels with **torque**,
but on a balancer torque is acceleration, so any non-trivial forward command runs the
wheels away and the bot falls — only `forward ≈ 0.001` is usable. The fix every wheeled
balancer uses is to command **speed**: a velocity loop holds a bounded target rate instead
of integrating a runaway. The real segby motors already run SimpleFOC closed-loop velocity
mode (they know their shaft velocity); the information simply never reaches the host, and
the host never closes a loop on it.

Two constraints shaped the design. (1) The wheel command must stay **torque** — the real
MKS Dual FOC is driven torque-voltage; swapping the sim to a MuJoCo `<velocity>` actuator
would make the sim a different plant than the hardware, defeating ADR-0008's sim-to-real
purpose. So the velocity loop is a host control law _on top of_ torque, not a command-type
change. (2) The host controller has **no seam to read wheel state today** — `SegbyV1.Balance`
subscribes only to the IMU pose, and `:status` (`applied_seq`/`floored`) is consumed
internally by the actuator view's liveness check, never published. So velocity could not
just be a `:status` field (that would conflate liveness with measurement and still need a
new read seam).

ADR-0009 resolves both: a wheel's measured speed is a **separate sensor port**
(`vel_left`/`vel_right`, a new consumer value-type `WheelSpeed` = one `:f32` rad/s,
`dir: :out`), surfaced through the **same `BB.Sensor` view + `[:sensor | …]` topic the IMU
pose uses** — so the controller subscribes to it exactly like pose, with no new mechanism
and `:status` left untouched (liveness stays separate from measurement). The firmware sources
it from the FOC `shaft_velocity`; the sim plant from MuJoCo's `qvel` (already in the child's
reply, currently dropped). The host then runs an inner loop: teleop `forward`/`turn` set a
per-wheel target speed, and the controller adds `kv · (target − measured)` to the per-wheel
balance torque — so "forward" is a bounded speed, not a runaway acceleration.

**Scope: `examples/segby_v1` only — the `bb_mcuhub` library is NOT changed.** Unlike
ADRs 0005–0008, this touches no `lib/bb_mcuhub`: `WheelSpeed` is a consumer value-type on
the stock `BBMCUHub.ValueType` behaviour, `vel_left`/`vel_right` use the stock port DSL,
and the codec / `BB.Sensor` view / generator / drift test are unchanged library mechanisms
that already render any consumer's ports. It exercises the library's extensibility (ADR-0003),
it does not change it. Within the example it is a contract change (two new output ports +
value-type → regen + drift), it closes a feedback path balance never had (pitch-only →
wheel-speed-aware), and it changes teleop semantics (speed setpoint, not torque bias).
Faithful both ways: velocity is _reported_ not commanded, the loop is host-side, so sim and
hardware run the identical control law over the identical torque plant. Design-only; not yet
implemented. See ADR-0009 for the proposed control law (numbers tuned against the real loop,
shown for sign-off) and the per-stratum change map.

---

## 2026-06-25 — A robot can run virtually: a sim transport closes the host loop over a physics engine (ADR-0008, design-only)

The chassis is not yet assembled, and even once it is, you do not want an un-tuned
balance loop's first real test to be on hardware that can hurt itself. Every existing
hardware-free path stops short of "a real, moving, controllable robot with faithful
dynamics": the example's `LoopbackTransport` has no plant (a motor command goes out and
nothing moves), and the test `VirtualHub` runs the real C floor but on a **frozen clock**
for deterministic assertion — it is `:test`-only and has no real-time loop. So there is no
way to _develop_ control software against a virtual bot and shrink the sim-to-real gap
before the bench.

ADR-0008 closes that by simulating at **the one boundary the whole stack already pivots
on — the transport**. A new `BBMCUHub.Sim` transport (a third implementation of the
existing `BBMCUHub.Host.Transport` behaviour, beside UART and loopback) plays the hub
tree against a physics engine:

- It captures each outbound actuator command from `send/2` (which is event-driven and
  clock-free), advances a **`Plant`** (a new public library behaviour — consumer-supplied
  dynamics, robot-agnostic, speaking value-type values keyed by wire slot, in the spirit
  of ADR-0005), and injects the resulting sensor readings back up as wire bodies built
  with `Codec.encode_body` and delivered as `{:circuits_uart, :sim, body}`.
- Everything **above** the transport is the unchanged real stack — codec, freshness
  monitor (born-stale honored via the injected `seq`/`t_dev`), floor semantics, views,
  `bb_tui` over BB PubSub. Only the transport knows the robot is virtual.
- The chosen built-in plant is **MuJoCo over a Port** (faithful-from-the-start, the
  decision): a headless Python child running MuJoCo's `launch_passive` viewer — which
  does not auto-step or pace, so **Elixir owns the clock** via a ~50 Hz live loop — sets
  `data.ctrl`, steps, and returns `data.sensordata`. A **native 3D viewer window** renders
  the bot beside the terminal `bb_tui`; the operator teleops the virtual bot through the
  genuine control software.

This **widens the library's public surface** (`BBMCUHub.Sim.Plant` + the sim transport),
makes the transport-is-the-hardware-boundary invariant load-bearing in three places (UART,
test loopback/`VirtualHub`, sim), and pulls MuJoCo into the **dev toolchain only** (the
shipped library/firmware are untouched; the plant is opt-in). It **narrows** the
sim-to-real gap but does not replace BRINGUP Stage 4's silicon truths (motor-phase/encoder
sign, pin map, FOC alignment, real IMU noise) — MuJoCo runs with whatever sign you modeled;
the bench still reveals the actual one. Design-only; not yet implemented. See ADR-0008 for
the seam, the live-loop/clock split from `VirtualHub`, and the open implementation
questions (where the loop lives, the Port wire format, hand-authored vs IR-generated MJCF).

---

## 2026-06-23 — Root-ness is firmware-realized from a generated ROOT_NODE (the -DROOT_HUB flag is gone)

ADR-0006 made root-ness **declared** in the IR — the hub with `parent: :host` is the
root — and the verifier and generator already computed it (`root_hub?/2`, the
`LINK<k>_TRANSPORT_UART` defines). But the FIRMWARE still hand-set `-DROOT_HUB` on the
root env: a second, _unverified_ source of the same truth. Forget the flag on the root
env (or paste it onto a leaf) and the board silently compiles as the wrong role — the
exact "inferred topology" foot-gun ADR-0006 set out to close, surviving in the build
flags after the IR was fixed.

Resolved by deriving root-ness in the chassis from a single generated fact:

- The generator emits `#define ROOT_NODE 0x<NN>` into the robot-scoped
  `wire_contract.h` (alongside the per-link `LINK<k>_TRANSPORT_UART` defines), where
  `NN` is the node id of the declared `parent: :host` hub. It is **always** emitted —
  a single-hub robot still declares `parent: :host`. No root ⇒ a clear comment (the
  topology verifier has already failed).
- The link layer (`link_esp32.cpp`) computes `#define IS_ROOT (MY_NODE == ROOT_NODE)`
  right after including `wire_contract.h`, and every former `#if defined(ROOT_HUB)`
  block now reads `#if IS_ROOT`. Both `MY_NODE` (from `-DMY_NODE=0x02`) and `ROOT_NODE`
  are integer literals, so the comparison folds at preprocess time — the root/leaf
  split still happens entirely in the preprocessor, just from generated truth.
- The example's `blaster_root` env drops `-DROOT_HUB`; `-DMY_NODE=0x02` now _also_
  drives root-ness, so there is one source, not two.

No new ADR — this is the firmware half of ADR-0006's declared-topology decision finally
catching up to the IR, not a new trade-off. It closes the part of ADR-0006's
"Option-1" firmware scope that had deferred root-ness to a hand-set flag. Implemented
and gated by the existing drift test + the two-env `pio run` (the authoritative proof
`IS_ROOT` resolves per env).

---

## 2026-06-23 — A command value-type names its own command message (completes the agnostic Component)

The same ergonomics review found the actuator **Component** contradicted its own
"value-type-agnostic, never hard-coding a struct shape" claim: it resolves and
`unlift`s the command value-type generically, but **hard-coded the PubSub subscribe**
to `BB.Message.Actuator.Command.Effort` — so a consumer's own command (a `Position`,
an `LedColor`) was filtered out _before_ the generic `unlift`, and `Effort` was the
only command that could flow through a BeamBots controller.

Resolved by giving a **command value-type** one more piece of its own contract:

- A new optional `command_message/0` on the **value-type** behaviour, defaulting to
  `nil` (overridable in `use BBMCUHub.ValueType`). A command value-type overrides it
  to return the `BB.Message` command struct it accepts; a sense-only value-type leaves
  it `nil`.
- The actuator view **derives its subscribe `message_types` from the value-type's
  `command_message`** instead of the literal `Effort` — so a consumer's command
  surfaces through the same view. The agnostic-Component claim now holds on the command
  direction, not just the sensor one.
- The **verifier requires** a non-`nil` `command_message` on any value-type placed on a
  `dir: :in` command port — a sense value-type on a command port fails loud (the same
  "the verifier checks roles, not just framing" theme as ADR-0005/0006).

No new ADR — this is a small completion of the value-type seam within the existing
"a value-type owns its full contract" decision, not a separate hard-to-reverse
trade-off. CONTEXT.md's **Value-type** and **Component** terms gain the
`command_message` note. Design locked; implementation lands with the ADR-0005/0006
follow-up (the actuator view, the behaviour default, the verifier check) — not yet done.

---

## 2026-06-23 — Topology is declared by parent links, not inferred from node ids (ADR-0006)

The same ergonomics review found a second cluster of silent foot-guns, all from
**inferring topology**: the root hub was `Enum.min(node_id)` (assign the host-
connected hub a higher id than a leaf → the leaf silently becomes root, host cut
off); the backplane transport was a robot-wide flag collapsed from a misleading
per-hub `transport:` field (on a baked-in "uniform backplane" assumption); and
parent/child structure was unexpressible (single-hop, single-backplane only). The
verifier checked wire framing but not topology.

Resolved by making the tree **declared and a link first-class**:

- **Transport is a property of a _link_** (the edge between a hub and its parent),
  not a hub or a robot — the real hardware truth (host→root is always UART; a hub
  reaches children over a shared CAN bus, point-to-point UART, several UARTs, or a
  mix).
- **Each hub declares `parent:` + `uplink:`**; the root declares `parent: :host`.
  The tree falls out of the parent pointers; links are derived first-class entities.
  A parent may own **any mix** of downlinks (no sibling-uniformity constraint — that
  was an artifact of the wrong per-hub model).
- **The route table maps `node → specific link`** (was `LINK_UP/DOWN/LOCAL`), so
  multi-downlink and multi-hop trees work; routing stays a flat `route_table[node]`
  lookup, only its values change.
- **The verifier checks the tree**: exactly one root, every parent resolves, no
  cycles, fully connected — a topology bug refuses to compile.

Decision recorded in **ADR-0006**; CONTEXT.md gains a **Link** term and the Root hub
entry now keys on `parent: :host`, not lowest id. Design locked; implementation is a
deliberate follow-up (the DSL `parent:`/`uplink:`, the verifier tree checks, the
node→link router in `wire_gen.ex` + the C router, `link_esp32.cpp` per-link
peripherals, the router/relay harnesses) — not yet done.

---

## 2026-06-23 — A safe action is a value-type value; the floor is byte-generic (ADR-0005)

An ergonomics/design review (three parallel lenses against the worked example)
found the highest-severity leak in the **safe-state path**: `safe_action` was a
bare atom that silently mapped to `0.0` for any unknown value, _omitting_ it
silently removed the dead-man floor entirely (a `dir: :in` port with no
`safe_action` became a non-floored direct-drive port, no error), and the C floor
was structurally `float`-only — so a multi-field floored command (servo position +
brake) was unrepresentable yet silently mis-decoded. The verifier, sold as "the bug
cannot ship," checked wire framing but not this role/safety well-formedness.

Resolved by making the safe action **the same kind of thing as the command**:

- **`safe_action` is a literal value of the port's own value-type** (`%{nm: 0.0}`,
  `%{pos: 90.0, brake: true}`) — declared once, so the verifier validates it like any
  value-type value and the generator packs it through the **existing layout codec**
  (the parity-witnessed one — no new translation layer) into C byte initializers.
- **The floor holds opaque bytes**, value-type-agnostic: it watches the `seq` and
  swaps two packed values (commanded ↔ safe), never interpreting them. The scalar
  case is just `N = 4`. This is truer to what the floor always was.
- **The verifier requires `safe_action` on every floored port** and rejects an
  ill-formed one at compile time — the three silent failures become loud or
  impossible.
- **A `dir: :in` port declares its role with a required `has_safe_action` boolean**
  (`true` ⇒ floored, safe*action required, floor generated; `false` ⇒ non-floored
  actuator like the LED, no safe_action). The floored-vs-not distinction is no longer
  \_inferred* from whether `safe_action` is present — so a motor with a forgotten safe
  action is a compile error, not a silently un-floored port. (Named `has_safe_action`
  rather than `floored` because it says exactly what it gates.)

Decision recorded in **ADR-0005**; CONTEXT.md gains a **Safe action** term and the
floor entry notes its value-type-agnosticism. Design locked; implementation is a
deliberate multi-stratum follow-up (floor.h/.c, generator, verifier, the C harness,
the firmware `_drive` hook, the test-only VirtualHub NIF, and the parity/drift
witnesses) — not yet done.

---

## 2026-06-21 — Observability is a separate plane: the observer (§09; ADR-0004)

First real on-hardware use of the architecture surfaced a coupling: the dashboard
(bb_tui) consumed the same PubSub topics at the same rate as the control loop, so
the 100 Hz pose + the controller's command cascade made the TUI sluggish — the
observability cadence was tied to the main loop. Resolved by making observability a
**separate plane** with its own consumer concept.

- **Observer** (a new domain term, host-side, pure reader): samples a
  `(node, port)` **slot** directly from the registry at its **own cadence** (fast or
  slow, its choice) and hands the result to a pluggable **sink**. It does not ride
  the control-plane PubSub. Because the slot is overwrite-only-latest, an observer
  pulling the latest at its own timer is _structurally_ unable to slow the
  producer/control plane or any other observer.
- **Two modes**, matching the two things a slot already is: **sample-state** (level
  — read the latest value, dropping is correct) and **stream-events** (edge — follow
  `seq`, catch every advance, none lost per the §04 Advance invariant). The former is
  for "what is it now?" (a UI monitor), the latter for "what happened?" (every
  command / floor-fired / arm-disarm — an event DB or disk log).
- **Four reduction axes**: sample · select · filter · project. v1 builds
  **sample + select**; filter + project are designed and documented as extensions of
  the same shape.
- **Protecting invariant — an observer is a PURE READER**: never writes, never
  commands, and nothing in the control plane may depend on one. This makes
  observability purely additive and "can't perturb the loop" structural.
- **Not part of the wire contract** — pure host runtime, no firmware/artifact/drift
  impact, freely additive. One job: sample → reduce → hand to a sink (PubSub
  republish / disk log / event DB / UI feed are all sinks). Authored imperatively
  (`BBMCUHub.Observer`); a declarative `observers do` section is later sugar over it.
- **Library owns the observer plane; the example demonstrates adoption** — wiring
  bb_tui onto an observer's slow topic (instead of the broad `[:sensor]` firehose it
  subscribes to) is a consumer use case in the worked example, not a constraint the
  framework's observer design bends around.
- **Why:** the overwrite-only slot was already a rate-decoupling buffer; the gap was
  that the only consumer was a control-plane view whose publish rate every PubSub
  subscriber shared. The observer makes "each consumer owns its cadence, decoupled
  from the loop and from each other" a structural property. Rejected: PubSub-side
  downsampling (still pays delivery + keeps the rate coupling), lowering the control
  publish rate to suit the dashboard (degrades the loop for an observer), observer as
  a Component variant (keeps the coupling), a wire/firmware-level observer (the host
  holds the truth; would muddy the recursive-hub model). CONTEXT.md gains the terms
  Observer + Control plane · observability plane.
- **Deferred to implementation:** this is the design pass (ADR-0004 + §09 +
  CONTEXT.md). The `BBMCUHub.Observer` core (sample-state, sink model) and the
  example's bb_tui-onto-observer use case are the build.
- **Refined after an architecture review** (five-lens, against the real code): the
  original framing claimed both modes were one pure registry-sampling reader — false,
  because the overwrite-only slot loses edges between polls, so **stream-events cannot
  be a poller**. Split honestly: **sample-state** = the pure registry poll (all of
  v1); **stream-events** = a deferred, distinct mechanism that taps the LinkOwner's
  unconditional decode fan-out (lossless, dependency-direction preserved). Five further
  sharpenings folded in: the pure-reader invariant is **enforced structurally** (a
  read-only registry capability, since the registry is `:public` ETS), not just
  documented; `filter`/`project` must **reuse the value-type** (ADR-0003) for field
  access, not duplicate it; a slow observer's freshness ≠ the control plane's, so read
  the **Status slot** for liveness; "can't slow the loop" is backpressure-isolation
  only (a slow sink degrades that observer; fast observers share the scheduler); and an
  observer **resolves its select at start and fails loud** on an unknown slot.

---

## 2026-06-18 — `bb_mcuhub` becomes a reusable library + a `segby_v1` consumer example (§06, §08, §09, §10; ADR-0003)

The system was one Mix app with the example tangled into the library namespace
(`BBMCUHub.Robots.{Follower,SegbyV1}`, `BBMCUHub.Segby.Balance`, `hubs/*`,
`robots/*` compiled into `:bb_mcuhub`) and the firmware glue hand-written per hub.
The design now draws a real consumer boundary, with two load-bearing seams.

- **Two apps, a path dependency.** A publishable **library** (`:bb_mcuhub`, repo
  root) + a separate **example** (`:segby_v1`, `examples/segby_v1/`) depending on
  the library exactly as a downstream consumer would — a Mix `path` dep (host) and
  a PlatformIO `lib_deps` dep on the chassis packaged as a `library.json` library
  (firmware). The example owns its own root namespace `SegbyV1.*` and references
  `BBMCUHub.*` only for library seams.
- **Value-type is the extensibility seam (Option C).** A wire value-type
  (`imu`/`effort`/`status`/…) is no longer a library-internal `@layouts` map entry;
  it is a standalone `use BBMCUHub.ValueType` module owning its layout, host
  `lift`/`unlift`, and firmware-hook signature. A port names its type by module;
  the library ships a lean stock set (imu, effort, status) and a consumer adds
  their own with no library edit. The host views become value-type-agnostic
  (delegate to `type_module.lift/unlift`; the old `BBHub.Lift` dissolves into the
  stock type modules).
- **Firmware per-hub glue is generated; the user writes only device hooks (Shape
  1).** The generator emits the router table, `hub_on_body`, command dispatch, the
  floor init/`on_command`/`control_tick`/status plumbing, the schedule, and
  `hub_tasks` into `<app>/firmware/gen/<slug>/`; the user implements only
  `<hub>_device_setup()` + per-port `<hub>_<port>_read`/`_drive` hooks (signatures
  owned by the value-type) under `<app>/firmware/mcu/`. Clean on-disk split:
  `gen/` is generated + drift-tested, `mcu/` is hand-authored. The emitted-artifact
  count grows from three to four (the per-hub glue header joins the C header,
  parity vectors, and the data-driven — not emitted — Elixir codec); the old
  `hubs/*/mcu/schedule.gen.h` is folded into the glue header.
- **Library is self-testing; Follower retired.** The Follower walking skeleton
  (robot + imu/motor hubs + their artifacts) is removed; a fresh,
  coverage-maximizing **fixture robot** under `test/support/` backs the drift +
  C-parity witnesses (both transports, stamped/unstamped, the actuator floor, and
  a custom value-type), so the library proves the wire _and_ the extension seam in
  isolation. The example adds its own drift test over its own artifacts.
- **Consumer ergonomics.** `@default_robot` defaults (which pointed at the
  now-external Follower) are removed — generation is always explicit-robot;
  `WireGen` takes an explicit output-base so each app generates into its own tree;
  the library ships `mix wire.gen --robot <Mod>`; a generic `BBMCUHub.Host`
  launcher (taking `robot:`, deriving command slots from the IR) absorbs the
  LinkOwner/slot-resolution supervisor a consumer otherwise hand-writes.

- **Why:** the prior tangle could not prove the import boundary, and the
  hand-written firmware glue forced a consumer to re-derive safety-critical
  seq/floor wiring per hub. Making the example a true consumer, the value-type the
  open extension seam, and the firmware glue generated turns "is this library good
  to consume?" into a property CI checks by construction — a broken seam breaks the
  example build. Rejected: a closed value-type map (can't extend without a fork),
  authoring layouts inline in the hub DSL (buries a reusable unit in a
  non-reusable one), C glue via macros (a second source of wiring, the dual-model
  problem ADR-0002 killed). Deferred (SAFeD): a registerable `HubDevice` vtable for
  runtime device-swap / host-mocked hubs (the generated glue does not preclude it).
- **Deferred to implementation:** this is the design pass (ADR-0003 + §06/§08/§09/
  §10 + CONTEXT.md terms Value-type, Firmware hook). The code split, the generator
  extension, the fixture robot, and the example app are the build phases.

---

## 2026-06-18 — Host command drain is event-driven, not polled (§07)

- **Was:** `BBMCUHub.Host.LinkOwner` drained watched command slots on a 5 ms
  `Process.send_after` poll (`@default_drain_ms`).
- **Now:** the drain is **event-driven**. The actuator view
  (`BBMCUHub.BBHub.Actuator`) — the sole writer of its command slot — calls
  `LinkOwner.notify_command_slot(node, port_id)` (a `cast`) after each write; the
  link owner then drains that one slot. The poll/timer is removed entirely.
- **Invariants preserved (§04):** the notification carries only the
  `(node, port)` to look at, never a value, so the link owner still **only reads**
  command slots (it cannot manufacture a `seq` advance), and the `seq`-inequality
  test still dedups (a redundant notification with no new value sends nothing; an
  unwatched slot is a no-op). The notify is best-effort like `disarm/1` — if the
  link owner is unavailable the on-chip floor still backstops.
- **Why:** the 5 ms poll was a v1 stopgap (the design's "revisit with conflation"
  note). Event-driven removes idle wakeups and the up-to-5 ms command latency,
  and is the natural shape now that the single writer is known. The doc (§07)
  already described the drain only as "read-only," not as a poll, so it needed no
  change.

---

## 2026-06-18 — Single-source DSL: contract authored in BeamBots' DSL; boot checks become a compile-time verifier (§06, §09)

The walking skeleton kept **two parallel models** that had to agree: the
producer-side wire facts in `hubs/*/contract.exs` (loaded by `Source`, projected
by `Contract.build_ir/2`) and the reader-side BeamBots `topology do` (the
`sensor`/`actuator` views naming `(hub, port)`). They were reconciled only
implicitly, at view `init/1`, via `PortIndex.resolve/2`. Authoring the same system
in two places is the sync hazard this change removes.

Decision (grilled with the user): **make the BeamBots DSL the single authored
source of truth, and ship the hub gateway as a Spark extension to it.** A user
imports the `bb` packages plus this library and gets new vocabulary to declare
hubs, place them, and wire their ports — focusing on structure/config, while the
library owns the communication logic (wire, floor, freshness, segmentation).

### What changes

- **Hub modules.** A hub is a reusable module (`use BBMCUHub.Hub`, a small Spark
  DSL) declaring its ports' **intrinsic wire facts** — `dir`, `type`, `rate`,
  `t_dev`, `safe_action`, and the pure `sample`/`step` core. Everything true about
  the device, deployment-independent. Read back via `BBMCUHub.Hub.Info`.
- **Placement in a sibling `hubs do` block.** A `hub :name, Module, node: 0xNN`
  entity lives in a top-level `hubs do` section our extension owns, composed onto
  `use BB, extensions: [BBMCUHub.Dsl]` (no `bb` fork). It places hubs on nodes;
  the existing `topology do` wires their ports to components. (The first-choice
  shape — injecting `hub` directly into BeamBots' `topology` section via
  `Spark.Dsl.Patch.AddEntity` — is blocked: `bb` 0.20.3's `topology` section is
  not `patchable?`, so the patch is silently dropped, and making it patchable
  would fork `bb`. A sibling section is the no-fork equivalent.) The view option
  `node:` is renamed `hub:` (it always named the hub, not a wire id; `node` is now
  strictly the placed id).
- **IR is projected, not authored.** A Spark **transformer** in the extension
  walks the `hub` + `sensor`/`actuator` entities, reads producer facts via
  `Hub.Info`, and **persists the IR row shape** into the robot's DSL state, read
  back via `BBMCUHub.Robot.Info`. The IR row shape (the seam `WireGen`/`PortIndex`
  and the parity/drift tests already trust) is **kept unchanged** — only its
  source changes — so the entire C/firmware/parity side stays green.
- **`build_ir`, `Source`, `contract.exs` are dissolved.** There is no file to
  load; the hub modules and the robot module are the source.
- **Boot checks → a compile-time verifier.** §06's checks move from "runtime, run
  before the link opens" to a Spark **verifier** (runs after the transformer, so
  it validates the persisted IR): reader↔producer reconciliation (every named
  `(hub, port)` has exactly one producer), node-id uniqueness + reserved ids (host
  0, broadcast `0x00`) unclaimed, `fresh_for` ≥ one writer period, no
  `(node, port_id)` collision, and the frame-size ceiling (`header + payload +
CRC` ≤ **512 B**, one named constant matching the C `SEG_MAX_BODY`). A violation
  raises `Spark.Error.DslError` naming the offending pair — **the bug cannot
  compile, never mind reach the bus.** This is a strict strengthening of the
  design (earlier = ship-then-refuse-at-boot).

### Why

- One authored model: a fact is stated once, in the DSL, so the producer and
  reader sides cannot fall out of sync (the failure the two-file scheme invited).
- Reuses the BeamBots ecosystem instead of paralleling it: Spark `dsl_patches` +
  verifiers + InfoGenerators are the upstream-blessed extension surface, so the
  library composes with `bb` rather than shadowing it.
- The IR seam is preserved deliberately: it is the proven, drift-tested boundary
  to the C side, which has no business reading the BeamBots DSL. Keeping it means
  the refactor is "swap the front-end that produces the IR," not a wire rewrite.

ADR-0002 records the single-source-of-truth / Spark-extension architecture
(hard to reverse). §06 and §09 of the doc are reconciled in-place to describe the
DSL-authored model and the compile-time verifier; CONTEXT.md gains **Hub module**
and **Topology validation** and sharpens **Contract**.

---

## 2026-06-18 — CAN segmentation/reassembly: on-wire encoding pinned (§03, §06)

`docs/hub-design.html` §03 had already moved frame **segmentation into v1** (out of
SAFeD) but deliberately left the on-wire encoding open ("the bridge segments
it"). Before implementing, the encoding was pinned in a grilling session and the
doc was sharpened **in-place** to describe the now-fixed design. The decisions:

### 1. The end-to-end CRC-16 is present and checked on the CAN path — always

- **Was (code):** single-frame CAN RX handed the raw CAN data field straight up
  as a "verified body," trusting only CAN's own per-frame hardware CRC. (The doc
  always _claimed_ the CRC guards the body across the re-framing boundary; the
  code did not honour it on CAN.)
- **Now:** our CRC-16 rides the body across CAN as its 2-byte trailer and is
  checked at **every** CAN receive, single- and multi-frame alike — the same
  "nothing unverified above the seam" invariant the UART seam already has.
- **Why:** CAN's per-frame CRC guards one bus segment but **cannot survive the
  re-framing** at a branch hub (decode → rebuild in RAM → re-transmit). A bit-flip
  in the hub between RX and TX is exactly what an end-to-end CRC catches and the
  hardware CRC cannot. A corrupted `seq` slipping through would poison freshness
  (§04), so the gate must hold on CAN too.

### 2. Fragment metadata rides the 13 reserved id bits, not the data field

- **Layout pinned:** `[NODE:8][PORT:8][FIRST:1][LAST:1][SEQLO:5][FRAG_IDX:6]`.
  6-bit index → **≤ 64 fragments → 512-byte body ceiling**; FIRST on index 0; LAST
  on the final fragment; the body `seq`'s low 5 bits bind every fragment to its
  body. A single-frame body sets FIRST+LAST, index 0.
- **Why id bits, not a data sub-header:** keeping the data field 100% body bytes
  makes the CAN body **byte-identical to the UART body** for the same logical
  value, so the parity vectors (§06) — the cross-language drift witness — hold
  across both transports unchanged. A data-field sub-header would have forked the
  vectors per transport. The reserved bits were already earmarked for exactly this
  ("deeper-CAN-segment id / priority band"). Fragment bits sit _below_ NODE/PORT,
  so they never disturb the `(NODE,PORT)` hardware filter and never outrank
  arbitration — `NODE 0x00` (e-stop) still wins the bus.

### 3. Reassembly is fail-closed, FIRST-seeded, strictly sequential, no timeout

- One in-flight buffer per `(node,port)`. A buffer is **seeded only by a FIRST
  fragment**; a non-first fragment with no open buffer is dropped+counted
  (`rx_frag_orphan`) — a stray/garbage fragment can never seed a body.
- Each subsequent fragment must have `FRAG_IDX == expected_next` **and** matching
  `SEQLO`; any gap/reorder/alias **abandons the whole partial** (`rx_frag_drop`)
  and only a FIRST may re-seed. Completion is the LAST fragment → CRC over the
  reassembled body → deliver only on pass (else `rx_crc_fail`).
- **No reassembly timeout in v1** — a stalled partial is reclaimed structurally by
  the next FIRST for that key (a timeout is a conflation-era refinement, SAFeD).
- **Why:** losing one fragment loses the **whole body** (no ARQ, no partial). A
  lost body is a stale-making non-event the freshness/born-stale machinery (§04)
  already tolerates; a _partial_ body reaching a slot would be silent corruption.
  Fail-closed is the only safe choice, and the structural reclaim avoids a timer.

### 4. `tx_oversize` re-aimed at the 512-byte ceiling

- **Was:** `tx_oversize` counted any classic-CAN body > 8 B (the refuse-don't-
  truncate stopgap). The 54-byte IMU was on the reject path.
- **Now:** segmentation **is** the > 8 B path; `tx_oversize` is re-aimed at the
  should-never-happen body > 512 B (over 64 fragments) — a belt to the §06 boot
  size-check, not a normal-frame reject. New CAN-seam counters: `rx_frag_orphan`,
  `rx_frag_drop`, `rx_crc_fail`.

### Doc reconciliation (in-place, stateless)

§03's "the one transport detail" callout, its byte diagram (reserved → segment),
and the §06 boot-check bullet were rewritten to describe the pinned encoding as
the current design. `CONTEXT.md` gained a **Segment** term and had **The frame**
and **Contract** sharpened (per-port `t_dev`; three artifacts; CRC always on CAN).
ADR-0001 records the wire-format trade-offs (hard to reverse).

---

## 2026-06-18 — Corrections from building the v1 walking skeleton

Building `bb_mcuhub` (the Elixir host stack, host-compiled + ESP32 firmware, and
the BeamBots views, with a cross-language parity witness) surfaced five places
where the design as written was wrong, underspecified, or made a claim reality
contradicted. All five were folded **in-place** into `docs/hub-design.html` so the
doc still reads as a single stateless source of truth. The changes:

### 1. CAN-FD is a hardware requirement; frame size is a real v1 constraint (§03, §06)

- **Was:** "use CAN FD … v1 sizes every value type to fit one frame, so
  segmentation is SAFeD and the common path never needs it."
- **Now:** CAN FD is stated as a hardware dependency on the MCU + transceiver.
  Frame size is a real constraint: a value type that overruns the chosen
  controller's single-frame size is **segmented by the bridge** (moved out of
  SAFeD into v1), and a bridge **never truncates** — it refuses and counts an
  oversized frame (`tx_oversize`). A per-frame **size check** was added to the §06
  boot checks, kept explicitly distinct from the (still-SAFeD) link-bandwidth
  wire-budget.
- **Why:** ESP32's built-in TWAI is **classic CAN (8-byte data field), not CAN
  FD**. The very first port broke the "fits one frame" promise: reusing the
  canonical `BB.Message.Sensor.Imu` (quaternion + two vectors) is a 40-byte
  payload → 54-byte wire body. That fits CAN FD's 64 but is ~7× over classic
  CAN's 8. Silent truncation would poison decode and the freshness counter (§04),
  so refuse-and-count is the only safe v1 behaviour on a classic-CAN board.

### 2. `t_dev` is opt-in per port (§04, §03)

- **Was:** `t_dev` (8 bytes) is carried on every frame; "costs 8 bytes and zero
  gate complexity."
- **Now:** `t_dev` is a **per-port** contract flag. Sensors that feed
  fusion/replay carry it; command and status ports omit it and stay small.
- **Why:** With a 12-byte header (8 of them `t_dev`), the header is 75%
  timestamp; a 16-byte effort command was half a `t_dev` the design itself says
  commands never use. Making it per-port more than halves command/status frames
  and eases the single-frame budget (item 1). The freshness rule is unaffected —
  `seq` was always the only trust stamp.

### 3. The Elixir codec is data-driven, not generated — "three artifacts" (§06)

- **Was:** the generator emits "four renderings of one model," one of them the
  Elixir codec (`codec.ex`, GENERATED).
- **Now:** the host Elixir codec is **data-driven** (reads the layout/header
  tables at runtime); the generator emits **three** artifacts — the C header, the
  per-hub schedule, and the parity vectors — and drift-tests them.
- **Why:** a codec that interprets the single source of truth cannot drift from it
  _within_ Elixir, so there is no generated `.ex` to fall stale. The drift surface
  collapses to exactly the cross-language boundary the in-language guarantee can't
  reach, which the parity vectors witness directly. Same guarantee, smaller
  surface.

### 4. §09 rewritten against the real `bb` 0.20.3 API

- **Was:** an invented `HubView.Sensor`/`HubView.Actuator` API: a `path:` option,
  `BB.publish/3` by path, manual `BB.subscribe` + `BB.Safety.register` in `init`
  described as "REQUIRED — silent no-op if forgotten," `live?/1` reading the raw
  status slot.
- **Now:** matches the real package — `use BB.Sensor` / `use BB.Actuator`
  **callback modules** (not GenServers) with `options_schema:`; `:bb`
  (`%{robot:, path:}`) and `:motor_profile` **auto-injected**; **no built-in poll
  loop** (the view drives its own beat); `BB.publish(robot, [:sensor | path],
msg)`; the actuator **server auto-subscribes** the command topic and
  auto-registers `disarm/1`, so the "forgotten subscribe" gotcha is gone;
  messages are concrete `BB.Message.Sensor.Imu` / `BB.Message.Actuator.Command.
Effort` with Nx-tensor-backed `Quaternion`/`Vec3`.
- **Why:** the doc's §09 was illustrative pseudo-code written before the real
  dependency was pinned. Coding against `bb` 0.20.3 showed the real shape, which
  is in several ways simpler (no manual subscribe to forget) and in one way
  different that matters (callback module, not GenServer).

### 5. Born-stale resolved strict, with its cost stated (§04, §05)

- **Was:** "born stale … until it personally sees `seq` advance since its own
  boot" — left implicit how a consumer tells a pre-boot leftover value from the
  producer's first post-boot value, which look identical on first observation.
- **Now:** the **strict** rule is explicit: the first observed `seq` is only a
  baseline; trust begins on the first _change_ from it. A leftover value is never
  trusted, at the cost of up to one extra producer period of first-trust latency.
  The monitor code block (§04) and the firmware floor (§05) both reflect this.
- **Why:** it is the literal reading of "advance since its own boot" and the safe
  one. A boot-epoch / generation marker that distinguishes the two cases (and so
  trusts a genuine first value immediately) is noted as a later SAFeD elaboration,
  since it only ever removes latency, never adds trust.

### Implementation bugs fixed alongside (code, not design)

These were defects in an earlier implementation relative to the (correct)
design, fixed in the same pass:

- **`Actuator.live/1` was not freshness-gated** — a stale "not floored" status
  read as `:driving` (the "confident green while floored" failure §05 warns
  against). Now gated by a born-stale status monitor on the view's own beat.
- **Classic-CAN silent truncation** — the TWAI TX path truncated bodies >8 B to 8.
  Now refuses and counts them (item 1).
- **Added a 2-hop router test** — verifies the relay invariant (forward by NODE,
  `seq`/`t_dev`/payload verbatim, meaning-blind), which had no end-to-end test.

### Per-port `t_dev` (item 2) — now implemented

The code was brought in line with the design: `t_dev` is a per-port contract flag
(`t_dev: true`), and the header is one of two shapes. The change rippled through
the Elixir contract/codec/`PortIndex`, the generator (a `wire_port_stamped`
lookup + `PORT_*_STAMPED` defines + `WIRE_HEADER_BASE_SIZE`/`_STAMPED_SIZE`), the
C `frame.c`/`frame.h` (`Frame.stamped`, `frame_decode_body(..., stamped, ...)`),
the parity vectors, and the firmware. Result: the IMU pose stays stamped (52-byte
body); the effort command dropped 16 B → **8 B** (and now fits classic CAN) and
status 15 B → **7 B**. The decoder learns a frame's shape from the per-`(node,
port)` index (host) / `wire_port_stamped` (firmware) after peeking the base
header, so an unstamped frame is never misread. The cross-language parity witness
confirms the new byte layouts agree C↔Elixir.

### Known gaps recorded (not yet built, beyond the design's own SAFeD list)

- **Frame-size check / segmentation (item 1):** the design moves these into v1;
  the code currently refuses-and-counts oversized classic-CAN frames (the safe
  half) but does not yet segment or run the boot-time size check. _(Segmentation +
  reassembly now built — see the 2026-06-18 segmentation entry above. The boot-time
  size check rides with the topology-validation gap below, still open.)_
- No on-hardware run yet; `imu_read`/`drive` are synthetic stand-ins.
- Inbound CAN multi-frame **reassembly** is unbuilt (waits on the segmentation
  work, item 1); undersized stray frames are safely rejected by the codec's
  header-size check. _(Now built — see the 2026-06-18 segmentation entry above.)_
- Boot-time **topology validation** (§06: one producer per `(node,port)`, unique
  ids, `fresh_for` ≥ one period, frame-size check) is specified but not yet
  implemented as a runtime boot check.
- The host command **drain is a 5 ms poll**, not event-driven — fine for v1 rates;
  revisit with conflation (SAFeD). _(Now event-driven — see the 2026-06-18 command-
  drain entry above.)_

---

## (earlier) — initial design

`docs/hub-design.html` v1 authored; supersedes the earlier design draft.
See `CONTEXT.md` for the glossary. Recent git history: design refinement,
hub-gateway v1 design added, hub gateway hardened / earlier draft superseded.
