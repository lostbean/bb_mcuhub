# Context — the hub gateway

A glossary of the load-bearing terms in the hub gateway: the design for reaching
microcontroller hardware from the BeamBots (`bb`) ecosystem through one recursive
abstraction. Definitions only — no implementation details. See `docs/hub-design.html`
for the full architecture.

## Terms

### Hub

The one MCU node type, and the whole topology model. A hub does any subset of three
jobs — **sense** (read a device, `sample` a typed value, publish it up with a `seq`),
**act** (drive a local actuator behind a floor), and **route** (forward frames for
child hubs, meaning-blind). Because a hub can be a parent, a tree of any depth is built
by composing this one shape — there is no separate gateway, leaf, or router type. A hub
with children is a branch; a hub with none is a leaf. See **Root hub**, **Contract**.

### Root hub

The hub that owns the host link: it speaks **UART** upward to the host and bridges down
to its children over their **links** (a shared CAN bus, point-to-point UART, or a mix).
It is the hub that **declares `parent: :host`** — not the lowest-numbered hub, not a
fourth node kind, just the one hub that holds the host connection. It is still an
ordinary hub (it may sense or act while it bridges). Exactly one root per robot, checked
at compile time (ADR-0006).

### Link

The **edge between a hub and its parent** — a first-class topology entity, not a port
(a port is a sense/act endpoint; a link is how a hub is _reached_). A link carries a
**transport** (`:can` or `:uart`) and a peripheral; the host→root link is always UART.
A hub **declares its parent and its uplink transport** (`parent: :blaster, uplink: :can`);
the tree falls out of those parent pointers. Children sharing a parent and a CAN uplink
share **one bus**; a UART uplink is point-to-point — and a parent may own **any mix**
(a CAN bus _and_ several UARTs), because a link is just a typed edge the parent
multiplexes (no sibling-uniformity constraint). The generated **route table maps each
node to the specific link** that reaches it (the host link is link 0), so routing stays
a flat `route_table[node]` lookup — only its values are links, not bare directions. The
verifier checks the tree is well-formed (one root, every parent resolves, no cycles,
fully connected). Nothing about transport or tree shape is inferred from node-id
ordering (ADR-0006).
_Avoid_: conflating a link with a **port** (a child link is never a port) or with a
**NODE** (a node is addressed; a link is traversed).

### Host

The board above the tree (a Raspberry Pi running Elixir/OTP under Nerves and the
BeamBots application). It is **not a hub** — it sits above the hub tree, reaches every
node through one UART to the root hub, and holds the robot's truth in a small
per-`(node, port)` registry. Logical id 0.

### Contract

The data describing what a hub produces on the wire: its ports, each port's value
`type` and `rate` (a single nominal number), its `safe_action`/`t_dev`, plus the pure
core (`sample` for a sensor, `step`/safe-action for an actuator). It is **authored in
the DSL** — the intrinsic facts on a **hub module**, placement and consumption in the
**topology** — not a standalone file; the contract is the producer-side facet that a
**topology validation** transformer projects into the IR. From that one model a
generator emits **three** artifacts — the C headers, the per-hub schedule, and the
parity vectors (the Elixir codec is data-driven, reading the model at runtime) — so the
C and Elixir sides cannot drift. A hub's contract is its public face.

### Hub module

A reusable building block: an Elixir module (`use BBMCUHub.Hub`) that declares one hub's
ports and their **intrinsic wire facts** — `dir`, value `type`, `rate`, `t_dev`,
`safe_action`, and the pure `sample`/`step` core. Everything true about the _device_,
independent of where it is deployed. A user imports a stock hub module, extends it, or
writes their own; the **topology** then _places_ it on a `NODE` and wires its ports to
BeamBots components. The library owns the communication logic (wire, floor, freshness,
segmentation); the hub module owns the device-specific logic. A port names a
**value-type**, not its own layout.

### Value-type

A standalone, reusable unit (`use BBMCUHub.ValueType`) defining _what bytes a kind of
value puts on the wire and how those bytes become a typed `BB.Message`_ — and nothing
else. It carries an ordered `[{field, wire_type}]` **layout** plus a `lift`/`unlift` pair
(raw field-map ↔ `BB.Message`). A **command** value-type also names the one `BB.Message`
command struct it accepts (`command_message`) — so the actuator **Component** subscribes
to _that_ struct, derived from the value-type, never a hard-coded one; a sense-only
value-type leaves it absent. This is part of owning the contract: a value-type knows its
own command message, the view does not guess it. It names no node, pin, rate, or bot, so the **same**
value-type composes across many **hubs** and robots — `:imu` is one contract whether on a
follower's IMU board or segby's Blaster. A **port** references its value-type by module;
the library ships a lean stock set — **imu**, **effort**, and **status** (the universal
pair plus the floor's reported-truth slot, §05) — and a consumer writes their own in their
own project to extend the wire vocabulary, no library edit. The worked example proves this
by defining its own `Range` and `Led` value-types rather than relying on stock ones, so the
extension seam is exercised — and validated — by construction. The IR carries the resolved layout, so the C struct, the Elixir codec, and the
parity bytes all derive from the one declaration. A value-type owns the contract on **both
strata**: the host `lift`/`unlift`, _and_ the **firmware hook** signature for ports of its
shape. It is the single extensibility seam — a new kind of value is one self-contained,
cross-bot-reusable unit. A port's `type:` is **parsed at compile time, not scanned
late**: the IR transformer rejects a `type:` that does not resolve to a real value-type
module (a typo like `:effor`, checked via `ValueType.resolved?/1`) with a named
`DslError`, **before** projection reads the layout — so an unknown type is a clear
compile error naming the `(hub, port)`, not an `UndefinedFunctionError` deep in the
generator.
_Avoid_: layout (that is one _field_ of a value-type, not the unit itself).

### Topology validation (the verifier)

The single compile-time check that the one authored model is well-formed (§06). It is a
**Spark verifier** our DSL extension adds to `use BB` (no fork — hubs are placed in a
sibling `hubs do` section the extension owns), so it runs **at compile time** over the
assembled topology: it reconciles every `(hub, port)` a reader
(a **component** view) names against exactly one producer in the IR, and checks node-id
uniqueness, reserved ids (host 0, broadcast `0x00`) unclaimed, `fresh_for` ≥ one writer
period, no `(node, port_id)` collision, and that every value body (`header + payload +
CRC`) fits the segmentation ceiling (512 B). A violation refuses to compile, naming the
offending pair — the bug cannot ship, never mind reach the bus.

### NODE / PORT (the wire identity)

A value's identity on the wire is `(NODE, PORT)`. **NODE** is a flat, whole-tree-unique
address — never a path; routing is a flat table lookup, `route_table[node] → local link`.
**PORT** names a sense/act endpoint on that node and nothing else — a child link is
**not** a port (a downstream hub is reached by addressing its own NODE). On CAN the two
pack into a generated 29-bit extended id `[NODE:8][PORT:8][rsv:13]`, so the controller
filters in hardware and id-range doubles as arbitration priority. `NODE 0x00` is the
reserved broadcast/e-stop address — the lowest id, so it wins bus arbitration.

### Slot (the registry row)

A named place keyed by `(node, port)` holding exactly one value plus two stamps: `seq`
(a per-write counter the producing hub bumps +1 on every real new value) and `t_dev`
(the producer's own 64-bit monotonic microseconds at the write). Overwrite-only; reads
never block and return the latest. **Exactly one writer per slot — now enforced
structurally** by a slot-scoped **write capability** (`Registry.Writer`, ADR-0007),
the mirror of the read-only `Reader`: a writer is minted for one `(node, port)` and its
`put` carries no node/port (writing another slot is unrepresentable), and a second live
mint for a slot raises `Writer.Taken`. (The table stays `:public` for lock-free
hot-path writes, so the raw-`:ets` bypass is the one remaining, documented hole —
ADR-0007.)

### seq · t_dev (the two stamps)

`seq` is the **only** stamp in the trust path: a consumer judges freshness by "did `seq`
advance within my `fresh_for` window, on my own beats?" — never a cross-board clock
comparison. `t_dev` is a passenger for **same-device** math only (aligning a single
device's samples, jitter, replay); it is never compared across nodes and never read by
the freshness check. The split is strict: `seq` decides trust, `t_dev` is carried but
inert to it.

### Advance (the freshness/floor test)

"`seq` advanced" is the plain inequality `seq != last_seq` — any change is a new write.
This is **sound only because every path is in-order**: point-to-point UART/CAN preserve
order and the relay is a strict FIFO byte pump (see **Relay**). No magnitude test means
counter wrap is a non-issue. If a future relay may reorder, this test must become a
windowed forward compare.

### Relay (the router discipline)

A branch hub forwarding a child's frame copies `seq` and `t_dev` **verbatim** (a relay
never mints a `seq`) **and** forwards in **arrival order** — a strict FIFO byte pump
that never reorders, holds, batches, or dedupes. This in-order guarantee is the
precondition that makes the **advance** test sound. Conflation (deferred) may drop
superseded frames but must preserve per-`(node, port)` order.

### fresh_for · born-stale

Each consumer declares a `fresh_for` window as a multiple of the producer's nominal
period (so "a window shorter than one write" cannot be expressed). A freshly started or
restarted consumer is **born stale**: it distrusts whatever value sits in a slot until
it _personally_ witnesses `seq` advance since its own boot. This is what makes a restart
safe — a rebooted board never trusts a leftover reading.

### The floor (dead-man)

The authoritative safe-state mechanism, on each actuator hub's **own chip**. It watches
the `seq` of _its own_ command against a compiled-in window on its own clock; if the
`seq` stops advancing, it drives the plant to its **safe action** and latches disarmed.
It needs no inbound frame, so it fires even if the parent, the tree above, or the host
is entirely gone. It is the guarantee; everything host-side is best-effort on top of it.
The floor is **value-type-agnostic**: it watches a `seq` and swaps between two values of
the command's own value-type (the commanded one and the safe one), never interpreting
their meaning — it does not know a torque from a position (ADR-0005). It is also
**fail-closed on value width** (defence-in-depth): an over-wide value (`n >
FLOOR_MAX_VALUE`) is never copied into its buffers — `floor_init` clamps to drive
nothing and stays disarmed, and `floor_on_command` ignores the command entirely (no
target, no `seq` update, so it cannot count as an advance), and the dead-man floors on
silence. The compile-time ceilings already bound `n`, so this is a should-never-happen
that resolves to the safe state instead of corrupting the safety chip's own memory.

### Safe action

The value an actuator hub's **floor** drives when disarmed — and it is **a value of that
command port's own value-type** (the same **layout** the command rides on the wire), not
a separate kind of thing. A float-effort port's safe action is a torque-zero value; a
servo's is a neutral position (plus, say, a brake flag); whatever the value-type can
express, a safe action can be. It is **declared once** on the port and is therefore
checked by the same validation as any value-type value (an ill-formed safe action refuses
to compile) and rendered into firmware by the same codec as the wire bytes, so the
on-chip safe state cannot drift from what was declared. A command port states its role
explicitly with a required **`has_safe_action`** boolean: `true` ⇒ floored, a safe action
is required and a floor is generated; `false` ⇒ a non-floored actuator (e.g. an LED), no
safe action. Because the flag is required, a floored port is never silently floorless — a
forgotten safe action is a compile error, not a missing dead-man (ADR-0005).
_Avoid_: treating a safe action as a bare scalar or a magic keyword (it is a typed value),
or inferring "floored" from whether a safe action happens to be present (it is the explicit
`has_safe_action` role).

### Born-disarmed

Every actuator hub boots `armed = false` with its output already at the safe action. It
begins driving only after it witnesses a fresh, in-window command `seq` advancing since
its own boot. A reboot, power glitch, or stale buffered frame cannot energise it —
**motion is continuously earned, never a default**.

### The e-stop (accelerator, not mechanism)

The heartbeat and broadcast disarm share **one** tested code path with the floor: a
broadcast disarm, a missed heartbeat, a pulled wire, or a dead parent all resolve to the
same thing at the actuator — _its command `seq` stops advancing_ → the floor fires. The
e-stop only makes the silence happen faster (and, as `NODE 0x00`, wins CAN arbitration);
it is never a second "react to the stop frame" path that could itself fail.

### A control loop falls silent on disarm

The floor's safe-state is reached by **command-silence** (see _the e-stop_) — which assumes
the **host stops commanding** on disarm. That is true for a dead parent / cut bus / crashed
host, but a **control loop is an always-commanding actor** (a self-balancer _must_ command
every tick to stay upright), so it never goes silent on its own and would re-advance the
floor's `seq` every tick, **defeating disarm**. So a host control loop (a `BB.Controller`
that commands actuators) **must gate its own output on the safety state**: it subscribes to
the safety transitions (`BB.Controller.handle_safety_state_change/2`, the lostbean fork) and
**publishes nothing while not armed** (seeding armed-ness from `BB.Safety.state` at boot so a
born-disarmed robot drives nothing). It is the host-side _producer_ of the silence the floor
waits for — the floor stays the guarantee; the controller supplies the silence. A control
loop that commands through disarm is a safety bug (ADR-0010). Distinct from **the e-stop**:
that accelerates the silence on the wire; this is who, on the host, actually creates it.

### Status slot

A slot an actuator hub produces (`{applied_seq, floored?}` at minimum) flowing _up_ the
wire. It is the authoritative source of "is this hub actually driving?" — read (gated by
the same born-stale check) instead of inferred from "we sent it a command," so the host
never shows a confident green while a wheel sits floored. It is **liveness, not
measurement**: it is consumed _internally_ by the actuator view's freshness check, never
published as a `BB.Message` — so a wheel's _measured speed_ is a separate **wheel-speed
sensor port**, not a status field (ADR-0009).

### Wheel-speed sensor · host velocity loop

A wheel's **measured angular velocity** (rad/s) reported _up_ as a first-class **sensor
port** (`vel_left`/`vel_right`, value-type `WheelSpeed`, `dir: :out`) — surfaced through
the same **component** view + `[:sensor | …]` topic the IMU pose uses, so the host
controller subscribes to it exactly like pose (ADR-0009). The firmware sources it from the
FOC's closed-loop `shaft_velocity`; the sim from MuJoCo's `qvel`. It exists so the host
**balance** loop, which is otherwise open-loop on wheel state (it targets pitch only), can
run an **inner velocity loop**: operator teleop sets a per-wheel _target speed_ and the
controller adds `kv · (target − measured)` to the per-wheel balance torque. The wheel
command stays **torque** (faithful to the real torque-voltage FOC); the velocity loop is a
host control law on top — so "forward" means a _bounded speed_, not an unbounded torque
(acceleration) bias that runs the wheels away. **Turn** is likewise closed-loop: it
commands a _target yaw rate_, and a yaw-rate controller (`kyaw · (target − gyro_z)`, the
IMU's yaw rate already on the wire) drives the differential torque so the measured yaw
tracks it — self-limiting, so a sustained hard turn can't run away (the open-loop
differential-speed turn it replaced eventually toppled the pitch-only balancer). Distinct
from the **status slot**: that is liveness; this is measurement. This lives entirely in the
**example** (`segby_v1`): a consumer value-type + ports on the stock seams (ADR-0003) — the
library is untouched.

### The frame

The on-wire shape: a body — `NODE · PORT · SEQ(2B) · [T_DEV(8B)] · PAYLOAD` — guarded by
a **real, pinned CRC-16/CCITT-FALSE** (check value `0x29B1` over `"123456789"`). `T_DEV`
is **per-port** (present only on stamped ports; see **seq · t_dev**). On UART the body is
`0x00`-delimited and COBS-framed; on CAN it is segmented (see **Segment**). The same body
rides both transports; the root hub re-frames UART↔CAN without touching
NODE/SEQ/T_DEV/PAYLOAD. The CRC covers the whole body and is **always present and verified
on both transports** — on CAN it travels as the body's 2-byte trailer, so a re-framing
bit-flip a hop's hardware CRC cannot reach is still caught. A corrupt frame is counted and
dropped at the seam before any value (or any `seq`) is read.

### Segment (CAN fragmentation)

A body wider than a CAN data field (8 B on the ESP32's classic-CAN TWAI; 64 B on CAN FD)
is **segmented by the bridge** into ordered fragments, one per CAN frame, and reassembled
**before** the CRC check (the CRC is over the whole reassembled body, never per-fragment).
Fragment metadata rides the **13 reserved id bits**, never the data field, so the CAN body
bytes are byte-identical to the UART body and the **parity vectors** hold across both
transports. The layout is `[FIRST:1][LAST:1][SEQLO:5][FRAG_IDX:6]`: a 6-bit index (≤ 64
fragments → a hard **512-byte body ceiling**, asserted by the §06 compile-time size-check), FIRST
on index 0, LAST on the final fragment, and the body `seq`'s low 5 bits binding every
fragment to its body. Reassembly is **fail-closed and strictly sequential**: a buffer is
seeded only by a FIRST fragment; any gap, reorder, `SEQLO` mismatch, or CRC failure
**drops the whole body** (counted, never delivered partial) — a lost body is a
stale-making non-event the **advance** test already tolerates, but a partial body must
never reach a **slot**. A bridge **never truncates**; it refuses-and-counts only the
should-never-happen case of a body over the 512-byte ceiling (`tx_oversize`). No
reassembly timeout in v1 — a stalled partial is reclaimed structurally by the next FIRST
for that `(node, port)` (a timeout is a conflation-era refinement, SAFeD).

### Parity vectors

A generated, committed fixture of `{port, payload, framed_bytes, crc}` rows asserted by
_both_ the Elixir suite and a host-compiled C harness — the cross-language witness that
both codecs agree byte-for-byte. The wire cannot drift past it; hand-editing a row is the
tell.

### LinkOwner

The OTP process (under Nerves, beside the BeamBots tree) that owns the host UART to the
root hub. It decodes inbound frames into `(node, port)` slots and drains outbound
commands to the wire. It is placed to survive a view or law crash, so telemetry keeps
flowing through a fault. **It is a read-only drain of command slots** — never their
writer — so it can never manufacture a `seq` advance. It _is_ the sole writer of the
**inbound** slots, minting a `Registry.Writer` capability (ADR-0007) per inbound slot
on first decode and writing through it. A command value it cannot pack to the wire (a
malformed value-type value) is **counted as `encode_fail` and skipped, never crashing
the drain** — the floor backstops the unsent command, but the cause stays legible (a
counter, not a distant floor firing).

### Virtual robot (the sim transport · plant · driver)

Running a robot's **real host stack with no hardware**, by swapping the one thing that
_is_ the hardware boundary — the **transport** the **LinkOwner** owns. A **sim transport**
(`BBMCUHub.Sim.Transport`, a third `Host.Transport` beside production UART and the test
loopback) plays the whole hub tree against a **plant**, so everything above the transport
— codec, freshness, the floor's meaning, the views, `bb_tui` — runs unchanged and cannot
tell the robot is virtual (ADR-0008). The **plant** (`BBMCUHub.Sim.Plant`, a behaviour)
is the **consumer-supplied dynamics**: given the latest per-slot commands and a `dt`, it
advances a simulated world and returns the sensor values the hubs would have produced —
spoken in **value-type values keyed by wire slot**, not robot structs (the same
byte-/value-generic stance as **safe action**, ADR-0005). A **driver**
(`BBMCUHub.Sim.Driver`) is the engine-agnostic real-time loop: a sibling process that owns
the `~50 Hz` clock, reads the transport's captured commands, calls `Plant.step`, and
injects the returned sensors up as wire bodies (so the sensor views see a fresh **seq**
advance and **born-stale** is honored). The library ships these three engine-agnostic
pieces; a **consumer** writes the plant — segby's is a MuJoCo plant
(`SegbyV1.Sim.MujocoPlant`) over a `Port` to a headless Python child whose native
`launch_passive` viewer renders the bot in 3D beside the terminal `bb_tui` (ADR-0003 split:
library building blocks, example plant). It **narrows the sim-to-real gap** — develop and
de-risk control (balance gains, teleop, the pitch→wheel sign) against faithful dynamics
before the bench — but does **not** replace the bench's silicon truths (motor/encoder sign,
pin map, FOC alignment), since the sim runs with whatever sign you _modeled_. Distinct from
the test **VirtualHub** (the deterministic, frozen-clock, real-C-floor _assertion_ seam):
the virtual robot is the interactive, real-clock, physics-backed _dev_ seam; both share
the philosophy "simulate at the transport" and nothing else.

### Component (the BeamBots view)

A thin `BB.Sensor` / `BB.Actuator` that surfaces a hub's port to BeamBots. A _view_: it
reads/writes slots through the LinkOwner and carries the hub contract in its
`options_schema`. It is **value-type-agnostic** — it lifts to/from a typed `BB.Message` by
delegating to the port's **value-type** (`lift`/`unlift`), never hard-coding a struct
shape, so a consumer's own value-type surfaces through the same view. This holds on
**both** directions: an actuator view subscribes to the command struct the value-type
names (`command_message`), so a consumer's own command value-type flows through the same
view — the view never names a specific command struct, the value-type does. It owns no
socket and names no transport, so it runs unchanged whether the port is on the root hub's
own I²C or a CAN leaf three hops down. A sensor view publishes only when born-stale
freshness passes; an **actuator view is the single writer of its command slot**.

### Observer (the observability plane)

A **host-side** consumer that **samples** a **slot**'s latest value at its **own
independent cadence** for the observability plane (a UI monitor, an event/metrics
database, a disk log) — never on the producer's or control plane's rate. The cadence is
the observer's own choice and may be **high or low**: a UI monitor samples slowly (~10 Hz,
human eyes), while a logger or event database may sample fast to capture full fidelity —
each picks its rate independently, and none affects the control loop or any other observer.
Categorically distinct from a **Component**: a view _pushes_ every beat into the control
plane; an observer _pulls the latest_ at the reader's rate, so it **structurally cannot
slow the control plane** (the overwrite-only slot absorbs the rate gap, and the observer
reads the host registry directly — it never rides the control-plane PubSub). It carries its
own born-stale **monitor** per slot, so `fresh_for` is relative to _its_ beats and it never
reports a leftover value.

An observer has **two modes** — two _different mechanisms_, not one reader:

- **sample-state** (level) — a **pure registry poll**: read the slot's latest _value_ on
  your own timer; dropping the values skipped between polls is correct (the slot is
  overwrite-only-latest). For "what is it now?" — a UI monitor of pose/status/effort.
  **This is the registry-direct pure reader, and it is all of v1.**
- **stream-events** (edge) — must catch **every** advance with none lost, which a poller
  **cannot** do: between polls the slot's `seq` can jump 1→5 and the overwrite-only slot
  has discarded the in-between values (the Advance invariant guarantees the _producer_
  emits every edge, not that a _sampler_ sees them). So stream-events is **not**
  registry-sampling — it taps the **LinkOwner**'s unconditional decode fan-out (the one
  place every decoded advance lands), so no edge is dropped, while the LinkOwner depends
  only on "a fan-out exists," never on "an observer exists." For "what happened?" — every
  command, a floor firing, an arm/disarm; an event database or disk log that must not miss
  one. **Deferred (SAFeD); v1 is sample-state only.** Naming the mechanism keeps the v1
  API from being shaped around a lossless guarantee a poller can't keep.

An observer is a declarative **reduction** of the firehose to just what it needs, along
four optional axes — **sample** (rate/time decimation; state mode only), **select** (which
`(node, port)` slots / which edges), **filter** (a value/event predicate), and **project**
(which fields). v1 implements **sample + select**; **filter** and **project** are named
extensions of the same concept. Because `filter`/`project` act on a value's _fields_ —
which only the **value-type** knows — they resolve the slot's value-type (via `PortIndex` +
`BBMCUHub.ValueType`, exactly as a Component does), never duplicating field knowledge. Many
observers run at once, each on its own flow and cadence.

A slow observer's freshness is **not** the control plane's: `fresh_for` is relative to its
own slower beats, so a 10 Hz observer may report `:fresh` for a slot the 100 Hz loop already
floored. That is correct for "is what I'm showing recent" — but for "is the hub actually
driving," read the authoritative **Status slot** (`floored?`), never an observer's own
freshness verdict.

**The invariant that keeps the concept clean: an observer is a PURE READER — enforced
structurally, not by convention.** It is handed a read-only registry capability (only
`get`/`dump`; `put` not in scope), so "an observer writes a slot" is _unrepresentable_, not
merely forbidden (the registry is `:public` ETS, so the "one writer per slot" rule is
otherwise unenforced). **Nothing in the control plane may depend on an observer existing**
(enforced by the dependency direction in the wiring). This makes adding observability purely
additive — a new observer touches no producer, view, controller, or other observer — and
makes "observers can never perturb the control plane" structural. Note the isolation is from
_backpressure_ (a direct `:ets.lookup` on a read-concurrent table can't block a producer),
not from CPU: many fast observers still share the BEAM scheduler, and a slow **sink** (it
runs in the observer's own process) degrades only _that_ observer, never the loop. An
observer that writes — or that the loop depends on — is a control-plane actor in disguise,
and is forbidden.

An observer is **not part of the wire contract** — it is pure host-side runtime, generates
no firmware and no wire artifacts, and is drift-test-neutral, so it can be added or removed
without regenerating anything (this is what makes it freely additive). Its one job is
**sample + reduce + hand to a sink**: the observer mechanism is type-agnostic; what to _do_
with the sampled value/event lives in a pluggable **sink** (republish to PubSub on the
observer's own topic, append to a disk log, insert into an event database, feed a UI). A
robot starts observers imperatively (`BBMCUHub.Observer`); a declarative `observers do`
section is later sugar over the same core — still host-runtime, never contract.

The observer plane is a **library** concept (general and dep-agnostic — it serves any sink:
a disk log, an event database, a metrics exporter, a custom UI). Wiring a _specific_
dashboard onto it — e.g. pointing `bb_tui` at an observer's slow republish topic instead of
the broad `[:sensor]`/`[:actuator]` firehose it subscribes to by default — is a **consumer
use case**, demonstrated in the worked example, not something the framework's observer
design bends around.
_Avoid_: calling an observer a "view" or a "Component" (the control-plane counterpart);
assuming an observer is always low-frequency (it owns its rate, fast or slow).

### Probe (the diagnostic read)

A **test/diagnostic affordance**, not a running-system component: a single,
flush-robust expression that returns the **authoritative, freshness-gated, decoded**
truth of a `(hub, port)` **slot** — for an actuator hub, its **Status slot**'s
`{applied_seq, floored?}` plus a born-stale freshness verdict — in **one** REPL/console
read. It exists because the on-board IEx console is unreliable for _multi_-statement
scripted reads, yet a fault-injection campaign must witness "did the floor fire?"
reliably; a probe collapses the resolve→read→decode→freshness-gate chain into one
expression so a `seq` that has gone stale never reads as "driving" (the false-green
**Status slot** guards against). A probe is **one-shot and synchronous** — it answers
"what is it right now?" once, on demand, and returns; it owns no timer, no cadence, and
no **sink**. It is **read-only** (the same pure-reader capability an **Observer** holds)
and is **not part of the wire contract** (host-side test tooling, drift-neutral, no
firmware).
_Avoid_: calling a probe an **Observer**. An observer is a _running plane_ concept — a
continuous pure reader sampling at its own cadence into a sink; a probe is a _one-shot
diagnostic_ used by tests and at the bench. (A probe resembles a sample-state observer
whose "sink" is its return value, but the distinction — continuous plane component vs.
on-demand test tool — is the point of the separate term.)

### Control plane · observability plane

Two planes over the same **slots**. The **control plane** is everything that acts on the
robot's truth at control rate — the **Components** (views), the BeamBots controllers/laws,
the command path, the floor. The **observability plane** is everything that _watches_ —
**observers** feeding UI monitors, event/metrics databases, and disk logs. They share the
slots (the overwrite-only registry) but nothing else: an observability-plane consumer
samples at its own chosen rate and can never apply backpressure to or slow the control
plane. Keeping them separate is why a slow dashboard — or a high-frequency logger — neither
perturbs the loop nor is throttled by it; each observer's cadence is decoupled from the
main loop and from every other observer.

### Firmware hook

The thin device-specific seam a user implements on the MCU: a small set of well-known C
functions the **generated** per-hub glue calls — `<hub>_device_setup()` (init pins/
peripherals) plus, per port, a sense/act hook (`<hub>_<port>_read` / `<hub>_<port>_drive`).
The hook _signature_ is owned by the port's **value-type**, not invented per hub, so a
user-defined value-type carries its own firmware-hook shape and is a first-class citizen.
Everything mechanical — router table, `hub_on_body`, command dispatch, the floor init/
`on_command`/`control_tick`/status plumbing, the schedule + `hub_tasks` — is generated
from the IR (never hand-written, so safety-critical seq/floor wiring cannot be miswired
per hub). The generator emits the hook prototypes into a `<hub>.device.h` so the contract
a user owes is legible, resolved at link time. Generating from the IR means a user-defined
hub gets its glue generated identically to a stock one — a device hook is _never_
hand-glued. Two rules the chassis enforces for the hooks: `<hub>_device_setup()` may
run as long as it needs (the task watchdog is armed **after** setup, so a slow FOC
`initFOC`/i2c settle never boot-loops the chip — §08), and every `_read`/`_drive`
tick must be **bounded** (no spin/blocking; a read that can't complete returns
nothing and goes stale, legible).
_Avoid_: hand-written `main_*.cpp` (the pre-generation baseline).

### SAFeD (Safe-by-Default, elaborate later)

The rule for deferred work: a stub must default to the _safe_ behaviour (disarmed, stale,
refused) so elaborating it later only ever _adds_ permission, never removes a guarantee.
Deferred items are named in the text, not hidden. Notable v1 holes left explicit: node
identity is **trust-on-first-use** (a mis-flashed/duplicate board is undetected until the
deferred `fw_id` check), and right-rate enforcement (wire-budget + conflation) is deferred
— v1 permits right-rate but does not yet enforce it. The **firmware hook** is resolved by
well-known link-time names (Shape 1); a registerable `HubDevice` vtable (runtime device
swap, host-mocked hubs) is a deferred extension the generated glue does not preclude.
