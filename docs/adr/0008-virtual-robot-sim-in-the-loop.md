# A robot can run virtually: a sim transport closes the host loop over a physics engine, no hardware

A bb_mcuhub robot can be run **without any hardware** by swapping one thing — the
**transport** the `LinkOwner` owns (§07) — for a `BBMCUHub.Sim` transport that plays
the whole hub tree against a **physics engine**. The sim transport feeds each
outbound actuator command into a **`Plant`** (a behaviour the consumer implements,
or a built-in MuJoCo plant driven over a Port), advances the simulated world on a
**live wall-clock loop**, and injects the resulting sensor readings back up the
**real** host stack as wire bodies — so the real codec, the real freshness monitor,
the real floor semantics, and the real views all run unchanged. `bb_tui` attaches
over BB PubSub exactly as against a real robot, and a **native engine viewer** (the
MuJoCo passive viewer window) renders the bot in 3D in real time. The operator
teleops the virtual bot through the genuine control software.

This is recorded because it adds a **new public seam to the library** (the `Plant`
behaviour + the sim transport), establishes that **the transport is the one and only
hardware boundary** the whole stack pivots on, and commits to an **external,
crash-isolated physics process** (MuJoCo over a Port) rather than embedding a sim in
the BEAM — a decision that is load-bearing for fidelity and hard to walk back once
robots are tuned against it.

## The problem

The library can already run its host stack with no hardware — the test suite proves
it. But every existing hardware-free path is a **dead end for the thing we actually
want**, which is to _develop and de-risk control software against a faithful virtual
robot_ and **shrink the sim-to-real gap** before a board is ever powered.

What exists today, and why each falls short of that goal:

- **The example's `LoopbackTransport`** (`examples/segby_v1/test/support/loopback_transport.ex`)
  round-trips a body through the real `FramingCOBS` and hands it back. It proves the
  _wire path_ is exercised, but there is **no plant**: nothing consumes the actuator
  command, nothing produces a sensor reading. A motor command goes out and _nothing
  moves_. You cannot drive it, watch it, or tune against it.
- **The test `VirtualHub`** (`test/support/virtual_hub.ex`) runs the **real C floor**
  behind the host stack — the strongest hardware-free seam we have — but it is built
  for **deterministic assertion**, not interactive use. It is `:test`-only (the
  `vhub_nif` C NIF is gated to `Mix.env() == :test` in `mix.exs`), and its clock is
  **frozen**: `tick(vhub, now_ms)` is the _only_ time source, advanced by a test that
  owns it. There is **no plant and no real-time loop** — sensors are hand-scripted
  via `emit_sensor/5`, one value at a time, by a test asserting a count. You cannot
  `mix run` it and watch a bot balance.
- **The bench** (BRINGUP.md) is the real thing, but it needs **assembled mechanics**.
  The whole reason this ADR exists is that the chassis is _not yet assembled_, and
  even once it is, you do not want an un-tuned PID swinging a real two-wheeled bot to
  discover the sign is backwards. The bench is where you _confirm_ sim-to-real, not
  where you _develop_ against it.

So the gap is precise: **there is no way to run a real, moving, controllable robot
with faithful dynamics and zero hardware.** Every seam stops short — no plant, or a
frozen clock, or it needs the hardware. And the silent risk in _not_ having this is
the one the whole project is built to avoid: control logic (balance gains, teleop
mixing, the sign that decides "drive under the fall vs. amplify it") gets its first
real test on hardware that can hurt itself, instead of in a sim that crashes for
free.

## The design

One sentence: **the transport is the hardware boundary, so simulate at the
transport.** Everything above the `BBMCUHub.Host.Transport` seam is the real stack;
everything below it — the wire, the hubs, the floors, the silicon, the _physical
plant_ — is what the sim transport stands in for.

### The seam it plugs into (unchanged)

The `LinkOwner` (§07) owns exactly one transport, behind a narrow behaviour
(`lib/bb_mcuhub/host/transport.ex`):

```elixir
@callback start_link(owner :: pid(), opts :: keyword()) :: {:ok, t()} | {:error, term()}
@callback send(t(), body()) :: :ok | {:error, term()}
@callback close(t()) :: :ok
```

- **Outbound** (host → hub): the `LinkOwner` calls `transport.send(t, body)` with an
  already-encoded body (NODE..PAYLOAD, no framing — framing lives below the seam).
- **Inbound** (hub → host): the transport delivers bodies to the owner's mailbox as
  `{:circuits_uart, tag, body}`; the `LinkOwner` `Codec.decode_body/1`s them and
  writes the registry slot (`link_owner.ex:115`).

The production transport is `Circuits.UART`; the test loopback is in-process. **A sim
transport is just a third implementation of this behaviour** — no change to the
`LinkOwner`, the registry, the views, the freshness monitor, the codec, or `bb_tui`.
That invariant — _only the transport knows whether the robot is real_ — is the
foundation this whole feature rests on, and it is already true in the codebase.

### The library/consumer split (ADR-0003)

This decision draws the line exactly where ADR-0003 draws every other: **the library
ships the reusable, engine-agnostic building blocks; the consumer (the example) ships
the robot-specific plant.** The right question is "what is the best library interface
for someone building a bot on bb*mcuhub who wants to run it virtually" — and the answer
is \_a behaviour and a generic driver*, never a baked-in physics engine.

**Library** (`lib/bb_mcuhub/sim/`, reusable, never edited by a consumer):

- `BBMCUHub.Sim.Plant` — the **behaviour** a consumer implements (the dynamics seam).
- `BBMCUHub.Sim.Transport` — the generic sim transport (a `Host.Transport` impl) that
  captures outbound commands per slot.
- `BBMCUHub.Sim.Driver` — the generic real-time loop: owns the timer, reads the
  captured commands, calls `Plant.step`, and injects the returned sensors up as wire
  bodies via `Codec.encode_body`. **Robot-agnostic** — it is handed the plant module
  and the slot/type map; it knows nothing about segby, MuJoCo, or wheels.

**Consumer** (`examples/segby_v1/`, the worked example, a downstream like any other):

- `SegbyV1.Sim.MujocoPlant` — the example's `Plant` implementation: owns the `Port` to
  the Python child, maps segby's `:effort` slots to `data.ctrl` and `data.sensordata`
  back to the `:imu`/`:status` values.
- `examples/segby_v1/sim/segby.xml` — the hand-authored MJCF.
- the Python child script + a `mix segby.sim` launcher.

The library has **no MuJoCo dependency and no Python** — a consumer who wants a pure-Elixir
toy plant, a different engine, or a recorded-trace plant just writes a different `Plant`.
This mirrors ADR-0005 (the plant speaks **value-type values keyed by wire slot**, not
robot structs) and keeps the library's promise: a consumer gets "run my robot virtually"
from the behaviour alone.

The behaviour:

```elixir
@callback init(opts :: keyword()) :: {:ok, state}
# Apply the latest per-actuator commands, advance physics by dt seconds,
# and return the sensor readings the hubs would have produced.
@callback step(commands :: %{ {node, port_id} => value }, dt_s :: float, state) ::
            {sensors :: [{node, port_id, type, value}], state}
@callback close(state) :: :ok
```

### The live loop and the clock

Unlike the test `VirtualHub`'s frozen clock, an interactive sim runs on a **real-time
wall-clock loop**, because a human is watching and driving it. The loop lives in a
**sibling `BBMCUHub.Sim.Driver`** that the sim transport supervises — _not_ inside the
transport GenServer. This keeps `Sim.Transport.send/2` a **pure command-capture** (no
timer, no plant call in the `LinkOwner`'s call path) and makes the loop+plant
**independently restartable**: a plant crash restarts the `Driver` (and its `Port`)
without tearing down the transport or the `LinkOwner`'s link. The `Driver` runs a
`~50 Hz` tick:

```
every ~20 ms (Process.send_after):
  cmds    = Sim.Transport.take_commands(transport)    # newest captured per slot
  sensors = Plant.step(cmds, dt, plant_state)         # advance physics
  for {node, port_id, type, value} <- sensors:
     body = Codec.encode_body(node, port_id, seq, t_dev, type, value, stamped?)
     send(owner, {:circuits_uart, :sim, body})        # inject up the real stack
```

`send/2` is **event-driven and clock-free** (confirmed: the production transport has
no internal timer), so the transport just **captures the newest command per slot** on
each `send/2` and the `Driver` reads them each tick. This is the one real departure
from the test seam — a test owns the clock; a live demo lets the `Driver` own it — and
it is why this is a _transport + driver_, not a reuse of `VirtualHub`.

#### Disarm in the sim: the silence comes from the host, not the transport

The sim correctly de-energises on disarm **without any sim-specific e-stop wiring** —
because disarm is realised the same way it is on hardware: the host **control loop
falls silent**, and the plant maps "no command for this slot" to that slot's safe
value (segby: 0 torque → the wheels go limp). When the robot disarms, `SegbyV1.Balance`
stops publishing (ADR-0010), so no command reaches `Sim.Transport`, so `take_commands`
reports nothing for the wheel slots, so the plant drives the safe value. **The sim
transport needs no special disarm handling** — it just keeps capturing whatever
commands arrive, and disarm means none arrive. (This mirrors the real floor, which
fires on command-silence; the broadcast `NODE 0x00` frame `LinkOwner.send_disarm`
emits is the wire accelerator, but the actual safe-state — sim and hardware alike —
comes from the controller going quiet.) See ADR-0010 for why the controller must be
the one to fall silent.

Determinism is preserved where it matters: MuJoCo's `mj_step` reads no wall clock, so
the _cadence_ only sets how fast the human sees motion, never the simulated result.
A future deterministic test mode can drive the same plant on an explicit step count.

### The sensor-injection recipe (the real stack does the rest)

Injecting a sensor reading is exactly building a wire body and handing it to the owner
— the same body `decode_body/1` expects, no framing needed for in-process delivery
(the example's loopback confirms this). For the segby IMU `pose` port:

- Resolve the slot: `PortIndex.resolve(:blaster, :pose)` → `{0x02, port_id}`.
- The `:imu` value is a 10-field f32 map mirroring `BB.Message.Sensor.Imu`
  (`value_type/imu.ex`): `%{qw,qx,qy,qz, wx,wy,wz, ax,ay,az}`.
- `pose` is **stamped** (`t_dev: true` in the hub) — so inject with `stamped?: true`
  and a real device timestamp; the freshness monitor depends on it.
- `body = Codec.encode_body(0x02, port_id, seq, t_dev, :imu, value, true)` then
  `send(owner, {:circuits_uart, :sim, body})`.

From there the **real** path lights up untouched: `LinkOwner` decodes → writes the
registry slot → the sensor view witnesses a fresh `seq` advance (born-stale honored) →
`lift/1` produces `BB.Message.Sensor.Imu` → publishes `[:sensor | path]` → `bb_tui`
and `SegbyV1.Balance` both receive it. The actuator path is the mirror: `Balance`
publishes an `Effort` → actuator view writes its slot → `LinkOwner` drains → `send/2`
→ the sim transport captures it for the next `Plant.step`.

### The MuJoCo plant (in the example): viewer + external stepping

The MuJoCo plant is the **example's** `Plant` implementation (`SegbyV1.Sim.MujocoPlant`),
not a library module — the library stays engine-agnostic (above). It owns a `Port` to a
headless Python child and translates between segby's wire slots and MuJoCo's arrays.

MuJoCo's **`launch_passive`** mode is purpose-built for this: it does **not** take the
loop, does **not** auto-step, and does **not** pace to real time — the external driver
calls `mj_step` then `viewer.sync()`, and the window reflects whatever state is current.
The Python child is small and command-driven over the Port, framed as **JSON-per-line**
(`Port.open(..., [:binary, line: 8192])`; the child `print`s one JSON object per line and
flushes):

```
ready → loop: read {set_ctrl, n} from stdin → data.ctrl[:]=… → mj_step ×n →
        viewer.sync() → write {qpos, sensordata, gyro, acc} to stdout
```

Load-bearing constraints, captured so the implementation does not rediscover them:

- **macOS requires `mjpython`**, not `python` (GLFW main-thread rule). The Port's
  executable is platform-selected by the plant.
- **No `time.sleep` in the child** — pacing lives in the Elixir `Driver`; the child runs
  as fast as it is driven.
- The viewer renders on its **own background thread**; the main thread does
  stdin→step→sync. One `state` reply per `step` gives a natural lockstep/backpressure
  handshake.
- **JSON-per-line caps line length** (the Port's `line:` window). segby's vectors are
  tiny (10-field IMU + a handful of joint values), so an 8 KB line is ample. A consumer
  with a large model would raise the cap or move to length-prefixed framing in its own
  plant — the library does not constrain this, since the wire format lives entirely in
  the consumer's `Plant`/child.
- A minimal MJCF (chassis freejoint + two wheel hinges + velocity actuators + a
  `site` carrying `gyro`+`accelerometer` sensors + a ground plane) is the segby model;
  the IMU `site` sensors map to the `:imu` value, wheel velocity actuators to the two
  `:effort` slots. **Hand-authored** at `examples/segby_v1/sim/segby.xml` (not generated
  from the IR — generation is a deferred, separable chunk; see below).

### What it is, and is not

- **It is** a real-control-stack-against-faithful-dynamics rig: every box except the
  plant is shipped code. It is where balance gains, teleop mixing, and the
  pitch→wheel _sign_ get developed and de-risked before the bench.
- **It is not** a replacement for the bench's silicon truths (motor-phase/encoder
  sign, pin map, FOC alignment, real IMU noise/axis, the WDT boot-loop). MuJoCo runs
  with whatever sign you _modeled_; the bench is still where the _actual_ sign reveals
  itself. This **narrows** the sim-to-real gap; BRINGUP Stage 4 still closes it.
- **It is not** the test `VirtualHub`. That stays as the deterministic, C-floor,
  frozen-clock assertion seam. This is the interactive, real-clock, physics-backed
  demo/dev seam. They share the philosophy (simulate at the transport) and nothing
  else.

## Consequences

- **New library surface — three engine-agnostic modules.** `BBMCUHub.Sim.Plant`
  (behaviour), `BBMCUHub.Sim.Transport`, and `BBMCUHub.Sim.Driver` become public API —
  every consumer gets "run my robot virtually" for free by implementing a `Plant`. The
  library ships **no** physics engine and **no** Python; the MuJoCo plant lives in the
  example as a consumer (`SegbyV1.Sim.MujocoPlant`), per ADR-0003. This widening of the
  library's contract is documented + drift-stable like the rest.
- **The Python/MuJoCo dependency enters the EXAMPLE's toolchain, not the library's.**
  The example's sim is opt-in; its `mix segby.sim` + Python child + MJCF are a
  dev/sim concern of the consumer. The shipped library, firmware, and host runtime are
  untouched. (The devShell/flake gains `mujoco` + `mjpython` for running the example.)
- **The transport-is-the-boundary invariant is now load-bearing in three places**
  (UART, test loopback/VirtualHub, sim). Any future change to the `Transport`
  behaviour must keep all three honest.
- **`Sim.Transport` accepts an optional `:name`.** The `LinkOwner` starts the
  transport (`transport.start_link(self(), transport_opts)`) and holds its pid
  privately, so a sibling `Sim.Driver` cannot reach it by pid. Naming the transport
  (`transport_opts: [name: ...]`) lets the driver locate it to `take_commands/1` — the
  seam that wires the generic library pieces together in a consumer's host tree.
- **Reproducibility is a property of the loop, not the bridge.** A later
  deterministic test mode can reuse the exact `Plant` on an explicit step clock — the
  interactive loop and a frozen-clock test differ only in who calls `step`.

## Resolved (decided 2026-06-25, during build kickoff)

- **The live loop lives in a sibling `BBMCUHub.Sim.Driver`** the transport supervises —
  `send/2` stays a pure capture, the loop+plant are independently restartable.
- **The Port wire format is JSON-per-line** (`line: 8192`). segby's vectors are tiny;
  the format lives in the consumer's plant, so a larger model can choose otherwise
  without touching the library.
- **The MJCF is hand-authored** at `examples/segby_v1/sim/segby.xml`. IR-generated MJCF
  is attractive (drift-tested) but a separable, deferred chunk.
- **The lib/example split** (ADR-0003): library ships `Sim.Plant` + `Sim.Transport` +
  `Sim.Driver` (engine-agnostic); the example ships `SegbyV1.Sim.MujocoPlant` + the MJCF
  - the Python child + `mix segby.sim`.

## Still open (deferred, not blocking this build)

- **Whether the live plant ever needs the real C floor.** The interactive demo can run
  the floor in pure Elixir (or rely on host-side safety) since the test `VirtualHub`
  already proves the C floor; promoting `vhub_nif` out of `:test` is a separable
  decision, deferred.
- **IR-generated MJCF** — a future generator chunk, drift-tested like the rest of `gen/`.
