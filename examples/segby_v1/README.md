# segby_v1 — a worked `bb_mcuhub` consumer

A two-wheel self-balancing bot, built as a **separate Mix app that consumes
[`bb_mcuhub`](../..) as a library** — a Mix `path` dependency on the host side, a
PlatformIO `lib_deps` dependency on the firmware side. This is the worked example
for the library/example split ([ADR-0003](../../docs/adr/0003-library-example-split.md)):
it supplies only device-specific logic and reaches the library through its public
seams (`BBMCUHub.Hub`, `BBMCUHub.BBHub.{Sensor,Actuator}`, `BBMCUHub.Host`,
`BBMCUHub.Dsl`, `BBMCUHub.ValueType`). Nothing here lives in the library's
namespace — it owns `SegbyV1.*`.

## What it demonstrates (the consumer experience)

- **Own value-types.** `SegbyV1.ValueTypes.{Range,Led}` (`use BBMCUHub.ValueType`)
  define wire vocabulary the library does _not_ ship, named by module on the hub
  ports — the proof a consumer extends the wire with no library edit.
- **Own hubs.** `SegbyV1.Hubs.{Blaster,Wheels}` (`use BBMCUHub.Hub`) — the Blaster
  root hub (MPU-9250 pose + HC-SR04 range + WS2812 LED, UART backplane) and the
  Wheels leaf (one MKS Dual FOC board driving both wheels behind two on-chip
  floors).
- **A robot.** `SegbyV1.Robot` (`use BB, extensions: [BBMCUHub.Dsl]`) places the
  hubs and wires the views; `SegbyV1.Host` is a thin wrapper over the generic
  `BBMCUHub.Host` launcher.
- **Host control.** `SegbyV1.Balance` (a `BB.Controller`: complementary-filter
  pitch → PID → per-wheel effort) and `SegbyV1.Teleop` (operator drive as a
  declared BB command).
- **Device hooks only.** `firmware/mcu/{blaster,wheels}.cpp` are the _only_
  firmware written here — the real MPU/HC-SR04/dual-FOC/current-sense logic. The
  per-hub router/floor/schedule glue is **generated** into `firmware/gen/segby_v1/`
  from the contract; the C chassis comes from the library via `lib_deps`.

## Build & test

From the repo root, enter the pinned toolchain first: `nix develop` (or
`direnv allow`). Then:

```sh
cd examples/segby_v1

# host
mix deps.get
mix test                      # the host stack over the real COBS+CRC seam, no hardware
mix wire.gen                  # regenerate this robot's artifacts (alias → --robot SegbyV1.Robot)

# firmware (ESP32) — the chassis is pulled FROM the library via lib_deps
cd firmware
pio run -e blaster_root       # the Blaster root hub (sense + UART backplane), NODE 0x02
pio run -e wheels_leaf        # the dual-FOC Wheels leaf, NODE 0x05
pio run -e blaster_root -t upload   # flash
```

After any change to the contract (a hub's ports, a value-type's layout), run
`mix wire.gen` and commit the regenerated artifacts — a drift test fails
otherwise.

## Running on the bot

See [`BRINGUP.md`](BRINGUP.md) for the staged hardware bring-up. In short: start
the host tree with `SegbyV1.Host.start_link(transport_opts: [port: "ttyAMA0"])`,
attach the dashboard with `mix bb.tui --robot SegbyV1.Robot`, and enable balance
live with `SegbyV1.Balance.enable(SegbyV1.Robot)`.

## Running it virtually (no hardware)

Run the whole control stack over a MuJoCo physics model with `mix segby.sim` —
the transport is swapped for the library's sim seam and a viewer window renders
the bot in 3D (ADR-0008). One-time `cd sim && uv sync`, then `mix segby.sim`
(it brings up the robot + sim + viewer and opens the bb_tui dashboard in the same
terminal; arm and run `:teleop` to drive). See [`sim/README.md`](sim/README.md).

## Using this as a template for your own bot

Copy the shape: your own root namespace, your value-types for any non-stock wire
shapes, your hub modules + robot, your device hooks in `firmware/mcu/`, and a
`platformio.ini` whose `lib_deps` points at the `bb_mcuhub` chassis. The host
launcher, the generated glue, the floor, and the freshness machinery are the
library's — you write the bits that are specific to _your_ sensors and actuators.
