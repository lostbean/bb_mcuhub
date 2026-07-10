# segby_v1 — hardware bring-up checklist

`segby_v1` is a two-wheel self-balancing bot ported onto `bb_mcuhub`: a Pi
**host** ↔ UART ↔ **Blaster** root hub (ESP32) ↔ UART backplane ↔ **Wheels** leaf
hub (MKS Dual FOC, dual SimpleFOC). This guide brings it up in independently
verifiable stages — do them in order; fix each before moving on. Everything below
host-builds green and passes the **Stage −1 pre-flight** suite; the stages here
are the half only real hardware can prove.

> Authoritative facts (from the build): backplane is **UART** (no CAN —
> `BACKPLANE_TRANSPORT_UART 1`). Blaster = NODE `0x02` (root), Wheels = NODE `0x05`
> (leaf). All links COBS+CRC. Host↔Blaster runs at **115200** (hardware-proven —
> 1 Mbit/s lost ~95% of frames on this wiring; the firmware pins
> `HOST_UART_BAUD=115200` and the host `transport_opts` baud **must match**);
> the Blaster↔Wheels backplane runs at **1 Mbit/s**. Per-wheel floor window =
> 5 × 20 ms = **100 ms** of command silence → that wheel de-energises
> (born-disarmed).

**Bench kit:** the three boards, 2× USB cables (flashing/monitoring), jumper
wires, a common-ground 12 V supply for the motors, a stand or leash for the
chassis, and — for Stage 2, and as the Stage-1 fallback — a logic analyzer or
scope. Production firmware prints nothing to the USB console by design (the
wire itself is the observable); the one exception is Stage 1's self-test boot
mode, which prints its per-link verdicts there.

## Boards & roles

| Board                                  | Role                                        | NODE          | PlatformIO env                     |
| -------------------------------------- | ------------------------------------------- | ------------- | ---------------------------------- |
| Pi Zero 2 W                            | host (Elixir/Nerves)                        | — (logical 0) | — ([`nerves_host/`](nerves_host/)) |
| DOIT V1 ESP32                          | Blaster — root hub (IMU/range/LED + bridge) | 0x02          | `blaster_root`                     |
| MKS Dual FOC v3.2 (ESP32 Lolin32-Lite) | Wheels — leaf (dual FOC)                    | 0x05          | `wheels_leaf`                      |

## Wiring (3 links + motors)

**1. Host ↔ Blaster UART** (the seam `LinkOwner` owns):

| Pi          | ↔   | Blaster           |
| ----------- | --- | ----------------- |
| GPIO 14 TXD | →   | GPIO 16 (host RX) |
| GPIO 15 RXD | ←   | GPIO 17 (host TX) |
| GND         | —   | GND               |

Pi side: PL011 on `/dev/ttyAMA0` (NOT mini-UART) — `enable_uart=1`, disable the
serial console, `dtoverlay=miniuart-bt` so PL011 lands on 14/15 as `ttyAMA0`.
**On Nerves, do not use `dtoverlay=disable-bt`** — it silently renames the PL011
to `ttyAMA1` (hardware-verified; [`nerves_host/`](nerves_host/) ships a working
`config.txt`). `disable-bt` is the Raspberry-Pi-OS variant only.

**2. Backplane: Blaster ↔ Wheels UART** (cross TX↔RX + common GND):

| Blaster (root)         | ↔   | Wheels (leaf)     |
| ---------------------- | --- | ----------------- |
| GPIO 26 (backplane TX) | →   | GPIO 16 (leaf RX) |
| GPIO 27 (backplane RX) | ←   | GPIO 17 (leaf TX) |
| GND                    | —   | GND               |

(Defaults from `link_esp32.cpp`: root `BACKPLANE_UART_TX_PIN 26 / RX 27`, leaf
`17/16`. No termination — it's a UART, not a CAN bus.)

**3. IMU: MPU-9250 on the Blaster I²C** — SDA 21, SCL 22, VCC 3V3, GND, AD0→GND
(addr 0x68). The firmware reads it for real (see "Real" below); wire it before
Stage 4.

**Motors + encoders (Wheels / MKS board)** — per `firmware/mcu/wheels.cpp`: M0 PWM
32/33/25, M1 PWM 26/27/14, shared enable 12; AS5600 M0 on Wire (SDA 19/SCL 18),
M1 on Wire1 (SDA 23/SCL 5); 12 V to VIN. These values are bench-verified (see
"Real" below) — still **confirm against your own MKS v3.2 silkscreen**, since
board revisions vary.

## Flash

The toolchain is pinned in the repo's `flake.nix` — enter it first with
`nix develop` (or `direnv allow`), then `pio` is on `PATH`:

```sh
cd examples/segby_v1/firmware
pio run -e blaster_root -t upload      # USB to the DOIT V1
pio run -e wheels_leaf  -t upload      # USB to the MKS board
pio device monitor -b 115200           # console — shows only the boot ROM banner;
                                       # the firmware itself prints nothing by design
```

The firmware pulls the C chassis from the library via `lib_deps`
(`symlink://../../../firmware`); the first `pio run` downloads the ESP32 toolchain
into a worktree-local `.pio-core`.

**Host side.** On the bot, flash the Pi with the
[`nerves_host/`](nerves_host/) firmware — it boots
`SegbyV1.Host` on `ttyAMA0` at 115200 by itself. To start the host by hand
instead (e.g. Raspberry Pi OS), it's
`SegbyV1.Host.start_link(transport_opts: [port: "ttyAMA0", baud: 115_200])` —
the baud must match `HOST_UART_BAUD`; the library default is 1 Mbit/s, which
this hardware drops. The dashboard attaches per Stage 5 (an SSH daemon on the
Nerves bot; `BB.TUI.run/2` in the same node on a workstation) — a separate
`mix bb.tui` process is a separate BEAM node and cannot see the robot.

## Stages — verify each before the next

### Stage −1 — pre-flight (run BEFORE flashing anything)

Catch the software-side bugs that destroy hardware, while everything is still
safe (no boards, no motors). From `examples/segby_v1/` inside `nix develop`:

```sh
mix test test/preflight_test.exs        # sign-consistency, born-stale, floor cadence, status
GOLDEN=print mix test test/golden_frames_test.exs   # print the golden wire bytes
```

The **golden frames** are the exact bytes the host puts on the host↔Blaster UART
for known commands (e.g. `motor_left` effort +0.5 Nm = body `05 18 00 01 3F 00 00
00`). Keep that table next to a logic-analyzer / `pio device monitor` capture in
the stages below: if a real frame doesn't match, the bug is in the **wiring or the
firmware build** (endianness, pin map, port id) — not the host. The **pre-flight**
suite pins the host control sign-convention and the freshness/floor-cadence
guarantees; it does NOT certify the absolute pitch→wheel sign (that closes through
motor/encoder wiring at Stage 3/4 — see Stage 4).

### Stage 0 — both ESP32s boot, no reset loop

Flash + monitor each. Expect a clean boot, no panic/reset loop. The Wheels board
runs an AS5600-presence probe before `initFOC` — if an encoder is absent that
motor stays disabled (born-disarmed) but the board still boots and the link comes
up. (A hung `initFOC` = a motor/encoder wiring problem, Stage 3 — but the UART
must still come up.)

### Stage 1 — self-test each board's links (loopback jumper, no cross-wire yet)

Prove each board's own UART paths before trusting any inter-board wiring. The
firmware ships a **link self-test boot mode**: flash a board's `*_selftest` env
and it boots as a loopback tester — it drives every link the board realizes
through the production TX/RX path and prints one PASS/FAIL verdict per link on
the USB console. No host, no robot, no other board:

```sh
pio run -e blaster_selftest -t upload   # or: wheels_selftest for the MKS board
pio device monitor                      # verdicts at 115200
```

The self-test env inherits the production env's build flags by reference, so
the pins/baud/node a PASS certifies are exactly what `blaster_root` /
`wheels_leaf` run — the tested config cannot drift from the robot's. Jumper the
link under test RX↔TX (a jumper displaces the cable — that displacement **is**
the electrical split, so the rest of the robot can stay assembled):

- **Blaster** (root, two links): host UART = GPIO 16 ↔ 17; backplane =
  GPIO 27 ↔ 26.
- **Wheels** (leaf, one link): backplane = GPIO 16 ↔ 17.

The cycle repaints roughly every 2 s — move the jumper and watch FAIL flip to
PASS live, no reflash between attempts. The Blaster with only its backplane
jumpered prints:

```
BB_SELFTEST node=0x02 root=1
link0 host-uart Serial1 rx=16 tx=17 baud=115200: FAIL echoed=0/100 rx_drop=+0   <- jumper 16<->17 or pin dead
link1 backplane Serial2 rx=27 tx=26 baud=1000000: PASS echoed=100/100 rx_drop=+0
```

(The Wheels board prints `BB_SELFTEST node=0x05 root=0` and a single
`link0 backplane Serial2 rx=16 tx=17 baud=1000000` line.)

Reading a verdict at the bench:

- **PASS** — this board's TX→RX path works electrically, at the production
  pins and baud. Done with this link.
- **FAIL with the jumper seated** — the pin or the board is dead, not the
  cable or the peer (the jumper took both out of the loop). The hint on the
  line names this board's actual RX/TX pins: confirm the jumper really sits on
  those two, then suspect the pin map or the silicon.
- **`rx_drop=+N` climbing while `echoed` stays short of the sent count** —
  bytes arrive but frames die in the decoder (CRC/framing): a marginal or
  lossy link, not a dead one. Suspect baud, noise, or grounding.

Test one board at a time and the failure space splits by itself: if both
boards PASS here but frames still don't cross in Stage 2, the fault is the
inter-board wire — a PASS has already cleared each end's TX and RX.

The logic analyzer is the **fallback**, not the first tool: reach for it only
for what the self-test can't see — analog signal quality on a marginal link,
or the cross-wired inter-board run (Stage 2's job).

When every link PASSes, reflash the production envs (`blaster_root` /
`wheels_leaf`) — Stage 2 needs the real firmware back on the boards.

### Stage 2 — cross-wire the backplane, confirm frames cross

Wire link #2. The Blaster relays the Wheels' status frames (NODE 0x05) upward,
and the host (Stage 4) drives commands down. The proof frames cross: a logic
analyzer on the Blaster's host-TX pin (GPIO 17) shows NODE 0x05 bodies riding
up — or skip ahead and let the host be the witness (Stage 4's sensor views only
go fresh if real frames arrive). No frames crossing ⇒ recheck TX↔RX crossed +
common GND (the usual culprit is a missing ground or an unseated Dupont pin).
Diff a captured command frame against the **golden frames**
([`test/golden_frames_test.exs`](test/golden_frames_test.exs), Stage −1):
matching bytes but no motor response ⇒ the wire is fine, look downstream
(Stage 3); mismatched bytes ⇒ a firmware-build/pin issue.

### Stage 3 — motors (Wheels board), 12 V applied

Command a small effort to one wheel at a time and confirm each spins (M0 = left,
M1 = right). Each wheel is gated by its **own floor**: with no fresh command for
100 ms it de-energises. If a wheel won't align, sanity-check that motor + its
AS5600 — the firmware skips a motor whose encoder didn't ACK at boot, so the
symptom is one wheel permanently limp while the other drives (there is no boot
log to read; the skip is silent). Pole pairs are **10**, bench-confirmed for the
stock motors (see "Real" below) — re-confirm only if your motors differ, along
with the PWM/encoder pins vs. the MKS silkscreen.

### Stage 4 — IMU + closed-loop balance

Wire the real MPU-9250 (link #3) — the firmware reads it for real.

**The absolute-sign gate — do this with balance OFF.** The pre-flight suite pins
the host's _internal_ sign convention, but the sign that decides "drive under the
fall vs. amplify it" closes through the motor phase wiring + encoder direction,
which only the bench resolves. Verify it in two safe steps before any closed loop:

```sh
# in IEx on the Pi (ssh segby-v1-<serial>.local), balance DISABLED,
# confirm pose tracks tilt:
BB.subscribe(SegbyV1.Robot, [:sensor, :base_link, :chassis_imu])
# tilt the chassis forward → the published pitch must go one consistent way
# (sign + magnitude track the tilt). A noisy/backwards/zero pitch ⇒ IMU wiring,
# axis, or the complementary-filter input — fix before enabling balance.
```

Then, still **balance OFF and chassis clamped**, command a small effort to each
wheel (Stage 3) and note which way it drives. Mentally check: when the bot tilts
forward, the balance loop will command the sign you saw give "negative torque" in
pre-flight — does that drive the wheels to move _under_ the fall? If not, the fix
is a wiring/encoder inversion (e.g. `encoder.invert`) or swapping the command
sign at the bench — **NOT** guesswork with balance on.

Only once the sign is confirmed: **enable balance**
`SegbyV1.Balance.enable(SegbyV1.Robot)`, **chassis leashed/on a stand**, and
confirm the wheels react to hold upright. PID gains (kp 0.5, ki 0.05, kd 0.1) are
placeholders — **tune on the real chassis**, starting conservative.

### Stage 5 — bb_tui dashboard + teleop

Attach the dashboard. Two real flows (a separate `mix bb.tui` OS process is a
separate BEAM node with no robot in it — it cannot attach):

- **Nerves bot**: the firmware already serves the dashboard as its own SSH
  daemon — `ssh tui@segby-v1-<serial>.local -p 2222` (password `segby`). A
  plain `ssh segby-v1-<serial>.local` (port 22) gives IEx for commands like
  `SegbyV1.Balance.enable/1`.
- **Workstation**: start `SegbyV1.Host` and call `BB.TUI.run/2` in the same
  node.

Confirm the panels populate (joints,
safety, events). Operator drive is the declared **`teleop` command** (forward/turn)
in bb_tui's Commands panel — running it biases the balance output (forward leans
both wheels, turn differentials them). Arm/disarm from the safety panel; recall
the on-chip floor is the real safe-state — disarm/silence both resolve to wheels
de-energising within 100 ms.

**Disarm-while-balancing — verify it, do not assume it.** This is the
one bench check that earlier bring-ups asserted but never ran: with **balance ON
and the chassis leashed/on a stand**, press **disarm**. The wheels must **go limp
within ~100 ms** (`SegbyV1.Balance` stops publishing on disarm → command-silence →
the floor de-energises). The failure mode this guards against is a control loop
that keeps commanding through disarm — it re-arms the floor every tick and the
motors never stop. Confirm the bot actually falls limp (not "keeps holding
upright"), then **re-arm** and confirm balancing resumes. A balancing bot that
ignores disarm is a safety stop, not a tuning note.

## Real (hardware-verified values)

These carry bench- and board-verified values — not placeholders:

- **MPU-9250 IMU**: the real I²C driver (WHO_AM_I → wake PLL → 14-byte burst @
  0x3B), addr 0x68, SDA 21 / SCL 22 @ 400 kHz. Scaled on the MCU to engineering
  units (±2g → ÷16384·g m/s², ±250°/s → ÷131·π/180 rad/s); orientation shipped as
  identity — the **host** fuses pitch via a complementary filter (α=0.98).
- **HC-SR04 range**: real (TRIG 18 pulse → bounded `pulseIn` ECHO 32 → metres).
- **Dual FOC**: pole_pairs **10** (cross-confirmed), Vbus 12 V, driver/motor
  V-limits 6/4, align 8, torque-voltage mode, AS5600 (0x36) M0 on Wire (19/18) /
  M1 on Wire1 (23/5). Driver pins M0 (32,33,25,12) / M1 (26,27,14,12), shared
  enable 12. `phase_resistance` left UNSET (keeps the target in volts).
- **Current sense** (telemetry-only; control stays torque-voltage): INA181A2 ×50,
  0.01 Ω shunt, M0 ADC 39/36, M1 ADC 35/34. Sampled out-of-band, LPF Tf 0.02.

Hardware to confirm at bring-up (verify against your actual board/motors):

- The MKS v3.2 silkscreen vs. the pin map above (a per-channel enable M0 22 / M1 12
  is one documented variant, but this firmware uses a shared enable 12
  for both — double-check on your board).
- `zero_electric_angle` is NOT fixed — `initFOC()` re-aligns each boot (correct).
- **torque→Uq** is an honest first-cut (effort = q-axis voltage,
  clamped to 4 V); a real Nm→V map needs the motor's Kt once measured.

## Still a no-op

- WS2812 `status_led_apply` (the `:led` decode is wired; bind a real LED lib if you
  want the strip to light — it's decorative, not in any control path).

## Safety reminders

- **Born-disarmed**: every wheel boots de-energised and only drives after it
  witnesses a fresh, in-window command `seq` advancing since its own boot. A
  reboot/glitch cannot energise a wheel.
- **Floor is the guarantee**: command silence > 100 ms → that wheel floors.
  Everything host-side (balance, teleop, the dashboard) is best-effort on top.
- Keep the chassis on a stand / leash for the first powered balance test.
