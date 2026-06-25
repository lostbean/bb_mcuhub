# segby_v1 — hardware bring-up checklist

`segby_v1` is a two-wheel self-balancing bot ported onto `bb_mcuhub`: a Pi
**host** ↔ UART ↔ **Blaster** root hub (ESP32) ↔ UART backplane ↔ **Wheels** leaf
hub (MKS Dual FOC, dual SimpleFOC). This guide brings it up in independently
verifiable stages — do them in order; fix each before moving on. Everything below
host-builds green and passes the **Stage −1 pre-flight** suite; the stages here
are the half only real hardware can prove.

> Authoritative facts (from the build): backplane is **UART** (no CAN —
> `BACKPLANE_TRANSPORT_UART 1`). Blaster = NODE `0x02` (root), Wheels = NODE `0x05`
> (leaf). All links COBS+CRC @ **1 Mbit/s**. Per-wheel floor window = 5 × 20 ms =
> **100 ms** of command silence → that wheel de-energises (born-disarmed).

## Boards & roles

| Board                                  | Role                                        | NODE          | PlatformIO env   |
| -------------------------------------- | ------------------------------------------- | ------------- | ---------------- |
| Pi Zero 2 W                            | host (Elixir/Nerves)                        | — (logical 0) | — (`mix bb.tui`) |
| DOIT V1 ESP32                          | Blaster — root hub (IMU/range/LED + bridge) | 0x02          | `blaster_root`   |
| MKS Dual FOC v3.2 (ESP32 Lolin32-Lite) | Wheels — leaf (dual FOC)                    | 0x05          | `wheels_leaf`    |

## Wiring (3 links + motors)

**1. Host ↔ Blaster UART** (the seam `LinkOwner` owns):

| Pi          | ↔   | Blaster           |
| ----------- | --- | ----------------- |
| GPIO 14 TXD | →   | GPIO 16 (host RX) |
| GPIO 15 RXD | ←   | GPIO 17 (host TX) |
| GND         | —   | GND               |

Pi side: PL011 on `/dev/ttyAMA0` (NOT mini-UART) — `enable_uart=1`, disable the
serial console, `dtoverlay=disable-bt` (or `miniuart-bt`) so PL011 lands on 14/15.

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
M1 on Wire1 (SDA 23/SCL 5); 12 V to VIN. **Confirm against the MKS v3.2
silkscreen** (pins are TODO-flagged in the firmware).

## Flash

The toolchain is pinned in the repo's `flake.nix` — enter it first with
`nix develop` (or `direnv allow`), then `pio` is on `PATH`:

```sh
cd examples/segby_v1/firmware
pio run -e blaster_root -t upload      # USB to the DOIT V1
pio run -e wheels_leaf  -t upload      # USB to the MKS board
pio device monitor -b 115200           # console (note: console=115200, links=1 Mbit/s)
```

The firmware pulls the C chassis from the library via `lib_deps`
(`symlink://../../../firmware`); the first `pio run` downloads the ESP32 toolchain
into a worktree-local `.pio-core`.

Host: `mix bb.tui --robot SegbyV1.Robot` (the dashboard owns the UART
via `SegbyV1.Host`; on the Pi pass
`transport_opts: [port: "ttyAMA0"]`).

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

### Stage 1 — loopback each board's backplane UART (no cross-wire yet)

Prove each board's own UART2 path before trusting the link. Jumper the board's own
backplane TX↔RX:

- **Blaster**: jumper GPIO 26 ↔ 27.
- **Wheels**: jumper GPIO 17 ↔ 16.
  Every frame the board sends should echo straight back. A `tx` with no matching
  `rx` ⇒ that board's UART path is broken (pin map / silicon), not the inter-board
  wiring.

### Stage 2 — cross-wire the backplane, confirm frames cross

Wire link #2. With both monitors open: the Blaster should relay the Wheels'
status frames (NODE 0x05) upward, and the host (Stage 4) drives commands down. No
frames crossing ⇒ recheck TX↔RX crossed + common GND (the usual culprit is a
missing ground or an unseated Dupont pin). Diff a captured command frame against
the **golden frames** (Stage −1): matching bytes but no motor response ⇒ the wire
is fine, look downstream (Stage 3); mismatched bytes ⇒ a firmware-build/pin issue.

### Stage 3 — motors (Wheels board), 12 V applied

Command a small effort to one wheel at a time and confirm each spins (M0 = left,
M1 = right). Each wheel is gated by its **own floor**: with no fresh command for
100 ms it de-energises. If a wheel won't align, sanity-check that motor + its
AS5600 (the firmware skips a motor whose encoder didn't ACK — check the boot log).
Confirm **pole pairs** (firmware assumes 10 — TODO) and the PWM/encoder pins match
the MKS silkscreen.

### Stage 4 — IMU + closed-loop balance

Wire the real MPU-9250 (link #3) — the firmware reads it for real.

**The absolute-sign gate — do this with balance OFF.** The pre-flight suite pins
the host's _internal_ sign convention, but the sign that decides "drive under the
fall vs. amplify it" closes through the motor phase wiring + encoder direction,
which only the bench resolves. Verify it in two safe steps before any closed loop:

```sh
# on the Pi, with balance DISABLED, confirm pose tracks tilt:
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

`mix bb.tui --robot SegbyV1.Robot`. Confirm the panels populate (joints,
safety, events). Operator drive is the declared **`teleop` command** (forward/turn)
in bb_tui's Commands panel — running it biases the balance output (forward leans
both wheels, turn differentials them). Arm/disarm from the safety panel; recall
the on-chip floor is the real safe-state — disarm/silence both resolve to wheels
de-energising within 100 ms.

**Disarm-while-balancing — verify it, do not assume it (ADR-0010).** This is the
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
