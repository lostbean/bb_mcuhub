# A link self-test is a boot-time build mode; the verdict travels out-of-band on USB

_Builds on ADR-0006 (links are per-hub local indices) and ADR-0011 (the
direction-aware router). Implements the self-test half of issue #8; the
always-on `link_stats` port proposed in the same thread is **rejected** below,
not deferred._

A hub board's links are testable **without the robot**: a dedicated build mode
(`-DBB_SELFTEST=1`), selected as a PlatformIO environment, boots the board into
a loopback tester that exercises each link the board realizes through the
**production** link layer and prints a plain-text pass/fail verdict on
UART0/USB — a channel no link is ever allowed to claim. The mode is static: it
is chosen at flash time, shares no binary with production firmware, and the
production build is byte-identical when the flag is absent.

## The problem

`BRINGUP.md` Stage 1 tells a consumer to jumper a board's backplane TX↔RX and
confirm frames echo — but the shipped chassis has no way to run or observe that
test. The firmware prints nothing, transmits nothing on its own initiative, and
has no diagnostic mode; the transport decoders count their drops
(`TransportDecoder.rx_drop`, the CAN seam counters) but nothing surfaces them.
On the `segby_v1` bench this turned "is this board's Serial2 path dead?" into
an afternoon: arm the whole balancing robot, then infer pin health over SSH
from the _absence_ of a looped-back frame in the host registry (issue #8). A
link you can't self-test is a link you debug by arming a robot.

## The decision

**A contained boot-time branch, split into a pure core and a thin shim, in the
library.**

- **Pure C core** — `firmware/src/selftest.c` + `firmware/include/selftest.h`,
  in the same style as `floor.c`: explicit time in, plain state struct, no
  hardware, host-compiled and tested. One session tests one link: emit
  `n_frames` known frames paced by `gap_ms`, feed every decoded body back in,
  match against expectation, and terminate through a `settle_ms` silence
  timeout so a dead link cannot hang the session. The pinned interface:

  ```c
  typedef struct { uint16_t sent, echoed, mismatched; } SelftestStats;

  void selftest_init(Selftest *s, uint8_t my_node, uint8_t link,
                     uint16_t n_frames, uint32_t gap_ms, uint32_t settle_ms);
  bool selftest_next(Selftest *s, uint32_t now_ms, Frame *out); /* true → send *out on s->link */
  void selftest_on_body(Selftest *s, const uint8_t *body, size_t len);
  bool selftest_done(const Selftest *s, uint32_t now_ms);
  SelftestStats selftest_stats(const Selftest *s);
  ```

  Test frames carry the board's own node, the reserved port `0xFF`
  (`SELFTEST_PORT`), an incrementing `seq`, no `t_dev` stamp, and a
  seq-derived payload pattern. The core deliberately does **not** count CRC
  failures: a corrupt frame dies inside the transport decoder, which already
  counts it; the runner reports that counter's per-session delta instead.
  `0xFF` is reserved by convention, not enforced by the generator — a contract
  that hashes a real port to `0xFF` is harmless, because self-test frames never
  coexist with a running router or floor on the same board, and at a live host
  a mismatched payload fails the value-type decode.

- **ESP32 runner** — `firmware/src/esp32/selftest_esp32.cpp`, guarded
  `#if defined(ARDUINO) && defined(BB_SELFTEST)`, provides `setup()`/`loop()`
  in place of `hub_main.cpp`, whose guard gains `!defined(BB_SELFTEST)`. That
  guard is the **only** touch to production code: with the flag undefined the
  production object code is byte-identical, which is the whole
  zero-production-impact argument, and it is checkable. The runner calls the
  production `link_begin()` — same generated `wire_contract.h`, same `-D`
  pin/baud overrides — so a PASS certifies _this configuration_, not a
  parallel test configuration. It registers its own body callback; the router,
  the floor, and the scheduler never run. It tests each realized link
  sequentially (`IS_ROOT` folds at compile time: root = link 0 host UART +
  link 1 backplane; leaf = link 0 backplane), prints one line per link, then
  repeats forever every ~2 s — move the jumper and watch FAIL flip to PASS
  live, no reflash between attempts. It feeds the WDT exactly as `hub_main`
  does.

- **Pinned output format** — greppable, owned by this ADR; the BRINGUP flow
  and any tooling may rely on it:

  ```
  BB_SELFTEST node=0x02 root=1
  link0 host-uart Serial1 rx=16 tx=17 baud=115200: FAIL echoed=0/100 rx_drop=+0   <- jumper 16<->17 or pin dead
  link1 backplane Serial2 rx=27 tx=26 baud=1000000: PASS echoed=100/100 rx_drop=+0
  ```

- **The reporting channel is an invariant, not a convenience.** A channel must
  never be the sole witness of its own health — so the verdict travels
  out-of-band on UART0/USB, which production firmware never touches (the host
  seam is deliberately Serial1). The rule this ADR pins: **UART0/USB is
  reserved as the out-of-band diagnostic console; no link may ever claim it.**
  The test is entirely board-local — stimulus, reception, and verdict on one
  chip; the host, the wire protocol, and the router are not in the loop, which
  is exactly what makes the verdict trustworthy while any of them is broken.
  Portability: the core returns a stats struct; each MCU shim decides how to
  print it (USB bridge on classic ESP32, native CDC on S3/C3, a debug UART or
  RTT elsewhere).

- **One link-seam addition:** `uint32_t link_rx_drop(uint8_t link)`. It also
  closes an existing blind spot — the root's host-seam decoder counts drops
  that nothing could read before.

- **Env selection = PlatformIO environments, ~4 lines per board**, copied from
  the NEW-ROBOT.md template. Flags are inherited by reference, so the
  self-test env can never drift from the production config it claims to test.
  The `build_src_filter` swaps the device file for a tiny stub whose only role
  is to `#include` a chassis header so PlatformIO's library dependency finder
  links the chassis — no device hooks, no floor, no router, no per-board
  libraries (e.g. SimpleFOC) are compiled. A generator-emitted
  `gen/<robot>/selftest.ini` for `extra_configs` could make the envs free
  later; v1 keeps a documented copy-paste block, simple and honest.

- **CI:** the core and its host-compiled harness (`firmware/test/`) ride the
  existing C-harness CI job. The ESP32 runner rides the same **local**
  `pio run` gate as the rest of the ESP32 stratum — this repo's CI
  deliberately excludes ESP32 toolchain builds (see `.github/workflows/ci.yml`),
  and that standing decision outranks the issue-thread suggestion of building a
  self-test env in CI.

- **UART first; CAN deferred with pinned proof tiers.** The core is
  transport-neutral; the procedure is not. A jumper is a complete
  point-to-point electrical test for UART, which is where the bench pain is.
  CAN cannot be jumpered (a lone transmitter dies on ACK error); the CAN
  runner will use TWAI's self-test mode (`TWAI_MODE_NO_ACK` + self-reception),
  with two proof tiers: no-wiring self-reception proves the controller plus
  the whole `seg_split`/`seg_reasm` path; the same mode through a powered
  transceiver and termination also proves the transceiver (a CAN transceiver
  always hears its own bus bits). Deferred until hardware exists to validate
  it.

- **Live-peer note.** A board in self-test still cross-wired to a live parent
  cannot false-PASS: with the direction-aware router (ADR-0011), its frames
  arrive on the parent's downlink, ascend unconditionally, and die harmlessly
  at the host as an unknown port — they are never reflected back.

## Considered options

- **A runtime-toggled diagnostic mode inside production firmware.** Rejected —
  the most complexity of any option: it adds an input path and mode state to
  safety-adjacent code, and its trigger cannot arrive over the link it exists
  to debug (a leaf's only command path _is_ the uplink under test).
- **A strap-pin boot trigger.** Deferred, not rejected — it layers onto this
  design later as an opt-in `-D` if bench pain demands a one-binary workflow.
- **The always-on `link_stats` port (issue #8's Part B).** **Rejected.** It
  would buy live drop counters for the marginal-but-not-dead link, but the
  price is permanent: a stock value-type, a generated diagnostic port on every
  hub of every robot, contract-hash churn on upgrade, and always-on host-side
  registry/observer surface — entropy in the shipped production path to serve
  a diagnostic need the boot mode already answers. The self-test mode is one
  contained branch that production never executes; the stats port is a
  production feature everyone carries. Accepted cost: a marginal link keeps no
  dedicated live counter (`link_rx_drop()` exposes the number to any future
  surface). Reopening this needs its own ADR with fresh bench evidence.
- **Status quo — infer link health from the full robot.** Rejected: arming the
  entire safety chain plus an SSH session to learn what one printed line says
  in ten seconds.

## Consequences

- BRINGUP Stage 1 becomes runnable by the mechanism the doc already assumed:
  flash the self-test env, open the USB console, read one line. The next
  "a leaf is invisible to the host" splits leaf-TX vs root-RX vs the wire in
  under a minute, one board at a time, with the rest of the robot assembled —
  a jumper inherently displaces the cable; that is the electrical test, not a
  design limitation.
- Any hub can leave the tree for self-test without a global stop: every
  actuator hub floors itself on command silence, so a leaf under test goes
  stale at the host, a root under test lets every leaf below disarm itself
  within its own window. Return path: reflash production; the board boots
  born-disarmed and re-arms on fresh commands.
- Consumer economics: zero cost in the production binary, ~4 env lines per
  board once, one flash cycle in and one out at the bench against the
  afternoon it replaces.
- `link.h` grows one accessor; the wire format, the floor, the seq/dead-man
  contract, and the router are untouched.
- The chassis library ships two more source files; `selftest_esp32.cpp`
  compiles to nothing without the flag, and `selftest.c` is dead code the
  linker drops from production images.
