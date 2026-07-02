# bb_mcuhub (thunderdome)

`bb_mcuhub` reaches microcontroller hardware from the BeamBots (`bb`) Elixir
ecosystem through one recursive abstraction — **the hub**. The repo has two
strata that build and test independently:

- **Elixir host** — a Mix app (`:bb_mcuhub`, `elixir ~> 1.19`) that runs on the
  board above the hub tree, owns the UART to the root hub, and exposes the
  BeamBots seam (host control, codec, scheduler, TUI dashboard).
- **C / ESP32 firmware** — `firmware/` is the C chassis, packaged as a PlatformIO
  library (`library.json`) a consumer pulls via `lib_deps`. Built with the
  `pioarduino` fork of `platform-espressif32` (arduino-esp32 3.x / ESP-IDF 5.x).
  The library itself ships no deployable env; the worked example
  (`examples/segby_v1/firmware/`) has the `blaster_root` + `wheels_leaf` envs.

This repo is a **library + a worked example** (ADR-0003): `bb_mcuhub` at the root
is the reusable library; `examples/segby_v1/` is a separate Mix app that depends
on it as a downstream consumer would. See `CONTEXT.md` for the domain glossary,
`docs/hub-design.html` for the architecture, `docs/adr/` for decisions, and
`design_changelog.md` for history.

## Repository layout

Library (`bb_mcuhub`, repo root — every consumer gets this, never edits it):

- `lib/bb_mcuhub/` — host platform: `wire/`, `contract/`, `value_type/` (+ the
  `BBMCUHub.ValueType` behaviour), `dsl.ex` (the `hubs do` extension), `gen/` (the
  generator), `hub.ex`, `host.ex` (the generic launcher) + `host/`, `bb_hub/` (the
  value-type-agnostic BeamBots seam).
- `firmware/` — the C chassis (`src/`, `include/`, `src/esp32/`) + `library.json`;
  `firmware/test/` host-compiled C harnesses (Makefile, no device); `firmware/gen/robot/`
  the fixture robot's generated artifacts. NO deployable `platformio.ini`.
- `test/` — Elixir tests; `test/support/fixtures/` the coverage-maximizing fixture
  robot that lets the library self-test in isolation. `test/support/virtual_hub.ex`
  - `test/support/c_src/` (a test-only NIF) run the **real firmware C floor + C wire
    path** behind the host stack for deterministic, hardware-free e2e (see below).

Example (`examples/segby_v1/`, app `:segby_v1`, namespace `SegbyV1.*`):

- `lib/segby_v1/` — robot, hubs, own value-types, balance, teleop, host wrapper.
- `firmware/mcu/` hand-authored device hooks; `firmware/gen/segby_v1/` GENERATED
  glue + headers; `firmware/platformio.ini` (`blaster_root`, `wheels_leaf`,
  `lib_deps` the chassis); `test/` its own suite + drift test; `BRINGUP.md`.

Everything under any `gen/` is generated + drift-tested; everything under `mcu/`
is hand-authored.

## Tooling

This repo uses a Nix-native dev setup so every git **worktree** gets the same
toolchain reproducibly. (Note: the legacy `firmware/.pio-venv` / `.pio-core` are
gitignored and the venv launcher is pinned to an absolute main-tree path, so it
does **not** work in a fresh worktree — use the devShell instead.)

- **Dev shell** — run `nix develop` from the repo root to enter a shell with
  Elixir 1.19, Erlang/OTP, PlatformIO 6.1, clang, make, and lefthook. Or let
  direnv load it automatically (`direnv allow` once). Inputs are pinned in
  `flake.lock`.
- **Formatting** — `nix fmt` formats the whole repo via treefmt (mix-format for
  Elixir, nixfmt, prettier for MD/YAML/JSON, shfmt, clang-format for C).
- **Commit gate** — a lefthook `pre-commit` hook formats staged files and
  re-stages them, so commits are always formatted. Install hooks with
  `lefthook install` (available in the devShell). If a commit reformats files,
  it still succeeds — the formatted result is what gets committed.

### Build & test (inside the dev shell)

| Stratum                | Command                                                    |
| ---------------------- | ---------------------------------------------------------- |
| Library (Elixir)       | `mix deps.get && mix test` (or `mix compile`)              |
| Library C test harness | `cd firmware/test && make`                                 |
| Example (Elixir)       | `cd examples/segby_v1 && mix deps.get && mix test`         |
| Example ESP32 firmware | `cd examples/segby_v1/firmware && pio run -e blaster_root` |

`pio run` downloads the ESP32 platform + toolchains into `PLATFORMIO_CORE_DIR`
on first use. The devShell defaults that to a **worktree-local** `.pio-core`
(gitignored) so each worktree keeps its own and never touches another tree's
core. Set `PLATFORMIO_CORE_DIR` yourself to override.

### End-to-end testing with the real C floor (the VirtualHub)

There is a host-side e2e seam that runs the **actual firmware C code** (the floor,
the COBS+CRC wire path) behind the real Elixir host stack — no hardware, fully
deterministic. Prefer it for any host↔hub behaviour (safety, freshness,
communication resilience): it exercises the real safety code, so it can't drift
from the device the way an Elixir mock would.

- `test/support/c_src/vhub_nif.c` — a **test-only**, purely-functional NIF wrapping
  the same `firmware/src/*.c` the C harnesses host-compile (`floor`, `transport`,
  `frame`, `crc16`, `cobs`). Built by `elixir_make` **only in `:test`** (the
  `compilers:` gate in `mix.exs`); the `.so` is gitignored; the shipped library
  needs no C. Loader: `BBMCUHub.Test.VHubNif`.
- `test/support/virtual_hub.ex` — `BBMCUHub.Test.VirtualHub`, a `Host.Transport`
  that plays the ESP32 hub tree: runs a real C floor per actuator port on **explicit
  simulated time** (`tick(vhub, now_ms)` is the only clock — no wall-clock sleeps),
  routes host-bound traffic through the real `FramingCOBS` seam, and injects faults
  (`silence`, `broadcast_disarm`, `reset_floor`, `inject_wire` for corrupt/torn
  bytes). Read back state with `armed?`/`drive`/`status_wire`.
- `test/host/soft_fault_e2e_test.exs` is the worked example to copy from.

When you add resilience/safety/wire coverage, reach for this seam **before** a
unit-level mock or a live-board test — keep assertions count-/value-based and time
explicit so nothing flakes. (Two things it can't reach, by design: the **WDT
boot-loop**, which needs a real RTOS/QEMU or the board, and **motor-sign
calibration**, which is a bench step.)

## Conventions

### Do

- Enter the dev shell (`nix develop` / direnv) before building or testing, in
  any worktree, so the toolchain is reproducible.
- Regenerate firmware/Elixir artifacts via `mix wire.gen` when contracts or
  topology change (a drift test fails the build otherwise).
- Run both strata's checks before committing: `mix test`,
  `cd firmware/test && make`, and `pio run` for the affected env.

### Don't

- **Do not add trailers, attribution, `Co-Authored-By`, or `Generated with`
  footers to commit messages.**
- Do not rely on `firmware/.pio-venv` / `.pio-core` in a worktree — they are
  main-tree-only and gitignored. Use the devShell's `pio` instead.
- **Do not reference ADRs in user-facing documentation** — that is README.md,
  `docs/NEW-ROBOT.md`, `firmware/README.md`, the example's docs, and every
  `@moduledoc` / `@doc` / Spark `doc:` string (anything that lands on hexdocs
  or in `mix help`). State the decision's _content_ in plain prose instead;
  when the pointer matters for maintainers, put the `ADR-NNNN` reference in a
  **code comment** (or a generated-header comment / verifier error string).
  ADR references stay first-class in the design layer only: `CONTEXT.md`,
  `docs/adr/`, `docs/hub-design.html`, `design_changelog.md`.
