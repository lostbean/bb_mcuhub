# segby_v1 sim — run the robot virtually (ADR-0008)

Run the **whole segby_v1 control stack** against a MuJoCo physics model with **no
hardware**. The transport is the one and only hardware boundary; swap it for the
library's `BBMCUHub.Sim` transport + driver and the real host stack — codec, freshness
monitor, floor semantics, views, `bb_tui` — runs unchanged against a simulated robot,
with a native MuJoCo viewer window rendering the bot in 3D.

This is the **consumer** half of the feature. The library ships the engine-agnostic
building blocks (`BBMCUHub.Sim.Plant` behaviour + `Sim.Transport` + `Sim.Driver`); this
example ships the MuJoCo plant (`SegbyV1.Sim.MujocoPlant`), the MJCF, and the Python
child.

## What's here

| File             | Role                                                                   |
| ---------------- | ---------------------------------------------------------------------- |
| `segby.xml`      | hand-authored MJCF: chassis freejoint + two wheel hinges + IMU sensors |
| `segby_sim.py`   | the command-driven MuJoCo child (passive viewer, JSON-per-line stdin)  |
| `pyproject.toml` | the Python deps (`mujoco>=3.3` + `numpy`) for `uv`                     |

The Elixir plant that drives this child lives at
`../lib/segby_v1/sim/mujoco_plant.ex` (`SegbyV1.Sim.MujocoPlant`).

## Run flow

1. **Enter the devShell** (`nix develop` at the repo root, or direnv) — it provides
   `python312` + `uv` alongside elixir/pio. MuJoCo itself is NOT from nix (it doesn't
   build on Darwin via nixpkgs); `uv` installs it from PyPI into a project-local
   `.venv`.

2. **Install the Python deps once** (needs network):

   ```sh
   cd examples/segby_v1/sim
   uv sync
   ```

   On **macOS**, `mjpython` needs `libpython3.12.dylib` symlinked into `.venv/lib`
   to dlopen it. This is handled automatically — the plant creates the symlink at
   launch (idempotent), and the devShell shellHook also creates it on entry — so no
   manual step is required after `uv sync`.

3. **Launch the virtual robot** — one command, from the example app root:

   ```sh
   cd examples/segby_v1
   mix segby.sim
   ```

   `mix segby.sim` wires the library's `Sim.Transport` + `Sim.Driver` to
   `SegbyV1.Sim.MujocoPlant` and brings up the real `SegbyV1.Host` stack: it injects
   the SIM transport into the host (so the `LinkOwner` owns it), starts the ~50 Hz
   driver that steps the plant and injects sensors back up the stack, opens the
   MuJoCo viewer window, and then launches the **bb_tui dashboard in this same
   terminal** (it blocks until you quit). The MJCF is resolved from `sim/segby.xml`
   relative to the app root, so run the task from `examples/segby_v1/`.

   In the dashboard: **arm** the robot (`a`), then run the `:teleop` command in the
   Commands panel with `forward` / `turn` to drive — the bot you see in the MuJoCo
   window moves. (`:teleop` biases the balance controller's per-wheel effort; it is
   segby's operator-drive seam — `bb_tui` has no built-in teleop.) Quit the dashboard
   (`q`) to stop everything. To enable the balance loop live, start with
   `iex -S mix segby.sim` and call `SegbyV1.Balance.enable(SegbyV1.Robot)`.

> **One node, not two.** The dashboard attaches to the _running_ robot tree over that
> tree's PubSub registry (`SegbyV1.Robot.PubSub`). A separate `mix bb.tui` invocation
> is a different BEAM node with no distribution to this one, so it would not find the
> tree (`unknown registry: SegbyV1.Robot.PubSub`). `mix segby.sim` therefore runs the
> dashboard _in the same node_ that owns the tree — the same-node attach the host
> design assumes. (To attach a dashboard from a separate workstation, start this node
> named and use `mix bb.tui --node ...` — see `BB.TUI`.)

### macOS: `mjpython`, not `python`

The MuJoCo passive viewer needs the main thread for GLFW, so macOS **must** launch the
child with `mjpython` (shipped in the `mujoco` wheel), not `python`. The plant
(`SegbyV1.Sim.MujocoPlant`) selects the executable per platform: `mjpython` on Darwin,
`python` on Linux. Linux runs the viewer fine under plain `python`.

## The JSON protocol (Port wire, JSON-per-line)

The plant talks to the child over a `line:`-framed `Port`, one JSON object per line.
segby's vectors are tiny (a 10-field IMU + a handful of joint values), so the 8 KB line
window is ample.

**Child → Elixir, once at startup:**

```json
{ "event": "ready", "actuators": ["left_wheel_motor", "right_wheel_motor"], "sensors": [...] }
```

**Elixir → child, one per host tick** (the driver's ~50 Hz loop; `n` substeps so one
host tick advances `dt_s` of simulated time at the MJCF's 2 ms timestep):

```json
{ "op": "set_ctrl_and_step", "ctrl": [left, right], "n": 10 }
{ "op": "reset" }
{ "op": "quit" }
```

**Child → Elixir, one reply per command** (except `quit`):

```json
{
  "event": "state",
  "time": 0.02,
  "qpos": [...], "qvel": [...], "sensordata": [...],
  "framequat": [w, x, y, z],
  "gyro": [gx, gy, gz],
  "acc": [ax, ay, az]
}
```

One reply per command gives a natural lockstep/backpressure handshake. The child
**never sleeps or self-paces** — Elixir owns the clock; the child steps only when
driven.

## The ctrl-index convention (load-bearing)

The actuators in `segby.xml` are declared **left first, right second**, and MuJoCo
orders `data.ctrl` by declaration order, so:

| ctrl index | actuator            | driven by segby wire slot |
| ---------- | ------------------- | ------------------------- |
| `ctrl[0]`  | `left_wheel_motor`  | `motor_left` (`:effort`)  |
| `ctrl[1]`  | `right_wheel_motor` | `motor_right` (`:effort`) |

`SegbyV1.Sim.MujocoPlant.step/3` builds `ctrl = [left_effort, right_effort]` from this
convention. If you reorder the actuators in the MJCF, the plant's mapping breaks —
keep left at index 0.

## The sensor mapping

The child reads three IMU sensors **by name** off the `imu` site and breaks them out in
the `state` reply; the plant maps them onto segby's wire slots:

| MJCF sensor (name)        | child field | segby slot / value                               |
| ------------------------- | ----------- | ------------------------------------------------ |
| `imu_quat` (framequat)    | `framequat` | `pose` `:imu` → `qw,qx,qy,qz`                    |
| `imu_gyro` (gyro)         | `gyro`      | `pose` `:imu` → `wx,wy,wz` (angular velocity)    |
| `imu_acc` (accelerometer) | `acc`       | `pose` `:imu` → `ax,ay,az` (linear acceleration) |

`pose` resolves to `PortIndex.resolve(:blaster, :pose)` (NODE 0x02) and is stamped
(`t_dev: true`).

The two wheel statuses are **synthesized by the plant**, not read from a sensor: in sim
the wheel is always _applied_, never _floored_, so each status is
`%{applied_seq: <u16 counter>, floored: false}` on `status_left` / `status_right`
(`PortIndex.resolve(:wheels, :status_left|:status_right)`, NODE 0x05). `applied_seq`
echoes a per-wheel counter that advances each step.

The `jointpos`/`jointvel` wheel sensors ride along in `sensordata` for debugging /
future use; the plant does not currently map them to a wire slot.
