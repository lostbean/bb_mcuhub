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

   On **macOS**, re-entering the devShell after `uv sync` symlinks
   `libpython3.12.dylib` into `.venv/lib` (the shellHook does this) so `mjpython` can
   dlopen it.

3. **Launch the virtual robot** — a `mix segby.sim` task wires the library's
   `Sim.Transport` + `Sim.Driver` to `SegbyV1.Sim.MujocoPlant` and brings up the host.
   _(That launcher is a later chunk; this directory ships the plant, MJCF, child, and
   deps it needs.)_

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
