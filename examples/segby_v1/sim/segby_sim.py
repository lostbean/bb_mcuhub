#!/usr/bin/env python3
"""segby_v1 MuJoCo child — a command-driven stepper with a passive viewer (ADR-0008).

Elixir owns the clock. This child does NOT sleep or self-pace: it blocks on stdin,
steps MuJoCo exactly as many times as the Elixir driver asks, replies ONE JSON line,
and loops. `mujoco.viewer.launch_passive` does not take the loop, does not auto-step,
and does not pace to real time — the driver calls `mj_step` then `viewer.sync()`, and
the window reflects whatever state is current. This is the deliberate opposite of a
`time.sleep`-paced sim loop.

Protocol (JSON-per-line over stdin/stdout, see SegbyV1.Sim.MujocoPlant):

  ready (emitted once at startup):
    {"event":"ready","actuators":[names],"sensors":[names]}

  from Elixir (one per host tick):
    {"op":"set_ctrl_and_step","ctrl":[left, right],"n":N}   # ctrl[0]=left, [1]=right
    {"op":"reset"}
    {"op":"quit"}

  reply (one per command, except quit):
    {"event":"state","time":t,"qpos":[...],"qvel":[...],"sensordata":[...],
     "framequat":[w,x,y,z],"gyro":[gx,gy,gz],"acc":[ax,ay,az]}

macOS requires `mjpython` (GLFW main-thread rule), not `python`. The plant selects
the executable; this script is identical under either.
"""

from __future__ import annotations

import json
import os
import sys

import mujoco
import mujoco.viewer


def _emit(obj: dict) -> None:
    """Write one JSON object as a single line and flush (the Port frames on \\n)."""
    sys.stdout.write(json.dumps(obj))
    sys.stdout.write("\n")
    sys.stdout.flush()


def _sensor(data: mujoco.MjData, name: str) -> list[float]:
    """Read a named sensor's data vector as a plain float list (or [] if absent)."""
    try:
        return data.sensor(name).data.tolist()
    except (KeyError, ValueError):
        return []


def _state(model: mujoco.MjModel, data: mujoco.MjData) -> dict:
    """The full state reply: raw arrays + the three IMU sensors broken out by name."""
    return {
        "event": "state",
        "time": float(data.time),
        "qpos": data.qpos.tolist(),
        "qvel": data.qvel.tolist(),
        "sensordata": data.sensordata.tolist(),
        # The plant reads these three by name; broken out so it needn't know offsets.
        "framequat": _sensor(data, "imu_quat"),  # [w, x, y, z]
        "gyro": _sensor(data, "imu_gyro"),  # angular velocity (rad/s)
        "acc": _sensor(data, "imu_acc"),  # linear acceleration (m/s^2)
    }


def _names(model: mujoco.MjModel, obj_type: int, count: int) -> list[str]:
    return [mujoco.mj_id2name(model, obj_type, i) for i in range(count)]


class _NullViewer:
    """A no-op stand-in for the passive viewer, for HEADLESS runs (no display).

    Lets the command loop stay byte-identical whether or not a viewer window is
    open — `sync()` does nothing, `is_running()` is always true. Used for automated
    balance/regression tests where there is no display (and `launch_passive` would
    segfault) — the physics is identical to a windowed run, only the rendering is
    skipped. Enable with the `--headless` arg or `SEGBY_SIM_HEADLESS=1`.
    """

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        return False

    def sync(self):
        pass

    def is_running(self):
        return True


def main() -> int:
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    flags = {a for a in sys.argv[1:] if a.startswith("--")}
    if not args:
        sys.stderr.write("usage: segby_sim.py [--headless] <path-to-mjcf.xml>\n")
        return 2

    mjcf_path = args[0]
    headless = "--headless" in flags or os.environ.get("SEGBY_SIM_HEADLESS") == "1"

    model = mujoco.MjModel.from_xml_path(mjcf_path)
    data = mujoco.MjData(model)

    # Windowed: the real passive viewer. Headless: a no-op shim (no display, no
    # GLFW) so the same loop runs under automated tests.
    viewer_ctx = _NullViewer() if headless else mujoco.viewer.launch_passive(model, data)

    with viewer_ctx as viewer:
        _emit(
            {
                "event": "ready",
                "headless": headless,
                "actuators": _names(model, mujoco.mjtObj.mjOBJ_ACTUATOR, model.nu),
                "sensors": _names(model, mujoco.mjtObj.mjOBJ_SENSOR, model.nsensor),
            }
        )

        # Block on stdin: one line in -> one reply out (lockstep). No sleep, no pacing.
        for line in sys.stdin:
            if not viewer.is_running():
                break

            line = line.strip()
            if not line:
                continue

            cmd = json.loads(line)
            op = cmd.get("op")

            if op == "set_ctrl_and_step":
                ctrl = cmd["ctrl"]
                n = int(cmd.get("n", 1))
                data.ctrl[: len(ctrl)] = ctrl
                for _ in range(n):
                    mujoco.mj_step(model, data)
                viewer.sync()
                _emit(_state(model, data))

            elif op == "reset":
                mujoco.mj_resetData(model, data)
                viewer.sync()
                _emit(_state(model, data))

            elif op == "quit":
                break

            else:
                _emit({"event": "error", "message": f"unknown op: {op!r}"})

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
