# Harvester Dashboard (canonical telemetry v1, view-only)

Qt Quick operator dashboard for the canonical ZeroMQ telemetry bus.  It is
source-agnostic: the same binary renders the Xavier simulation gateway, an
audit replay, or the future Orin hardware aggregator.  It contains **zero
ROS imports** and runs under the system `/usr/bin/python3`.

## Safety semantics

- **View-only.** The only sockets the dashboard creates are:
  1. one SUB socket to `--pub` (read),
  2. an optional REQ to `--status` (read-only query; disabled with `--status ''`),
  3. an optional annotation forward PUB (`--annotation-pub`, **disabled by
     default**; suggested `tcp://127.0.0.1:5592`).
- Keys `1`/`2` (cutter/docking) are **render-only**: both subscriptions stay
  live and no traffic is emitted.  This is enforced by a wire-level test
  (`test_no_emit_proof.py`).
- Maintenance stream controls exist in the UI but are hidden unless the
  status REP reports a hardware source; in simulation there is no control
  endpoint at all and none is contacted.
- Annotations are camera-relative only in Phase 1; `tree_base_xyz` stays
  `null` and the UI never claims a world-fixed target.

## Environment

Ubuntu 20.04 arm64 (Xavier), PySide2 5.14 (no QtQuickControls2 — QML uses
QtQuick 2 primitives only), python3-zmq, python3-msgpack, numpy, Pillow.

```bash
sudo apt install python3-pyside2.qtquick python3-zmq python3-msgpack \
     qml-module-qtquick2 qml-module-qtquick-window2 qml-module-qtquick-layouts
```

## Precaution: two different Python interpreters

This workspace deliberately uses **two different `python3` interpreters**.
Never mix them — each role is pinned to the interpreter that has (and lacks)
exactly the right modules:

| Role | Interpreter | Why |
|---|---|---|
| ROS 2 / Gazebo nodes | active `python3` = anaconda 3.8.8 (`~/anaconda3/bin/python3`) | Has ROS 2 Foxy, `rclpy`, MessagePack, ZeroMQ, numpy, PIL. **No PySide2.** |
| Canonical ZeroMQ gateway + replay | active `python3` = anaconda (run inside a ROS-sourced terminal) | Needs `zmq` + `msgpack` + `harvester_telemetry_contract`, which only exist under anaconda. |
| **This dashboard** | **system `/usr/bin/python3` 3.8.10** | Has apt PySide2 5.14 with **QtQuick** bindings. No ROS, and its `zmq`/`msgpack` come from apt. |

Rules of thumb:

- **Gateway / replay terminals**: `source /opt/ros/foxy/setup.bash` and
  `source ~/ros2_ws/install/setup.bash`, then plain `python3`.
- **Dashboard terminal**: do **not** source ROS.  Invoke
  `/usr/bin/python3` explicitly and set `PYTHONPATH=src/harvester_dashboard`
  so neither interpreter's site-packages shadow the other.
- The gateway launch file intentionally uses the active `python3`
  (anaconda) — generated ament entry points are pinned to `/usr/bin/python3`,
  which lacks MessagePack/ZeroMQ.  Do not "fix" this.
- All three processes talk only via canonical ZeroMQ on localhost
  (`5590` live PUB, `5591` replay PUB, `5600` status REP), so the interpreter
  split is invisible on the wire.

## Run A: with live Gazebo + RViz simulation (canonical ZeroMQ live)

Three terminals, in this order.

**Terminal 1 — Gazebo + RViz** (already your normal workflow; unchanged):

```bash
cd ~/ros2_ws
source /opt/ros/foxy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch oil_palm_harvester_description gazebo_harvester_and_tree.launch.py \
  harvester_collision_mode:=off articulation_control_mode:=kinematic
```

**Terminal 2 — canonical telemetry gateway** (ROS-sourced terminal, anaconda
python; binds `tcp://*:5590` PUB + `tcp://*:5600` status REP):

```bash
cd ~/ros2_ws
source /opt/ros/foxy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch harvester_telemetry_gateway gateway.launch.py
```

The gateway is read-only toward ROS 2: it subscribes to existing sensor
topics only and never publishes commands, so sliders/RViz/Gazebo behave
exactly as before.  Keep this process running; it is the live source.

**Terminal 3 — dashboard** (system python, no ROS sourcing):

```bash
cd ~/ros2_ws
DISPLAY=:1 PYTHONPATH=src/harvester_dashboard \
  /usr/bin/python3 -m harvester_dashboard.main \
  --pub tcp://127.0.0.1:5590 \
  --status tcp://127.0.0.1:5600
```

The "Harvester Telemetry Dashboard" window opens on `DISPLAY=:1` beside
RViz.  You should see within ~2 s: live cutter camera, green stream rows,
SIMULATION badge (from `--status`), docking/cutter ranges, trunk estimate,
calibration line, and the LiDAR inset.  Move a slider in RViz/Gazebo and the
camera/trunk/ranges react — nothing is fed back into the simulation.

## Run B: replay of a recorded session (no Gazebo/ROS needed)

Uses an existing audit directory (e.g. `~/harvester_audits/run_001`).  Great
for UI work, regression checks, and demos without touching the simulation.

**Terminal 1 — replay publisher** (anaconda python; it may be run without
sourcing ROS because the gateway modules are pure ZeroMQ — just set
`PYTHONPATH` to the two telemetry source trees).  It binds `tcp://*:5591`,
deliberately separate from the live `5590`:

```bash
cd ~/ros2_ws
PYTHONPATH=src/harvester_telemetry_contract:src/harvester_telemetry_gateway \
  python3 -m harvester_telemetry_gateway.replay ~/harvester_audits/run_001 \
  --endpoint tcp://*:5591 --speed 1.0 --max-gap-s 0.25
```

**Terminal 2 — dashboard against replay, status disabled** (system python):

```bash
cd ~/ros2_ws
DISPLAY=:1 PYTHONPATH=src/harvester_dashboard \
  /usr/bin/python3 -m harvester_dashboard.main --pub tcp://127.0.0.1:5591 --status ''
```

Replay behaviour to expect:

- `--speed 1.0` preserves the original ~real-time pacing of the audit;
  `--speed 4` runs 4× faster; `--max-gap-s 0.25` caps the preserved silence
  between packets (use `--max-gap-s 1.0` for even slower pacing).
- A recording plays through once and exits.  When replay stops, the
  dashboard keeps running: streams go red (stale) after 2 s and ages climb —
  that is the intended freshness behaviour, not a fault.
- With `--status ''` the status line reads `status: disabled (replay)` and
  maintenance controls stay hidden — correct for a simulation-source replay.
- Optional: to also forward annotations somewhere, add
  `--annotation-pub tcp://127.0.0.1:5592` (still disabled by default and
  still never touches 5590/5600).

## Troubleshooting

| Symptom | Cause / fix |
|---|---|
| Dashboard window opens but "no packet yet" everywhere | Gateway (Run A) or replay (Run B) not running, or wrong `--pub` port. Check `ss -tln | grep -E '5590|5591'`. |
| `ModuleNotFoundError: No module named 'zmq'`/`'msgpack'` in a gateway/replay terminal | You used `/usr/bin/python3` or forgot `PYTHONPATH=src/harvester_telemetry_contract:src/harvester_telemetry_gateway` with plain `python3` (anaconda). |
| `ModuleNotFoundError: No module named 'PySide2'` in the dashboard terminal | You used anaconda `python3` instead of `/usr/bin/python3`, or the apt packages are missing (see Environment). |
| Dashboard dies instantly with an import error mentioning ROS | ROS env was sourced in the dashboard terminal — open a clean terminal and use `/usr/bin/python3`. |
| Streams red although Gazebo runs | Gateway terminal lost its ROS environment (`sensor_msgs` etc.) — re-source and restart only the gateway. |
| Everything stale after ~134 s in Run B | The audit finished replaying; restart replay (or loop it in a shell `while true; do ... ; sleep 1; done`). |

## Controls

| Key / button | Action |
|---|---|
| `1` | Render cutter view (render-only) |
| `2` | Render docking view (render-only) |
| `3` | Toggle sensor HUD |
| `4` | Toggle LiDAR inset |
| `0` / `Esc` | Clear current annotation |
| click on camera | Annotate (depth-validated; "NO DEPTH" toast otherwise) |

## Tests

```bash
PYTHONPATH=src/harvester_dashboard \
  /usr/bin/python3 -m unittest discover -s src/harvester_dashboard/test -v
```

Pure-python tests pass without any GUI packages.  The GUI smoke test skips
cleanly when PySide2/QtQuick are unavailable.  `test_replay_ingest.py` is a
live test (opt-in via `DASHBOARD_TEST_REPLAY=1` with replay running) that
proves every canonical channel is received and decoded from a real audit.
