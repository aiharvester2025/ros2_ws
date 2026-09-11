# Environment setup + run/build

## The interpreter split (the critical constraint)

| Interpreter | Has | Lacks | Use for |
|---|---|---|---|
| `/home/ubuntu/anaconda3/bin/python3` (3.8.8, active `python3`) | zmq, msgpack, numpy, PIL | PySide2 | gateway, replay |
| `/usr/bin/python3` (3.8.10) | PySide2 QtCore/QtGui | QtQuick bindings, zmq, msgpack | dashboard |

The dashboard must run under `/usr/bin/python3`; the gateway under anaconda. **Never mix them.**

System python can borrow anaconda's pyzmq/msgpack via
`PYTHONPATH=/home/ubuntu/anaconda3/lib/python3.8/site-packages` (verified working) — a fallback if
apt install of `python3-zmq`/`python3-msgpack` fails.

## One-time apt install (D0)

Required packages (all confirmed as apt candidates on Ubuntu 20.04):

```
python3-pyside2.qtquick        (5.14.0)
python3-zmq                    (18.1.1)
python3-msgpack                (0.6.2)
qml-module-qtquick2
qml-module-qtquick-window2
qml-module-qtquick-layouts     (5.12.8)
```

Sudo works but requires the password **interactively**. Do NOT write the password into any file,
script, or doc.

Note: PySide2 5.14 has no `QtQuickControls2` bindings even as an apt candidate — QML must use
QtQuick 2 primitives only.

## Run the dashboard

```bash
cd ~/ros2_ws
DISPLAY=:10 PYTHONPATH=src/harvester_dashboard \
  /usr/bin/python3 -m harvester_dashboard.main \
  --pub tcp://127.0.0.1:5590 \
  --status tcp://127.0.0.1:5600
```

No ROS sourcing. Window opens on `DISPLAY=:10` (the xrdp session, 2560×1440) — this machine's `:0`
is a headless 640×480 fallback and `:1` does not exist. Streams appear within ~2 s.

For replay-based validation (no Gazebo), point `--pub` at the replay endpoint 5591 (and use
`--status ''` to disable the status REP). Optional flags: `--annotation-pub tcp://127.0.0.1:5592`
(default disabled) and `--dock-pub tcp://127.0.0.1:5593` (for the dock-orchestrator flow).

## Build

The dashboard is a plain Python package under `src/harvester_dashboard/` (no colcon message build
needed). The gateway/contract use colcon with `--merge-install --symlink-install`.

## Order of work (de-risks the apt dependency)

1. Scaffold package + pure-python tests (zmq_source, decoders, models) — pass without GUI packages.
2. apt install (D0).
3. QML views + image provider bridge.
4. GUI smoke-test on DISPLAY=:10.

## Regression guard

Finished work must leave `git status` showing only `src/harvester_dashboard/` (plus plan/docs if
updated). `oil_palm_*` and both telemetry packages stay untouched.
