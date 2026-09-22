---
name: harvester-dashboard
description: "Build, run, or extend the Qt Quick/QML operator dashboard (harvester_dashboard) that consumes the canonical ZeroMQ v1 telemetry, including its source-agnostic data model and non-actuating target annotation."
metadata:
  author: ros2_ws
  version: "1.0.0"
  status: stable
---

# Operator Dashboard (Qt Quick / QML)

Build and extend the source-agnostic Qt Quick/QML dashboard that subscribes to canonical ZeroMQ v1
and renders camera views, a sensor HUD, a LiDAR inset, and non-actuating operator target annotations.
It is designed on Xavier (simulation) and ported as-is to the Orin (hardware) with a different data
source — the dashboard never reads ROS topics.

This skill records the **hard environment constraints and the hard semantic rules** that govern this
specific dashboard implementation.

## When this applies

- Editing `src/harvester_dashboard/**` (QML views, decoders, models, bridge, config).
- Adding a view, a decoder, a HUD panel, or target-selection behavior.
- Diagnosing why the dashboard won't start, won't decode, or renders incorrectly.

## Core facts (internalize before editing)

<critical>
- **Interpreter split is the critical constraint.** Dashboard runs under `/usr/bin/python3`
  (system 3.8.10, has PySide2); the gateway runs under anaconda. Never mix them. System python can
  borrow anaconda's pyzmq/msgpack via
  `PYTHONPATH=/home/ubuntu/anaconda3/lib/python3.8/site-packages`.
- **PySide2 5.14 (Ubuntu 20.04) has NO QtQuickControls2 bindings.** QML must use only QtQuick 2
  primitives (Item, Rectangle, Text, Image, Canvas, Keys, ColumnLayout/RowLayout). No Buttons,
  Sliders, or Controls2 components — build touch buttons from Rectangle + MouseArea.
- **No `cv2` anywhere; PIL 8.2.0 present on both interpreters.** JPEG decode via PIL; H.264/H.265
  decoders are stubs that raise a clear error on `codec: h264|h265` (must not crash).
- **Render-only camera switching is a hard semantic:** keys `1`/`2` switch the rendered source and
  must NEVER emit socket traffic. Hardware stream enable/disable is a separate maintenance command
  (unavailable in simulation, never triggered by 1/2).
- **All dashboard events (including target clicks) are non-actuating annotations only.** No control
  command ever goes to the robot — the machine is human-operated, hydraulics via PLC/valve, no
  encoder, no joint cmd.
</critical>

## Reference map

- **Architecture + data model:** [references/architecture.md](references/architecture.md)
- **Environment setup + run/build:** [references/environment.md](references/environment.md)
- **Simulation vs hardware behavior (pose exporter, target selection, codecs):** [references/sim-vs-hardware.md](references/sim-vs-hardware.md)

## Quick workflow

1. Read `src/harvester_dashboard/harvester_dashboard/` (config.py, bridge.py, projection.py,
   protocol_shim.py) and the QML under `qml/` before editing.
2. Order matters when building: scaffold package + pure-python tests (zmq_source, decoders, models)
   BEFORE QML views — tests pass without GUI packages, de-risking the apt dependency.
3. Test pure-python parts (no GUI needed):
   ```bash
   PYTHONPATH=src/harvester_dashboard /usr/bin/python3 -m unittest discover -s src/harvester_dashboard/test
   ```
4. Smoke-test the GUI on `DISPLAY=:10` (the xrdp session — `:1` does not exist on this machine);
   guard GUI tests with import checks so they skip headless.

## Key invariants

- **Freshness uses local receipt monotonic time**, never header timestamps (those are display-only,
  labeled with their `clock_domain`, never compared across domains/hosts).
- Threading: ZMQ drain + decode in a worker QThread, post to UI via queued signals; `zmq.Again`
  terminates the drain loop each tick; per-channel `deque(maxlen=4)`, count drops.
- Image provider: PySide2 QML `Image` needs a QQuickImageProvider bridge.
- Regression guard: finished work leaves `git status` showing only `src/harvester_dashboard/`;
  `oil_palm_*` and both telemetry packages are untouched (except later additive pose-channel work).

## Related skills

- `docking-safety-guidance` — the docking-view operator safety HUD (stopping-distance + TTC) built
  inside this dashboard.
- `cutter-safety-guidance` — the cutter-view operator safety HUD (cutter tip clearance + cut
  sequence) built inside this dashboard.
- `telemetry-gateway` — produces the canonical ZeroMQ v1 packets this dashboard consumes; owns the
  `5590`/`5600` endpoints and the wire format.
- `harvester-simulation` — the simulation source whose raw sensor topics feed the gateway.
