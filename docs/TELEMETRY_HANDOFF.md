# Canonical Telemetry Handoff

## Purpose and safety boundary

This handoff records the implemented **Xavier simulation** telemetry baseline.
It is an observation-only layer between ROS 2/Gazebo and a future Orin
operator dashboard. It must never publish a joint command, velocity command,
TF transform, Gazebo service request, hydraulic command, PLC write, or
solenoid command.

The existing Gazebo/RViz control path remains authoritative:

```text
joint_state_publisher_gui -> /harvester/joint_commands -> Gazebo bridge
Gazebo bridge -> measured /harvester/joint_states -> RSP -> RViz
```

The telemetry gateway only subscribes to sensor and derived-perception topics.

## Current implementation state

Implemented on Xavier:

- Canonical ZeroMQ v1 three-frame wire format and strict header validation.
- One configurable PUB endpoint, default `tcp://*:5590`.
- One read-only REP status endpoint, default `tcp://*:5600`.
- Bounded newest-packet queues; `ZMQ_CONFLATE` is prohibited.
- Simulation RGB/depth/CameraInfo, raw LiDAR, five docking ranges, cutter
  range, trunk estimate, calibration status, and system status.
- `32FC1` or `16UC1` depth conversion to little-endian `uint16` millimetres.
- XYZ-only simulation LiDAR with declared point count, byte stride, and field
  layout.
- Required per-packet capability map.
- Opt-in exact packet recording and independent replay.

Not implemented yet:

- Orin hardware adapter/aggregator.
- Timestamp-correct pose exporter and world-fixed click target.
- Any hardware calibration, localization, encoder feedback, or robot actuation.

Implemented since (dashboard v1 on Xavier):

- `src/harvester_dashboard/`: source-agnostic Qt Quick operator dashboard
  (system `/usr/bin/python3`, PySide2 5.14 QtQuick 2 primitives, zero ROS
  imports).  Renders every canonical channel from the live gateway, the
  replay publisher, or the future Orin aggregator unchanged.
- View-only semantics enforced and wire-tested: keys `1`/`2` switch the
  rendered camera only; no socket traffic is emitted.  The dashboard's
  only sockets are one SUB to `--pub`, an optional read-only REQ to
  `--status` (disabled with `--status ''`), and an optional
  default-disabled annotation PUB (`--annotation-pub`, suggested
  `tcp://127.0.0.1:5592`) publishing `v1/operator/target_selection`.
- Camera-relative click annotation with depth validation ("NO DEPTH"
  toast and no annotation when depth is invalid), crosshair overlay,
  `0`/`Esc` clear; `tree_base_xyz` stays null (never claims world-fixed).
- JPEG/depth/LiDAR decoding per header metadata; H.264/H.265 are clear
  stub errors awaiting the Jetson decoder.  Freshness from local receipt
  monotonic time (red after 2 s silence); SIMULATION/HARDWARE/MIXED
  source badge; stream rows with ages, sequence gaps, drops, and decode
  errors; docking/cutter ranges, trunk estimate, calibration status.
- Maintenance stream controls exist in the UI but stay hidden unless the
  status REP reports hardware mode; no control endpoint is contacted.

The real human-operated harvester has no joint encoders. Do not claim a
world/tree-fixed target on hardware without a separately validated pose source.

## Authoritative files

| File | Role |
|---|---|
| `docs/canonical_zmq_v1.md` | Protocol authority: endpoints, headers, payloads, and compatibility. |
| `src/harvester_telemetry_contract/harvester_telemetry_contract/protocol.py` | Pure Python packet validation/packing used by producers and consumers. |
| `src/harvester_telemetry_gateway/harvester_telemetry_gateway/gateway_node.py` | Read-only Xavier ROS 2 to ZeroMQ gateway. |
| `src/harvester_telemetry_gateway/harvester_telemetry_gateway/recording.py` | Exact three-frame MessagePack audit recorder. |
| `src/harvester_telemetry_gateway/harvester_telemetry_gateway/replay.py` | Replay publisher; default endpoint is `tcp://*:5591`. |
| `src/harvester_telemetry_gateway/config/gateway.yaml` | Xavier endpoint, image quality, LiDAR reduction, and recording configuration. |
| `src/harvester_dashboard/` | Source-agnostic view-only Qt Quick dashboard (see its README for run/test commands). |
| `.kilo/plans/1787018525986-canonical-telemetry-dashboard-plan.md` | Forward plan for the Orin adapter and dashboard. |

## Canonical source mapping

| ROS 2 source | Canonical channel |
|---|---|
| `/harvester/platform_camera/depth/image_raw` | `v1/camera/cutter/rgb` |
| `/harvester/platform_camera/depth/depth/image_raw` | `v1/camera/cutter/depth` |
| `/harvester/platform_camera/depth/camera_info` | `v1/camera/cutter/camera_info` |
| `/harvester/docking_camera/depth/image_raw` | `v1/camera/docking/rgb` |
| `/harvester/docking_camera/depth/depth/image_raw` | `v1/camera/docking/depth` |
| `/harvester/docking_camera/depth/camera_info` | `v1/camera/docking/camera_info` |
| `/harvester/lidar/raw_points` | `v1/lidar/raw` |
| Five `/harvester/*_range` docking topics | `v1/range/docking` |
| `/harvester/cutting_tool_left_range` | `v1/range/cutter` |
| `/harvester/docking/trunk_center` | `v1/docking/trunk_estimate` |
| `/harvester/docking/calibration_status` | `v1/calibration/status` |
| Gateway state | `v1/system/status` |

Never substitute `/harvester/lidar/points`: it is deliberately zero-stamped
for RViz display only and is not a fusion or audit source.

## Time and calibration contract

- Sensor-derived simulation packets declare `source_mode: simulation` and
  `clock_domain: ros_sim_time`.
- Calibration/system status uses `clock_domain: utc_host` because those source
  messages lack an acquisition header. It must not be presented as Gazebo time.
- `gateway_monotonic_ns` is local freshness metadata only. Never compare it to
  a host UTC or PLC clock as if it were synchronized.
- The current camera/LiDAR and range calibration IDs identify nominal,
  URDF-derived simulation geometry. They are not physical hardware calibration.
- `capabilities` reports source support; stream health, drops, and recorder
  state are reported by `v1/system/status` and the REP response.

## Run: live simulation telemetry

Terminal 1 starts the supported simulation:

```bash
source /opt/ros/foxy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch oil_palm_harvester_description gazebo_harvester_and_tree.launch.py \
  harvester_collision_mode:=off articulation_control_mode:=kinematic
```

Terminal 2 starts the read-only gateway:

```bash
source /opt/ros/foxy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch harvester_telemetry_gateway gateway.launch.py
```

Do not pass the literal placeholder `/absolute/path/to/gateway.yaml`. If a
custom configuration is needed, pass its real absolute path, for example:

```bash
ros2 launch harvester_telemetry_gateway gateway.launch.py \
  config:=/home/ubuntu/harvester_gateway_audit.yaml
```

The gateway launch intentionally invokes the active `python3` environment.
On this Xavier setup, generated ROS Python entry points can select
`/usr/bin/python3`, which lacks the active environment's MessagePack/ZeroMQ
modules.

## Recording and replay

`record_dir: ""` disables recording. To make an audit configuration, copy the
provided YAML, set an explicit new capture directory, rebuild if source files
changed, then restart only the gateway. The recorder creates the directory and
writes one atomic `.msgpack` record for every complete canonical packet before
the live queue discards old packets.

Each record contains:

```text
record_format_version
recorded_monotonic_ns
frames = [channel bytes, MessagePack header bytes, binary payload bytes]
```

Do not delete individual files from an audit run while it is being recorded.
Camera/depth/LiDAR captures can consume significant storage; use a dedicated
directory per run and manage retention outside the gateway.

Replay needs no Gazebo or RViz process. Start a dashboard/subscriber first,
then run:

```bash
source /opt/ros/foxy/setup.bash
source ~/ros2_ws/install/setup.bash
python3 -m harvester_telemetry_gateway.replay \
  /home/ubuntu/harvester_audits/run_001 \
  --endpoint tcp://*:5591 --speed 0.2 --max-gap-s 1.0
```

The replay endpoint is intentionally separate from the live `5590` endpoint.
The dashboard chooses either Xavier live data, replay data, or future local
Orin hardware data; it must not assume that mixed sources have comparable time.

Run the dashboard against replay (no ROS environment needed; system Python
with the PySide2/QtQuick apt packages installed):

```bash
DISPLAY=:1 PYTHONPATH=src/harvester_dashboard \
  /usr/bin/python3 -m harvester_dashboard.main --pub tcp://127.0.0.1:5591 --status ''
```

Against the live gateway, point `--pub tcp://127.0.0.1:5590 --status
tcp://127.0.0.1:5600`.  Dashboard tests:

```bash
PYTHONPATH=src/harvester_dashboard \
  /usr/bin/python3 -m unittest discover -s src/harvester_dashboard/test -v
```

## Verification

```bash
cd ~/ros2_ws
source /opt/ros/foxy/setup.bash
source install/setup.bash
PYTHONPATH=src/harvester_telemetry_contract:src/harvester_telemetry_gateway:${PYTHONPATH} \
  python3 -m unittest discover -s src/harvester_telemetry_contract/test -v
PYTHONPATH=src/harvester_telemetry_contract:src/harvester_telemetry_gateway:${PYTHONPATH} \
  python3 -m unittest discover -s src/harvester_telemetry_gateway/test -v
```

The current source test baseline is eight passing tests: four protocol tests,
three encoder tests, and one exact-frame recording/reload test.

For a live gateway, query the read-only status endpoint or subscribe to a
channel prefix. A changing cutter-camera sequence/payload while a robot joint
moves confirms the camera observation reaches canonical transport; the gateway
does not publish joint state itself.

## Forward handoff to Orin

The Orin implementation has one canonical adapter/aggregator as the only PUB
owner of `tcp://*:5590`. OAK, LiDAR, and range ingestion modules feed that
aggregator; they must not each bind competing canonical PUB sockets. It uses
the same protocol, capability map, recorder format, REP status shape, and
replay fixtures as Xavier.

Hardware codecs may be JPEG, H.264, or H.265 as declared by the packet header.
Hardware LiDAR may add intensity/tag/line/point-time fields, always declared
in `point_fields`. The dashboard must decode from metadata rather than assume
the simulation's JPEG and XYZ-only payloads.
