---
name: telemetry-gateway
description: "Build, run, or extend the canonical ZeroMQ v1 telemetry gateway and protocol contract (harvester_telemetry_gateway + harvester_telemetry_contract) in this ROS2 Foxy workspace."
metadata:
  author: ros2_ws
  version: "1.0.0"
  status: stable
---

# Canonical Telemetry Gateway & Protocol

Operate and extend the read-only Xavier simulation gateway that subscribes to ROS topics and
publishes a frozen canonical ZeroMQ v1 wire format to the Orin dashboard, plus the shared
pure-Python protocol contract.

This skill records the **frozen wire format, the additive-only boundary, and the runtime quirks**
that a smaller model would otherwise rediscover the hard way.

## When this applies

- Editing `src/harvester_telemetry_gateway/**` or `src/harvester_telemetry_contract/**`.
- Adding a new canonical channel, encoder, or sensor subscription.
- Running/recording/replaying the gateway or diagnosing why a stream is missing.

## Core facts (internalize before editing)

<critical>
- **The gateway is strictly additive and read-only.** It must never modify Gazebo, RViz, URDF, TF,
  controls, or calibration nodes, and never publish a control/actuation command. It only *reads* ROS
  topics and *publishes* ZeroMQ. `git status` after gateway work must show only telemetry packages.
- **Raw LiDAR topic is `/harvester/lidar/raw_points`, never `/harvester/lidar/points`** (the `points`
  topic is zero-stamped and must not be used).
- **The wire format is frozen** in `docs/canonical_zmq_v1.md`. Every packet is exactly 3 frames:
  `[channel bytes, MessagePack header bytes, binary payload bytes]`. Integer-nanosecond timestamps
  (`acquisition_timestamp_ns`), a required `capabilities` map in every header.
- **No `ZMQ_CONFLATE`** — use bounded newest-wins queues (drop complete old packets).
</critical>

## Reference map

- **Wire format + channel list:** [references/wire-format.md](references/wire-format.md)
- **Runtime quirks + run/record/replay:** [references/runtime.md](references/runtime.md)

## Quick workflow

1. Read `src/harvester_telemetry_contract/harvester_telemetry_contract/protocol.py` (the validator)
   and `src/harvester_telemetry_gateway/harvester_telemetry_gateway/gateway_node.py` before editing.
2. A new channel means three coordinated edits: register it in `CANONICAL_CHANNELS` (+ any header
   validation) in `protocol.py`, add an encoder in `encoders.py`, and add the subscription + handler
   in `gateway_node.py`.
3. Test (no ROS graph needed for the contract/encoders):
   ```bash
   PYTHONPATH=src/harvester_telemetry_contract:src/harvester_telemetry_gateway \
     python3 -m unittest discover -s src/harvester_telemetry_contract/test
   ```
   Gateway tests that import `sensor_msgs` must run under the ROS Foxy environment
   (`source /opt/ros/foxy/setup.bash`), not the anaconda python.
4. Build with `--merge-install --symlink-install` (this workspace's layout).

## Key invariants

- Depth is normalized to row-major uint16 millimetres regardless of 16UC1/32FC1 source (NaN→0,
  clip 1–65535).
- LiDAR headers declare `point_count`, `point_stride_bytes`, `point_fields`; simulation sends XYZ
  only, hardware may add intensity/tag/line/point-time.
- Ports: single PUB `tcp://*:5590`; status REP `tcp://*:5600`; replay PUB `tcp://*:5591`; dashboard
  annotation PUB `5592` (default disabled); dock-request PUB `5593` (out-of-contract, orchestrator
  only). See `docs/canonical_zmq_v1.md` for the full endpoint table.
- The canonical source mapping (ROS topic → channel) is authoritative in `docs/TELEMETRY_HANDOFF.md`;
  it now includes `/harvester/lidar/imu` → `v1/imu/lidar`, `/harvester/platform_camera/imu` →
  `v1/imu/camera`, and `/harvester/dock/status` → `v1/docking/plan`.
- The gateway launches under the **active (anaconda) python**, because ROS entry points select
  `/usr/bin/python3` which lacks msgpack/zmq. Do not "fix" this — it's deliberate.
- `flush_one_packet()` scans channel queues alphabetically and returns after one send; a persistent
  high-rate channel sorting early could starve later channels at high load. Known and acceptable at
  current rates — do not change without cause.
- `on_docking_range` republishes all five records on every single-sensor update (~100 msgs/s
  aggregate on `v1/range/docking`). Consumers should overwrite state, not accumulate.

## Related skills

- `harvester-simulation` — the ROS topics this gateway subscribes to (raw sensor topics).
- `harvester-dashboard` — the consumer of the canonical packets this gateway publishes.
- `sensor-calibration` — the range/camera-LiDAR calibration whose status this gateway forwards.
