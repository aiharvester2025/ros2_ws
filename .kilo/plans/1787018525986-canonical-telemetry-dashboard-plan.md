# Canonical Xavier–Orin Telemetry and Operator Dashboard — Implementation Plan (Phase 1)

**Phase:** 1 of 2 — Non-actuating operator annotation and guidance only.
**Scope boundary:** This plan delivers a telemetry gateway, protocol adapter, pose exporter (simulation), and Qt Quick dashboard. It does **not** include hydraulic actuation, solenoid valve control, PLC integration, joint command paths, or Eye-in-Hand closed-loop guidance. Those are Phase 2.

**Baseline constraint:** Do not modify any existing file in `src/oil_palm_harvester_description/` or `src/oil_palm_tree_description/`. All new work is additive.
**Hardware constraint:** The real Orin robot is human-operated. It has no joint encoders, no joint command interface, and no software control path to the hydraulic system. The dashboard is a passive guidance tool only.

## Implementation status — 2026-08-18

Completed in this Xavier workspace:

- Canonical v1 protocol/validator, one PUB endpoint and one read-only REP
  status endpoint.
- Read-only Xavier ROS 2 gateway for the current cameras, raw LiDAR, ranges,
  trunk estimate, and calibration status.
- Depth normalization, XYZ LiDAR layout metadata, required capability flags,
  bounded newest-packet queues, exact-packet recording, and replay.

Still planned, not implemented here:

- Orin hardware adapter/aggregator and hardware-side recorder.
- Qt Quick dashboard, codec-aware decoder, and UI/HUD.
- Simulation pose-at-image exporter and any target anchoring.
- All real-hardware calibration/localization work.

The current operational reference is `docs/TELEMETRY_HANDOFF.md`; the protocol
authority is `docs/canonical_zmq_v1.md`.

---

## 1. Freeze the canonical ZeroMQ v1 schema

**File:** `docs/canonical_zmq_v1.md` (new, in the `harvester_vision` repo)

### 1.1 Wire format
- Transport: `tcp://*:PORT` for PUB bindings; `tcp://HOST:PORT` for REQ/REP and SUB connect.
- Pattern: PUB/SUB for telemetry streams; REQ/REP for control/status queries.
- Multipart message: `[channel_bytes, msgpack_header_bytes, binary_payload_bytes]`
- First frame is the channel string (e.g. `b"v1/camera/cutter/rgb"`). ZeroMQ topic filtering uses this frame only.
- Header is always MessagePack map. Payload is binary (JPEG bytes, depth bytes, LiDAR point bytes, JSON bytes).
- Bounded queues with `RCVHWM`/`SNDHWM`. Drop complete old packets on overflow. Never use `ZMQ_CONFLATE` with multipart messages.
- Every header carries a `capabilities` map of string-to-boolean source feature
  flags. It describes installed support (for example depth, LiDAR intensity,
  cutter range, or world-fixed target anchoring); dynamic stream availability
  remains in `v1/system/status`.

### 1.2 Required header fields (every message)
```json
{
  "schema_version": 1,
  "source_mode": "simulation" | "hardware",
  "source_id": "xavier" | "orin",
  "sequence": 0,
  "frame_id": "platform_depth_camera_optical_frame",
  "acquisition_timestamp_ns": 1234567890123456789,
  "clock_domain": "ros_sim_time" | "utc_host" | "plc_rtc_utc",
  "gateway_monotonic_ns": 1234567890123456789,
  "codec": "jpeg" | "h264" | "h265" | "depth_uint16_le" | "lidar_xyz_f32" | "json",
  "pixel_encoding": "MJPEG" | "H264" | "H265" | "",
  "width": 1280,
  "height": 720,
  "calibration_id": "gazebo_nominal_camera_lidar_v1",
  "capabilities": {"camera.cutter.depth": true, "lidar.intensity": false},
  "transform_valid": true,
  "transform_freshness_s": 0.01
}
```

**Depth encoding decision:** Use `depth_uint16_le` encoding for depth payloads. Values are millimeters (matches ROS `16UC1` convention). The decoder divides by 1000.0 to obtain meters. This avoids float32 precision loss at range and keeps the wire format compact.

### 1.3 Canonical channels
| Channel | Payload codec | Payload format |
|---|---|---|
| `v1/camera/cutter/rgb` | jpeg / h264 / h265 | Annex-B byte stream (H.264/H.265) or JPEG bytes |
| `v1/camera/cutter/depth` | depth_uint16_le | Row-major uint16 LE millimeters, width×height |
| `v1/camera/cutter/camera_info` | json | ROS CameraInfo JSON (K, D, R, P, width, height) |
| `v1/camera/docking/rgb` | jpeg / h264 / h265 | Same as cutter rgb |
| `v1/camera/docking/depth` | depth_uint16_le | Same as cutter depth |
| `v1/camera/docking/camera_info` | json | ROS CameraInfo JSON |
| `v1/lidar/raw` | lidar_xyz_f32 | Row-major float32 LE XYZ triplets, point_count×3 |
| `v1/range/docking` | json | Array of 5 sensor readings |
| `v1/range/cutter` | json | Single sensor reading |
| `v1/docking/trunk_estimate` | json | Side-pair trunk centre |
| `v1/calibration/status` | json | Calibration ID, frame, validity, age |
| `v1/system/status` | json | Gateway/adapter heartbeat, uptime, errors |
| `v1/operator/target_selection` | json | Operator annotation event (non-actuating) |

#### 1.3.1 JSON payload formats

**`v1/range/docking`** — array of 5 sensor objects:
```json
[
  {
    "telemetry_key": "center_line",
    "distance_m": 2.345,
    "valid": true,
    "timestamp_us": 1234567890123456,
    "frame_id": "center_range_sensor_link",
    "calibration_id": "gazebo_nominal_c_channel_v1"
  }
]
```

**`v1/range/cutter`** — single sensor object (same schema as one docking sensor).

**`v1/docking/trunk_estimate`** — side-pair trunk centre:
```json
{
  "header": {"stamp": {"sec": 1234567890, "nanosec": 123456789}, "frame_id": "c_channel_reference"},
  "pose": {"position": {"x": 1.5, "y": 0.0, "z": 0.8}, "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}},
  "covariance": [0.0004, 0.0, 0.0, 0.0, 0.0004, 0.0, 0.0, 0.0, 0.0004],
  "status": "VALID",
  "equivalent_diameter_m": 0.55
}
```

**`v1/calibration/status`**:
```json
{
  "calibration_id": "gazebo_nominal_camera_lidar_v1",
  "mode": "simulation_only",
  "reference_frame": "c_channel_reference",
  "transform_valid": true,
  "transform_freshness_s": 0.01,
  "note": "URDF-derived nominal geometry"
}
```

**`v1/system/status`**:
```json
{
  "source_id": "xavier",
  "uptime_s": 1234.5,
  "streams": {
    "v1/camera/cutter/rgb": {"enabled": true, "last_sequence": 12345, "last_timestamp_us": 1234567890123456},
    "v1/lidar/raw": {"enabled": true, "last_sequence": 678, "last_timestamp_us": 1234567890123000}
  },
  "errors": []
}
```

**`v1/operator/target_selection`** — published by dashboard on Orin:
```json
{
  "event": "target_selected",
  "timestamp_us": 1234567890123456,
  "source_mode": "hardware",
  "frame_id": "platform_depth_camera_optical_frame",
  "pixel": {"u": 640, "v": 360},
  "point_3d": {"x": 0.0, "y": 0.0, "z": 2.5},
  "depth_m": 2.5,
  "tree_base_xyz": null,
  "note": "camera-relative annotation; no actuation"
}
```
In simulation mode, `tree_base_xyz` is populated with the transformed coordinates. In hardware mode, it is `null`.

### 1.4 Endpoints and ownership

Each source has exactly one canonical PUB owner. It binds one configurable
endpoint, default `tcp://*:5590`, and carries all `v1/*` channels on that one
socket. Xavier binds on the Xavier host; the future Orin adapter binds on Orin
(`192.168.50.10`). These do not conflict because they are different machines.
The dashboard profile selects **one** endpoint: `tcp://XAVIER_IP:5590` for
simulation or `tcp://127.0.0.1:5590` for Orin hardware.

Future OAK, LiDAR, and range ingest modules must not each bind a canonical PUB
socket. They feed one Orin adapter/aggregator, which is the only owner of port
5590 and assigns canonical sequence numbers. This preserves uniform
multipart messages and lets one recorder cover the complete hardware run.

Each source also exposes one configurable, read-only REQ/REP status endpoint,
default `tcp://*:5600`. Its response contains schema version, active profile,
calibration revisions, source capabilities, live streams, packet drops,
recorder state, and errors. It cannot command robot motion.

---

## 2. Xavier simulation gateway (new ROS 2 package)

**New package:** `harvester_telemetry_gateway`
**Location:** `/home/ubuntu/ros2_ws/src/harvester_telemetry_gateway/`
**Language:** Python/rclpy (matches existing nodes).
**Dependencies:** `rclpy`, `sensor_msgs`, `std_msgs`, `geometry_msgs`, `json`, `msgpack`, `zmq`.

### 2.1 Responsibilities
- Subscribe to existing Gazebo topics only. Do not modify Gazebo, URDF, TF, controls, or calibration nodes.
- Republish as canonical ZeroMQ v1 multipart.
- Publish `source_mode: simulation`, `clock_domain: ros_sim_time`, and a `gateway_monotonic_timestamp` sampled from `time.monotonic()` at publish time.
- Never compare Gazebo simulation time directly to Orin UTC.

### 2.2 Topic mapping
| Source topic | Type | Canonical channel | Notes |
|---|---|---|---|
| `/harvester/platform_camera/depth/image_raw` | Image | `v1/camera/cutter/rgb` | Encode as JPEG for simulation transport. Header `codec: jpeg`, `pixel_encoding: MJPEG`. |
| `/harvester/platform_camera/depth/camera_info` | CameraInfo | `v1/camera/cutter/camera_info` | JSON encode K, D, R, P, width, height. |
| `/harvester/docking_camera/depth/image_raw` | Image | `v1/camera/docking/rgb` | Encode as JPEG. Header `codec: jpeg`, `pixel_encoding: MJPEG`. |
| `/harvester/docking_camera/depth/camera_info` | CameraInfo | `v1/camera/docking/camera_info` | JSON encode. |
| `/harvester/lidar/raw_points` | PointCloud2 | `v1/lidar/raw` | Downsample/ROI per config. Float32 XYZ only. |
| `/harvester/center_range`, `/harvester/left_45_range`, ... | Range | `v1/range/docking` | JSON per sensor. |
| `/harvester/cutting_tool_left_range` | Range | `v1/range/cutter` | JSON. |
| `/harvester/docking/trunk_center` | PoseWithCovarianceStamped | `v1/docking/trunk_estimate` | JSON. |
| `/harvester/docking/calibration_status` | String | `v1/calibration/status` | JSON passthrough. |

### 2.3 Implementation details
- Use a single `rclpy` node with one subscriber per source topic.
- Bounded application queue (e.g. `collections.deque(maxlen=4)`) per stream. Drop complete old packets when full.
- JPEG quality, lossless-depth compression, LiDAR downsampling, LiDAR ROI, and per-stream enablement are YAML-configurable via the node’s `--config` argument or ROS 2 parameters.
- Calibration IDs come from the existing `config/*.nominal.json` files. Do not duplicate them.
- Label the simulation camera–LiDAR transform explicitly as `URDF-derived nominal geometry` in the calibration status stream.
- An opt-in `record_dir` writes each complete canonical packet as an exact
  three-frame MessagePack audit record before the bounded live queue can drop
  old data. Storage retention is an operator responsibility.

### 2.4 Files
- `harvester_telemetry_gateway/harvester_telemetry_gateway/gateway_node.py`
- `harvester_telemetry_gateway/harvester_telemetry_gateway/config/gateway.yaml`
- `harvester_telemetry_gateway/package.xml`
- `harvester_telemetry_gateway/CMakeLists.txt`
- `harvester_telemetry_gateway/launch/gateway.launch.py`

---

## 3. Orin hardware adapter (new module in harvester_vision)

**Repo:** `https://github.com/aiharvester2025/harvester_vision`
**Language:** Python 3 (matches existing `oak_rgb_publisher.py` style).
**New directory:** `adapter/`

### 3.1 Responsibilities
- Act as a protocol adapter separate from the dashboard.
- Translate existing OAK camera packets, range JSON, LiDAR output, calibration metadata, and hardware timestamps into canonical ZeroMQ v1 messages.
- Map hardware profile topic/device names and frames.

### 3.2 Components

#### 3.2.1 `oak_canonical_publisher.py`
Replaces `oak_rgb_publisher.py`:
- Same DepthAI v3 pipeline and capture logic.
- Sends encoded frames to the single Orin canonical adapter/aggregator, which
  owns the canonical PUB endpoint (`tcp://*:5590`). It does not bind a
  per-camera canonical PUB port.
- The adapter publishes multipart canonical messages:
  `[channel_bytes, msgpack_header_bytes, encoded_video_bytes]`.
- Control PULL endpoints (`5566`, `5567`) accept the same `{"topic": "...", "enabled": true/false}` commands. When `enabled=false`, the DepthAI pipeline is stopped entirely (not just paused). When `enabled=true`, the pipeline is restarted. This eliminates CPU/XLink load from inactive cameras.
- **Primary encoding: H.264 or H.265** via DepthAI `VideoEncoder` with `Profile.H264_HP` or `Profile.H265_MAIN`. The OAK hardware encoder produces Annex-B byte-stream frames. The canonical `codec` header field is set to `"h264"` or `"h265"`, and `pixel_encoding` is set to `"H264"` or `"H265"`.
- **MJPEG is a fallback only**, used when the hardware encoder is unavailable or when debugging. Default config selects H.264/H.265.
- Build canonical header fields (`schema_version`, `source_mode: hardware`, `source_id: orin`, `clock_domain: plc_rtc_utc`, etc.).
- Use `time_sync.capture_timestamp_us()` for `acquisition_timestamp` and `time.monotonic()` for `gateway_monotonic_timestamp`.
- The encoded Annex-B frame is sent as the binary payload. The dashboard backend uses Jetson hardware decoding (see Section 5.7).

#### 3.2.2 `range_canonical_publisher.py`
- Reads the Raspberry Pi telemetry (existing `sensor_viewer.py` input).
- Parses `harvester.sensor-telemetry.v2` JSON.
- Republishes as `v1/range/docking` canonical multipart messages.
- Adds header fields: `frame_id` from config, `calibration_id`, `transform_valid`, `transform_freshness_s`.

#### 3.2.3 `lidar_canonical_publisher.py`
- Connects to MID-360 (Livox) UDP or serial per existing hardware setup.
- Downsample to configurable rate (e.g. 10 Hz).
- Publish `v1/lidar/raw` as `lidar_xyz_f32` binary payload.
- Header includes `frame_id: vehicle_lidar_link`, `calibration_id`.

#### 3.2.4 `cutter_range_canonical_publisher.py`
- Subscribes to the moving cutter range sensor on Orin hardware.
- Publishes `v1/range/cutter` canonical multipart messages.
- Header includes `frame_id: cutting_tool_left_range_sensor_link`, `calibration_id`, `transform_valid`, `transform_freshness_s`.

#### 3.2.5 `adapter_status.py`
- REQ/REP server on `tcp://0.0.0.0:9000`.
- Returns JSON with schema version, active profile, calibration revision, stream availability, and latest-status snapshot.
- All Orin adapter publishers use the same opt-in exact-packet recorder as
  Xavier. Real OAK, LiDAR, and range audit captures are replayable by the
  dashboard with the same tooling as Gazebo captures.

### 3.3 Config
- `adapter/hardware_profiles/cutter_camera.yaml`
- `adapter/hardware_profiles/docking_camera.yaml`
- `adapter/hardware_profiles/lidar.yaml`
- `adapter/hardware_profiles/ranges.yaml`

Each profile contains: `topic`, `channel`, `frame_id`, `calibration_id`, `codec`, `width`, `height`, `fps`, `enabled`, and codec-specific fields:
- For `jpeg`: `mjpeg_quality` (1–100).
- For `h264` / `h265`: `bitrate_kbps` (target encoder bitrate), `profile` (e.g. `HIGH` for H.264, `MAIN` for H.265), `keyframe_interval_s`.

### 3.4 Evolution path from existing code
- `oak_rgb_publisher.py` → rename to `adapter/oak_canonical_publisher.py`. It binds to the **same** PUB ports (`5556`, `5557`) and control PULL ports (`5566`, `5567`) but switches from single-part msgpack to multipart canonical format. The control command schema is extended but remains backward-compatible for `enabled` toggles.
  - **Breaking change for `oak_rgb_viewer.py`:** The existing viewer expects single-part msgpack. After migration, it must be replaced by the new `dashboard/` app or updated to parse multipart canonical messages. Do not run both simultaneously on the same ports.
- `sensor_viewer.py` remains as a standalone debug viewer for the raw Raspberry Pi JSON stream; `range_canonical_publisher.py` replaces its subscription path for dashboard data.
- `time_sync.py` is reused unchanged.
- `set_oak_ip.py` remains unchanged.

**Cutter range on Orin:** The `harvester_vision` hardware includes a cutter range sensor. `cutter_range_canonical_publisher.py` reads it and publishes `v1/range/cutter` alongside the docking range streams. The Xavier simulation gateway also publishes `v1/range/cutter` from `/harvester/cutting_tool_left_range`.

---

## 4. Pose-at-image exporter (new ROS 2 package)

**New package:** `harvester_pose_exporter`
**Location:** `/home/ubuntu/ros2_ws/src/harvester_pose_exporter/`
**Language:** Python/rclpy.
**Constraint:** Do not touch existing `oil_palm_harvester_description` nodes.
**Scope: Simulation only.** This package runs on Xavier during Gazebo simulation. The real Orin hardware has no joint encoders (human-operated robot), so hardware target anchoring uses a different strategy described in Section 4.5.

### 4.1 Why forward kinematics instead of TF2
The existing simulation contracts (`CALIBRATION_FRAME_CONTRACT.md`, `CAMERA_LIDAR_CALIBRATION_CONTRACT.md`) explicitly prohibit dynamic TF lookup at exact Gazebo simulation timestamps because:
- `joint_state_publisher_gui` and `robot_state_publisher` use system time.
- Gazebo publishes sensor data and the kinematic plugin publishes `world -> base_link` in simulation time.
- The time-domain mismatch makes TF2 lookups at exact sim timestamps unreliable for the arm-mounted cameras.

The Gazebo kinematic plugin does publish measured joint states on `/harvester/joint_states` in simulation time. Using these measured positions with URDF forward kinematics bypasses the TF time-domain problem and gives the true simulated chain pose at the image acquisition time.

**On real Orin hardware:** Joint encoders do not exist. The FK path is unavailable. Hardware target anchoring must use a configured stable map frame or tree-local frame, not joint-state FK.

### 4.2 Responsibilities
- For every cutter and docking image, publish the camera-to-`tree_base` transform sampled in the same Gazebo time domain as that image.
- Publish transform with validity and age.

### 4.3 Implementation
- Subscribe to `/harvester/joint_states` (Gazebo plugin measured feedback, sim time).
- Subscribe to `/harvester/platform_camera/depth/image_raw` and `/harvester/docking_camera/depth/image_raw`.
- For each image, record `header.stamp` (Gazebo sim time).
- Select the most recent `/harvester/joint_states` message whose stamp is at or before the image stamp. Reject if the joint-state age exceeds `0.1 s`.
- Load the URDF (`oil_palm_harvester_kinematic.urdf`) and use `urdf_parser` + `kdl_parser` / `pykdl` to compute forward kinematics from `base_link` to each camera optical frame using the measured joint positions.
- Static sub-chains (e.g. `cutting_arm_base_link -> platform_depth_camera_optical_frame`) are computed once from the URDF and cached.
- The Gazebo plugin publishes `world -> base_link` in sim time. Look up this transform at the image stamp using TF2 (this single dynamic link is published by the Gazebo plugin in the same sim-time domain as the image).
- Compose: `T_tree_base_camera = T_tree_base_world * T_world_base_link * T_base_link_camera_optical`.
  - `T_tree_base_world` is the known static transform `world -> tree_base` = `(8.5, 0, 0)` in simulation. In a future hardware variant, this would come from rail localisation.
  - `T_world_base_link` comes from the Gazebo plugin at the image sim timestamp.
  - `T_base_link_camera_optical` comes from FK using measured joint states.
- Publish on `/harvester/pose_exporter/cutter_to_tree_base` and `/harvester/pose_exporter/docking_to_tree_base`.
- Include `header.stamp` (image acquisition time), `transform_status`, `transform_age_s`, and `joint_state_age_s`.

### 4.4 Published message format
```json
{
  "header": {
    "stamp": {"sec": 1234567890, "nanosec": 123456789},
    "frame_id": "tree_base"
  },
  "child_frame_id": "platform_depth_camera_optical_frame",
  "transform": {
    "translation": {"x": 8.5, "y": 0.0, "z": 0.0},
    "rotation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
  },
  "transform_status": "VALID",
  "transform_age_s": 0.01,
  "joint_state_age_s": 0.005
}
```
`transform_status` enum: `VALID`, `STALE_JOINT_STATE`, `STALE_WORLD_BASE_LINK`, `MISSING_JOINT_STATE`, `FK_FAILED`.

### 4.5 Files
- `harvester_pose_exporter/harvester_pose_exporter/pose_exporter.py`
- `harvester_pose_exporter/package.xml`
- `harvester_pose_exporter/CMakeLists.txt`
- `harvester_pose_exporter/launch/pose_exporter.launch.py`

### 4.6 Validation gate
- Until this exporter passes validation, the dashboard must not claim a world-fixed clicked target.
- Validation script: `python3 scripts/validate_pose_exporter.py` while Gazebo is running. Checks that transform is finite, age < 0.1 s, and `tree_base` is the expected static frame at `(8.5, 0, 0)`.

### 4.7 Hardware target anchoring (no encoders — future design note)
The real Orin robot is human-operated with no joint encoders. When the system moves to hardware:
- The arm-mounted camera pose cannot be computed from joint states.
- Options under evaluation:
  1. **Fixed-arm acquisition:** Operator positions the arm at a known/registered pose before target selection. The camera extrinsics are then fixed and known.
  2. **Base-mounted camera alternative:** Mount the operator-view camera on `base_link` instead of the arm, eliminating joint-dependent pose.
  3. **Visual arm-pose estimation:** Use the camera/LiDAR to estimate the arm pose from the scene, though this is circular if the arm is in the FoV.
  4. **Tree-local frame:** Express targets in a tree-local or map frame using only the base localisation and static camera extrinsics (arm assumed fixed during target acquisition).
- The `tree_base` frame remains the simulation anchor. On hardware, it is replaced by the configured stable map or tree-local frame.
- The canonical ZeroMQ header field `frame_id` documents which frame is used, so the dashboard can adapt without wire-format changes.

---

## 5. Orin Qt Quick/QML dashboard (new module in harvester_vision)

### 5.1 Technology choice
- **Backend:** Python 3 + PySide2 (or PySide6) + `pyzmq` + `msgpack`.
- **Frontend:** Qt Quick/QML.
- Rationale: Matches existing Python-first style on both Xavier and Orin. PySide2 is available on Ubuntu/Jetson. If Orin latency later demands C++, the frozen ZeroMQ wire format allows swapping the backend without touching QML.

### 5.2 Architecture
```
dashboard/
  main.py                  # QApplication + QQuickView, owns ZMQ sockets and decoder pool
  decoder/
    __init__.py
    jpeg_decoder.py         # cv2.imdecode; used for Xavier simulation JPEG frames
    h264_decoder.py         # Jetson hardware decode via PyNvCodec; primary Orin path
    h265_decoder.py         # Jetson hardware decode via PyNvCodec; primary Orin path
    lidar_decoder.py        # struct.unpack XYZ float32
    depth_decoder.py        # uint16 LE -> float32 meters
    range_decoder.py        # JSON parse
  model/
    __init__.py
    telemetry_model.py      # Per-stream state: last header, last payload, freshness_s, stale flag
    target_model.py         # Selected target: mode-aware. Simulation stores tree_base_xyz + camera uv. Hardware stores camera-relative 3-D point + uv only. Both track validity, depth_m, and out_of_view flag.
  zmq/
    __init__.py
    subscriber.py           # Bounded deques per channel, drain loop, multipart parse
    control_client.py       # REQ/REP to local adapter control endpoint
  qml/
    Dashboard.qml           # Main viewport
    CameraView.qml          # Image display + click handler
    HudOverlay.qml          # Source badge, freshness, ranges, trunk, calibration
    SensorPanel.qml         # Five docking ranges + cutter range + trunk status
    LidarInset.qml          # 3-D point inset (OpenGL via Qt Quick 3D or a QQuickFramebufferObject)
    TargetOverlay.qml       # Selected target dot + crosshair + stale/out-of-view warning
```

### 5.3 Main viewport controls
- **Touch buttons (primary):** QML on-screen buttons for camera switching, HUD toggle, LiDAR toggle, and target clear. This matches the existing Orin todo item for finger-touch operation.
- **Keyboard (supplementary):** `1` / `2`: switch cutter / docking camera. `3`: hide/show sensor HUD. `4`: hide/show LiDAR inset or projected overlay. `0` / `Esc`: clear selected target.
- Touch and keyboard controls send the same enable/disable commands to the local adapter control endpoint.

### 5.4 HUD contents
- Source badge: prominent `SIMULATION` / `HARDWARE`. The mode is determined by the canonical header field `source_mode` on any active stream. If streams disagree, the HUD shows `MIXED` and logs a warning.
- Camera freshness: seconds since last frame per active stream; turn red when > 2 s.
- LiDAR freshness: same.
- Five docking ranges, cutter range, and calibrated trunk status (from `v1/range/docking` and `v1/docking/trunk_estimate`).
- Calibration ID / transform validity (from header).
- Selected target state and out-of-view/stale warnings.

### 5.5 Target selection logic
Target behavior differs between simulation (Xavier, with pose exporter) and hardware (Orin, no encoders).

**Simulation mode (`source_mode: simulation`):**
1. On valid click in camera view, read `(u, v)` pixel coordinates.
2. Back-project depth pixel using the depth camera's `K` matrix and the depth value at `(u, v)`.
3. Transform the resulting 3-D point from `camera_optical_frame` to `tree_base` using the latest `pose_at_image` transform from `/harvester/pose_exporter/...`. If the transform is stale or invalid, reject the click and flash a warning.
4. Store `target_tree_base_xyz`.
5. For every subsequent frame, reproject `target_tree_base_xyz` back into the current camera image using current intrinsics and current `T_tree_base_camera`. Draw a crosshair. If the point falls outside the image or the depth is invalid, show `OUT OF VIEW`.
6. Publish target selection as a non-actuating operator annotation event on `v1/operator/target_selection`. No control command is sent to the robot.

**Hardware mode (`source_mode: hardware`):**
1. On valid click in camera view, read `(u, v)` pixel coordinates.
2. Back-project depth pixel using the depth camera's `K` matrix and the depth value at `(u, v)`. If no depth is available, reject the click and warn "NO DEPTH".
3. Store the 3-D point in the camera's current `frame_id` only. Do **not** transform it into `tree_base`, `base_link`, or any world frame because the arm pose is unknown (no encoders).
4. For every subsequent frame, reproject the stored camera-relative point back into the current image using current intrinsics. Draw a crosshair. If the point falls outside the image, show `OUT OF VIEW`.
5. Publish the camera-relative point as a non-actuating operator annotation event on `v1/operator/target_selection`. The event payload includes `frame_id`, `acquisition_timestamp`, and the 3-D coordinates in that frame. No control command is sent to the robot.
6. The HUD shows `ANNOTATION (camera-relative)` instead of a world-fixed target label.
7. **Human-in-the-loop guidance:** The operator uses the on-screen crosshair and relative offset as a visual guide while manually controlling the hydraulic arm via the physical joystick/valves. The dashboard does not command the hydraulic system.

### 5.6 Bounded memory
- Per-channel deque with `maxlen=4` in `subscriber.py`.
- Drain loop in `main.py` using `recv_multipart(flags=zmq.NOBLOCK)` until `zmq.Again`.
- Drop complete old packets; never queue partial frames.

### 5.7 Codec-aware decoder
- **Primary path (Orin hardware):** H.264/H.265 via Jetson hardware decoding. The decoder backend uses `PyNvCodec` (recommended) or `Gst-nvdec` through GStreamer to decode Annex-B byte-stream frames from the canonical binary payload. Hardware decoding offloads the Orin CPU and retains good 720p/15-FPS quality at lower bitrate than MJPEG.
  - Decoder abstraction: `decoder/h264_decoder.py` and `decoder/h265_decoder.py` wrap the hardware path and present a uniform `decode_to_qimage(payload_bytes) -> QImage` interface to QML.
  - Key-frame / SPS/PPS handling: the OAK `VideoEncoder` outputs Annex-B frames including start codes. The decoder must detect and handle IDR/slice boundaries. If the pipeline drops frames, request a key-frame refresh via the control endpoint.
- **Fallback path (Xavier simulation / debug):** JPEG via OpenCV `imdecode`. The Xavier simulation gateway encodes RGB as JPEG; the dashboard decoder falls back to `decoder/jpeg_decoder.py` when `codec` is `"jpeg"`.
- **Future extension:** If DepthAI produces raw NV12 on a future pipeline variant, add an `nv12_decoder.py` that also uses `PyNvCodec` for zero-copy conversion.
- The QML `CameraView` item never sees codec details; it only receives a `QImage` from the decoder pool.

### 5.8 Backend threading and event-loop model
- **Qt event loop owns the UI thread.** `main.py` creates a `QApplication` and a `QQuickView`.
- **ZMQ I/O runs in a background `QThread`.** The subscriber socket and control client socket live in a dedicated `zmq/ZmqWorker` thread. A `QTimer` with 0 ms interval (or `zmq.Poller` + `QSocketNotifier`) drains the socket queue and posts decoded frames to the UI thread via `QMetaObject.invokeMethod` with `Qt.QueuedConnection`.
- **Decoder pool runs in the ZmqWorker thread.** Decoding is CPU-bound; it must not block the UI thread. Each decoded `QImage` is wrapped in a `QPixmap` or passed as a raw buffer and emitted as a signal to the UI thread.
- **Touch/keyboard events originate in QML** and are handled by QML `MouseArea` / `Keys` handlers. They call Python slots via `QObject` bridges exposed through `setContextProperty` or a QML `Python` plugin. Camera switching and target clear commands are sent synchronously or queued to the ZmqWorker thread.
- **No `cv2.imshow` or OpenCV GUI.** The existing `oak_rgb_viewer.py` is replaced entirely. All rendering goes through Qt Quick.

### 5.9 Error handling and recovery
- **Stream offline:** If no frame arrives within `timeout_s` (default 2.0 s), the HUD freshness indicator turns red and the camera view shows a "CAMERA OFFLINE" placeholder. The worker does not crash; it continues draining the queue.
- **Decoder failure:** If `h264_decoder.py` or `h265_decoder.py` returns an error (corrupt Annex-B, missing SPS/PPS, hardware decoder busy), the worker logs the error, increments a counter in `system/status`, and requests a key-frame refresh via the control REQ/REP endpoint. The previous good frame is held for display until a new valid frame arrives.
- **ZMQ queue overflow:** `RCVHWM` is set per socket. When the high-water mark is hit, ZeroMQ drops oldest messages. The worker tracks drop count and publishes it in `v1/system/status`.
- **Adapter restart:** If the adapter process restarts, the ZMQ sockets reconnect automatically (ZeroMQ TCP reconnect). The worker resets sequence-number tracking and flags the stream as `RECONNECTING` until the first new frame arrives.
- **Depth missing on click:** If the operator clicks a pixel with `depth == 0` or `depth == invalid`, the click is rejected with a brief on-screen "NO DEPTH" toast. No annotation is published.
- **Hardware decoder unavailable:** If `PyNvCodec` import fails or the Jetson hardware decoder is not available, the adapter falls back to MJPEG at startup and the header `codec` is set to `"jpeg"`. The dashboard automatically uses `jpeg_decoder.py`. This is logged in `v1/system/status`.

---

## 6. Integration wiring

### 6.1 Xavier side
1. Add `harvester_telemetry_gateway` and `harvester_pose_exporter` as new packages in `~/ros2_ws/src/`.
2. Modify the top-level `launch/gazebo_harvester_and_tree.launch.py` **only** to include new launch files via `IncludeLaunchDescription` (does not modify existing nodes or topics).
3. Existing Gazebo, RViz, and control nodes remain untouched.

### 6.2 Orin side
1. In `harvester_vision` repo:
   - Add `adapter/` and `dashboard/` directories.
   - Evolve `oak_rgb_publisher.py` into `adapter/oak_canonical_publisher.py`.
   - Add `adapter/range_canonical_publisher.py`, `adapter/lidar_canonical_publisher.py`, `adapter/cutter_range_canonical_publisher.py`, `adapter/adapter_status.py`.
   - Add `dashboard/` with Python backend + QML UI.
2. Update `deploy/systemd/oak-rgb-publisher@.service` path and add new systemd units for the full adapter stack.
3. The dashboard subscribes to the local adapter’s PUB endpoints (e.g. `tcp://127.0.0.1:5555` etc., configurable).


### 6.3 Systemd units (Orin)
Add these unit files to `deploy/systemd/`:

- `oak-canonical-publisher@.service` — template for each OAK camera. Replaces `oak-rgb-publisher@.service`. Runs `adapter/oak_canonical_publisher.py --camera-role %i`.
- `harvester-range-publisher.service` — runs `adapter/range_canonical_publisher.py` for the Raspberry Pi docking sensors.
- `harvester-lidar-publisher.service` — runs `adapter/lidar_canonical_publisher.py` for the MID-360.
- `harvester-cutter-range-publisher.service` — runs `adapter/cutter_range_canonical_publisher.py`.
- `harvester-adapter-status.service` — runs `adapter/adapter_status.py`.
- `harvester-dashboard.service` — runs `dashboard/main.py`. This must run in the user session (not as a system service) because it needs desktop/OpenGL access for the Qt Quick window. It depends on `graphical.target`.

All hardware-publisher services have `Restart=always`, `RestartSec=3`, and `After=network-online.target chrony.service`. The dashboard service has no `Restart` (the operator restarts it manually after display sleep/resume).
---

## 7. Validation and acceptance criteria

### 7.1 Xavier validation
- [ ] `colcon test` passes for both new packages.
- [ ] Gateway publishes all canonical channels when Gazebo is running. Run `python3 scripts/validate_gateway.py` to confirm every expected channel appears within 5 s.
- [ ] `python3 scripts/validate_pose_exporter.py` confirms transform validity and freshness.
- [ ] `gz model -l` still shows `oil_palm_harvester` and `oil_palm_tree`.
- [ ] `/robot_description` still has one harvester publisher.
- [ ] Sliders still change Gazebo and RViz harvester.
- [ ] No new static `world -> base_link` transform.
- [ ] **Simulation target test:** With the pose exporter running, clicking a depth camera in the dashboard back-projects a point, transforms it to `tree_base`, and shows it fixed to `tree_base` while the camera/arm moves via sliders.

### 7.2 Orin validation
- [ ] Adapter publishes canonical multipart messages on all configured channels. Run `python3 tests/validate_adapter.py` to confirm every expected channel and codec.
- [ ] REQ/REP endpoint returns valid JSON status.
- [ ] Dashboard camera switching (touch buttons and 1/2 keys) enables/disables the correct OAK pipeline.
- [ ] Dashboard HUD shows `SIMULATION` or `HARDWARE` badge correctly.
- [ ] **Hardware mode:** Clicking a depth camera back-projects a point and stores it as a camera-relative annotation. The dashboard does **not** claim a world-fixed `tree_base` target.
- [ ] **Hardware mode:** Clicking a point shows a persistent on-image crosshair that tracks the camera view; the annotation is labeled `ANNOTATION (camera-relative)`.
- [ ] **Simulation mode (Xavier linked):** Clicking a depth camera back-projects a point, transforms it to `tree_base`, and shows it fixed to `tree_base` while the camera/arm moves.
- [ ] Bounded memory: run with slow subscriber (e.g. `tc` delay or `RCVHWM` hit); confirm no unbounded growth.
- [ ] Raw LiDAR delivery: confirm `v1/lidar/raw` channel carries downsampled XYZ float32.
- [ ] Stale indicators turn red after 2 s of missing data.
- [ ] No Gazebo/RViz control regression on Xavier.
- [ ] `v1/range/cutter` is active on Orin and published alongside docking ranges.
- [ ] **No actuation:** Confirm that the dashboard, adapter, and gateway do not publish any message on any topic or port that could be interpreted as a joint command, hydraulic valve command, PLC write, or motion command. The only outbound command path is the camera enable/disable control endpoint.

---

## 8. Delivery order

1. Freeze schema and write `docs/canonical_zmq_v1.md`.
2. Build `harvester_telemetry_gateway` and validate against live Gazebo topics.
3. Enable opt-in exact-packet recording in the Xavier gateway. Each record
   stores the original three binary multipart frames in MessagePack
   (`record_format_version`, `recorded_monotonic_ns`, `frames`), never
   JSON/hex. Add the same recorder to every Orin adapter source so real OAK,
   LiDAR, and range data can be audited offline. Build
   `harvester_vision/tests/record_replay.py` around this shared format; it
   replays stored packets over a separate local PUB endpoint and verifies that
   the adapter and dashboard parse every codec without crashing.
4. Build `harvester_pose_exporter` and validate.
5. Build Orin `adapter/` and replace `oak_rgb_publisher.py` with canonical publisher.
6. Build Orin `dashboard/` Qt Quick app with camera switcher (touch + keyboard), HUD, sensor panel, and LiDAR inset.
7. Enable persistent 3-D target selection.
8. End-to-end integration and acceptance test.

---

## Out of scope — Phase 2 (Eye-in-Hand Human-in-the-Loop Guidance)

**Phase:** 2 of 2 — Eye-in-Hand (Eye-on-Tool) passive guidance and target tracking.
**Scope boundary:** This phase adds a cutting-tool-mounted sensor suite, 3-stage target tracking, and enhanced dashboard guidance overlays. It remains strictly non-actuating: no hydraulic valve commands, no solenoid control, no PLC writes, no joint commands. The human operator retains full manual control of the hydraulic arm.

### Phase 2.1 — Sensor architecture: Eye-in-Hand

The cutting tool arm carries three sensing modalities:

1. **Tool-mounted depth camera** — replaces/ supplements the existing arm-mounted `platform_depth_camera_link`. In simulation, this is a second Gazebo depth camera parented to `cutting_tool_link`. On Orin hardware, this is a second OAK device or a stereo pair mounted directly on the cutter head.
2. **Tool-mounted LiDAR** — a second, smaller LiDAR (e.g. Livox Mid-40 or Ouster) mounted on `cutting_tool_link`. On Orin hardware, this may be the same MID-360 relocated or a dedicated tool-LiDAR. In simulation, it is a second Gazebo LiDAR sensor on the cutter link.
3. **Dual IMUs** — one IMU on the OAK camera (`platform_depth_camera_link`) and one IMU on the LiDAR (`vehicle_lidar_link` / tool LiDAR). Both publish `sensor_msgs/Imu` at 200–500 Hz. In simulation, these are Gazebo IMU plugins on the respective links.

**Why Eye-in-Hand:** With sensors mounted on the moving cutter, the target stays in view during approach. The relative error vector `e(t) = P_target^C(t) − P_crosshair^C` drives the operator’s visual guidance. No global FK is needed for the operator to see where the crosshair sits relative to the target.

**Existing simulation reuse:** The Gazebo kinematic plugin already accepts joint commands on `/harvester/joint_commands`. Phase 2 simulation uses those same joints to drive the arm in Gazebo while the new Eye-in-Hand sensors provide the tracking inputs. No Gazebo controller changes are required.

### Phase 2.2 — 3-stage target tracking and handoff

| Stage | Distance | Primary sensor | Secondary sensor | Goal |
|---|---|---|---|---|
| Far | > 1.0 m | Tool LiDAR + tool depth camera | Base-mounted LiDAR (existing) | Coarse target lock and 3-D initialization |
| Mid | 0.3 m – 1.0 m | IMU-stabilized tool camera | Tool LiDAR (sparse) | Crosshair precision and motion-blur compensation |
| Terminal | < 0.3 m | Tool cutter-range sensors (micro-ToF / short-range laser) | Tool depth camera (if not blind) | Final engagement and contact confirmation |

**Handoff logic:**
- The dashboard publishes a single `v1/operator/target_lock` status stream with fields: `stage`, `distance_m`, `primary_sensor`, `tracking_quality`, `lock_status`.
- Distance is estimated from the tool depth camera’s disparity or the tool LiDAR’s closest valid range.
- Stage transitions are automatic but logged. The operator sees the current stage in the HUD.

### Phase 2.3 — Target tracking pipeline

**Initialization (operator click):**
1. Operator taps the target point (FFB stalk/frond base) on the dashboard.
2. The dashboard back-projects the depth pixel at `(u, v)` using the tool depth camera’s `K` matrix and depth value.
3. The tool LiDAR’s current cloud is searched for the nearest point to the back-projected 3-D point within a radius (e.g. 5 cm). If found, the LiDAR point refines the target coordinate.
4. The initial 3-D target `P_target^C` is stored in the tool camera frame `C`.
5. The tool crosshair `P_crosshair^C` is a fixed offset in the tool camera frame (measured once at URDF calibration, e.g. `(0, 0, −0.15)` for 15 cm ahead of the blade tip in the camera’s +Z).

**Tracking across frames:**
1. **Optical flow:** OpenCV `calcOpticalFlowPyrLK` tracks the target pixel `(u, v)` from frame to frame using the tool depth camera RGB stream.
2. **LiDAR association:** Each new LiDAR scan is deskewed using the IMU (see Phase 2.4). The nearest LiDAR point to the predicted target location updates the depth value.
3. **Depth-gated flow:** If optical flow loses lock (tracking quality < threshold), the LiDAR nearest-neighbour search reinitializes the pixel coordinate in the next frame.
4. **IMU dead-reckoning:** During brief occlusions, the IMU propagates the target’s relative 3-D position forward at IMU rate. When vision recovers, the tracked position is reset to the IMU-predicted location.

**Dashboard display:**
- A persistent crosshair overlay marks `P_crosshair^C` at the fixed pixel offset.
- A tracked target dot marks `P_target^C(t)`, updated every frame.
- A vector arrow shows the relative error `e(t) = [ΔX, ΔY, ΔZ]`.
- The HUD shows: `ΔX`, `ΔY`, `ΔZ` in cm; tracking stage; lock quality; distance to target.

### Phase 2.4 — IMU-based motion compensation

**Point-cloud deskewing (tool LiDAR):**
- The tool LiDAR IMU publishes orientation at ≥ 200 Hz.
- During a single LiDAR scan (e.g. 10 Hz, 100 ms), the arm moves. Each laser point is timestamped by the LiDAR driver. The IMU orientation at each point’s timestamp is interpolated and used to rotate the point from the sensor frame at capture time to the sensor frame at scan start.
- Result: a deskewed point cloud in the tool LiDAR frame.

**Camera motion deblurring (tool depth camera):**
- The tool depth camera IMU publishes linear acceleration and angular velocity.
- Short exposure times (OAK default) already reduce motion blur. If blur is detected (Laplacian variance below threshold), the IMU is used to predict the per-pixel motion and apply a rolling-shutter correction before optical flow.
- The dashboard HUD shows a `MOTION` warning when the IMU-detected jerk exceeds a threshold.

**Vibration filtering:**
- A high-pass filter on the IMU accelerometer removes structural whip (> 20 Hz). The filtered linear acceleration integrates to velocity for dead-reckoning between vision updates.

### Phase 2.5 — Terminal engagement (< 30 cm)

**Cutter-range sensor handoff:**
- When `distance_m ≤ 0.30 m` (configurable), the tracking stage switches to `TERMINAL`.
- The tool’s micro-range sensors (existing hardware, 20–30 cm range) take over depth measurement.
- Multiple range sensors across the cutting edge measure distance and tilt angle relative to the stalk.
- The dashboard shows: blade-to-stalk distance per sensor, tilt angle, and `CONTACT READY` when all sensors agree within tolerance.
- If any range sensor reports `distance_m > 0.30 m` during terminal stage, the system flags `TARGET LOST — RETREAT`.

**Blind-zone handling:**
- Below ~20 cm, the depth camera’s disparity may saturate. The dashboard falls back to range-sensor-only depth and freezes the last valid depth-camera pixel coordinate.
- The crosshair remains visible; only the depth update source changes.

### Phase 2.6 — Software architecture

**New ROS 2 packages (Xavier):**
- `harvester_tool_tracking` — subscribes to tool camera, tool LiDAR, tool IMUs, cutter-range sensors. Publishes `v1/operator/target_lock` and `v1/tool/target_track`.
- `harvester_imu_deskew` — subscribes to tool LiDAR raw cloud + tool LiDAR IMU. Publishes deskewed cloud on `v1/lidar/tool_raw`.
- `harvester_range_fusion` — subscribes to cutter-range sensors. Publishes `v1/range/cutter` (already exists in Phase 1) plus a `terminal_lock` status.

**New Orin adapter modules:**
- `adapter/tool_camera_canonical_publisher.py` — second OAK or stereo pair on `cutting_tool_link`.
- `adapter/tool_lidar_canonical_publisher.py` — tool LiDAR UDP parser + deskewed cloud publisher.
- `adapter/tool_imu_publisher.py` — IMU topics from OAK and tool LiDAR into canonical `v1/imu/...` channels (new channels, schema v1 compatible).

**Dashboard changes:**
- QML `CrosshairOverlay.qml` — fixed crosshair at calibrated tool-camera offset.
- QML `TargetTrackOverlay.qml` — tracked target dot + error vector arrow.
- QML `StageIndicator.qml` — current tracking stage (FAR / MID / TERMINAL).
- Backend `tracking_engine.py` — optical flow + LiDAR association + IMU dead-reckoning.
- Backend `handoff_controller.py` — stage transitions based on distance and lock quality.

**Canonical ZeroMQ v1 additions (frozen, additive):**
- `v1/camera/tool/rgb` — tool depth camera RGB (H.264/H.265 or JPEG).
- `v1/camera/tool/depth` — tool depth camera depth (`depth_uint16_le`).
- `v1/camera/tool/camera_info` — tool camera intrinsics.
- `v1/lidar/tool_raw` — deskewed tool LiDAR cloud (`lidar_xyz_f32`).
- `v1/imu/tool_camera` — tool camera IMU (`json` with orientation/angular_velocity/linear_acceleration at IMU rate).
- `v1/imu/tool_lidar` — tool LiDAR IMU (same schema).
- `v1/operator/target_lock` — tracking status, stage, distance, quality.

### Phase 2.7 — Safety and validation

**Safety boundaries (unchanged from Phase 1):**
- No outbound command to hydraulic valves, solenoids, or PLC.
- No joint command publication.
- The dashboard is a display and annotation tool only.
- Emergency stop is external to this software (operator’s physical E-stop on the hydraulic panel).

**New failure modes to handle:**
- **Target occlusion:** Fronds or debris block the target. IMU dead-reckoning holds the predicted position for up to 0.5 s. If vision does not recover, the HUD shows `TRACK LOST` and the operator must reinitialize.
- **Tool sensor blind zone:** Below 20 cm, depth camera saturates. Range sensors take over. If range sensors also fail (e.g. sap on lens), the HUD shows `SENSOR BLIND — MANUAL ENGAGE`.
- **LiDAR saturation:** Dense foliage causes multiple returns. The deskewed cloud is filtered by radius around the predicted target location before nearest-neighbour search.
- **IMU dropout:** If one IMU fails, the other continues deskewing. If both fail, the system stays in `MID` stage with a `NO IMU` warning; tracking continues with vision-only optical flow.

**Validation:**
- Gazebo simulation with Eye-in-Hand sensors mounted on `cutting_tool_link`.
- Validate 3-stage handoff by moving the simulated arm through a scripted approach trajectory.
- Validate IMU deskewing by comparing deskewed tool LiDAR cloud against a static-target ground truth.
- Validate optical flow + LiDAR tracking against simulated target motion with known ground-truth pose.
- Validate terminal-range handoff by driving the simulated cutter into contact with the tree trunk.
- Confirm zero outbound actuation messages: run Wireshark/tcpdump on Orin during simulation; verify no packet leaves the Orin on any port except canonical PUB and control PULL/REQ.

### Phase 2.8 — Delivery order

1. Freeze additive canonical ZMQ v1 schema extensions (`v1/camera/tool/*`, `v1/lidar/tool_raw`, `v1/imu/*`, `v1/operator/target_lock`).
2. Add Eye-in-Hand sensors to the Gazebo URDF (`oil_palm_harvester_kinematic.urdf`) and launch file. Validate raw topics in RViz.
3. Build `harvester_imu_deskew` and validate deskewed tool LiDAR against static target.
4. Build `harvester_tool_tracking` with optical flow + LiDAR association + IMU dead-reckoning.
5. Build `harvester_range_fusion` and validate terminal-range handoff at < 30 cm.
6. Update Orin `adapter/` with tool camera, tool LiDAR, and IMU publishers.
7. Update dashboard with crosshair overlay, target track overlay, stage indicator, and error vector display.
8. End-to-end simulation validation: scripted approach, 3-stage handoff, zero actuation audit.

---

## 9. Open questions / decisions for implementer

1. **Qt binding:** PySide2 vs PySide6. Recommend PySide2 because the existing OpenCV viewer already relies on Qt plugins and PySide2 is more widely packaged on Ubuntu 20.04/Jetson L4T. If PySide6 is already installed, use it.
2. **LiDAR driver on Orin:** The MID-360 UDP protocol is not yet in `harvester_vision`. The implementer must choose between adding a Livox SDK dependency or a lightweight UDP parser. Recommend lightweight parser to keep CPU low.
3. **Target event bus:** The spec says "non-actuating operator annotation event." Recommend a local ZMQ PUB `v1/operator/target_selection` on Orin. If a ROS 2 bridge is needed later, it can subscribe to this ZMQ topic.
