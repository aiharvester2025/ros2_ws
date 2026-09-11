# Recording format + sweep procedure

How to sweep the arm, record a LiDAR scan, and replay it offline for height estimation.

## The canonical recording format (from `harvester_telemetry_gateway`)

Each recording file is MessagePack of `{'frames': [channel, header, payload], 'version': 1}`, where:
- `channel` is a UTF-8 string (e.g. `v1/lidar/raw`).
- `header` is a MessagePack dict with `point_count`, `frame_id`, `point_fields` (`lidar_xyz_f32`), etc.
- `payload` is `point_count * 12` bytes of XYZ float32 triples.

Decode with the existing helpers in `harvester_telemetry_gateway.recording`
(`load_recording` / `iter_recordings`) and `harvester_telemetry_contract.protocol.unpack_message`.

Directory naming uses **underscores**: `v1_lidar_raw`, `v1_range_docking`, `v1_range_cutter`, etc.
(not `v1/lidar/raw` — that was an early wrong assumption).

## Frame subtlety (caused real confusion)

The gateway has two LiDAR config knobs:
- `lidar_level_translation: true` → points in absolute **world** frame (tree at x≈8.5, z up to 12.2).
- `lidar_level_translation: false` (default) → **rotation-only leveling**: LiDAR stays at origin,
  points gravity-aligned, translation dropped.

`frame_id` in the header may still read `vehicle_lidar_link` even when points are leveled — do not
trust `frame_id` alone; check the actual point coordinates. When `_lidar_in_world` returns None (TF
unavailable at that instant), the raw sensor-frame cloud is recorded. Handle both: filter files by
whether the trunk cluster appears at x≈8.5 (world) or not.

## Sweep procedure

The sweeper moves **only `cutting_arm_lift_joint`** from `-0.35` rad (down) up to `+1.05` rad
(canopy), ~40 steps, capturing one LiDAR scan per step. `tree_scan_sweeper.py` lives in
`src/oil_palm_harvester_description/scripts/`.

### LiDAR FOV / coverage (research finding — how many passes are needed)

- Cutting-arm LiDAR vertical FOV is **−7°…+52° (59°)**, horizontal ±60° (120°).
- At the ~6.6 m standoff, the trunk (0.5–0.7 m diameter, 12 m tall) subtends ~4° horizontally × ~62°
  vertically from the LiDAR.
- **One correctly-aimed nod (3–5 steps) spans the whole trunk** — the 59° vertical FOV already covers
  the 62° trunk height. A full −20°…+60° sweep overshoots and adds almost no trunk points.
- The real Livox Mid-360 emits 200,000 pts/s **non-repetitive** (vs the Gazebo grid's ~68,480 rays/s)
  and **densifies with dwell**, so far fewer passes are needed on hardware than in the sparse grid.

### Critical operational pitfall: the joint_state_publisher_gui

`joint_state_publisher_gui` **owns** `/harvester/joint_commands` and publishes its slider values at
~10 Hz, continuously overriding any sweeper command — so the arm appears "not moving" and the lift
joint stays at its GUI default (~-0.017 rad). The fix is to **kill the GUI process** so the sweeper
is the sole publisher. Diagnose with `ros2 topic info /harvester/joint_commands` (shows GUI as a
publisher) and `ros2 topic echo /harvester/joint_commands`.

### Stale ROS 2 daemon

A leftover `_ros2_daemon` from a prior session can partition the DDS data-plane: new nodes see the
graph but can't exchange messages (e.g. `/harvester/joint_states` never arrives, so a
feedback-waiting sweeper hangs). Kill the stale daemon and restart.

## Run steps (three terminals)

```bash
# Terminal 1 — simulation
source /opt/ros/foxy/setup.bash && source install/setup.bash
ros2 launch oil_palm_harvester_description gazebo_harvester_and_tree.launch.py \
  harvester_collision_mode:=off articulation_control_mode:=kinematic

# Terminal 2 — recording gateway (set record_dir in config)
ros2 launch harvester_telemetry_gateway gateway.launch.py config:=<your_record_config.yaml>

# Terminal 3 — sweeper (ROS-sourced)
python3 src/oil_palm_harvester_description/scripts/tree_scan_sweeper.py
```

## Offline analysis

```bash
python3 analyze_tree_scan_v2.py ~/harvester_audits/tree_scan_001
```

`tree_scan_001` yielded ~1924 world-frame clouds (~1.1 M points); crown base ~9.0–9.5 m (GT 9.2),
canopy top 99th pct 12.26 m (GT 12.0). The "ground" reads ~1.84 m due to occlusion — expected and
harmless for the height *difference*.

## IMU recordings

IMUs (`v1/imu/lidar`, `v1/imu/camera`) were added to the URDF/gateway/contract later. The older
`tree_scan_001` recording has **no** `v1_imu_lidar` directory, so the IMU-assisted Strategy 2 cannot
be evaluated against it — a fresh recording is required to exercise that path.
