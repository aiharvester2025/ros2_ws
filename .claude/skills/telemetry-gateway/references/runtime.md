# Gateway runtime: run / record / replay quirks

## Run commands

```bash
# Terminal 1 — Gazebo + RViz
source /opt/ros/foxy/setup.bash && source install/setup.bash
ros2 launch oil_palm_harvester_description gazebo_harvester_and_tree.launch.py \
  harvester_collision_mode:=off articulation_control_mode:=kinematic

# Terminal 2 — gateway (read-only, binds 5590 PUB + 5600 REP)
ros2 launch harvester_telemetry_gateway gateway.launch.py
```

## Recording (audit capture)

Recording is **opt-in** (`record_dir: ""` default). To enable:

```bash
cp src/harvester_telemetry_gateway/config/gateway.yaml ~/harvester_audits/my_scan.yaml
# edit: record_dir: /home/ubuntu/harvester_audits/my_scan
ros2 launch harvester_telemetry_gateway gateway.launch.py config:=~/harvester_audits/my_scan.yaml
```

Recorded directories use **underscore names**: `v1_lidar_raw`, `v1_range_docking`, `v1_range_cutter`
(not `v1/lidar/raw`). Each file is one MessagePack `{'frames': [channel, header, payload],
'version': 1}`.

## Replay

```bash
python3 -m harvester_telemetry_gateway.replay <dir> --endpoint tcp://*:5591   # anaconda python
# --max-gap-s 1.0 to slow pacing for UI testing (default caps inter-packet gap at 0.25 s)
```

Replay binds 5591 so it doesn't collide with the live gateway (5590) or status REP (5600).

## Runtime quirks (do not "fix" without cause)

1. **Interpreter split.** The gateway runs under the **active (anaconda) python** (has zmq/msgpack).
   ROS entry points select `/usr/bin/python3`, which lacks them. This is deliberate — the launch file
   invokes the active `python3`. Keep it.

2. **`flush_one_packet()`** scans channel queues alphabetically and returns after the first non-empty
   one (newest-wins, clears the rest as drops). A persistent high-rate channel sorting early (e.g.
   `v1/calibration/status`) could starve later channels at high load. Fine at current ~100 pkt/s vs
   500 flushes/s. Do not change gateway code for this.

3. **`on_docking_range`** republishes all five records on every single-sensor update (~100 msgs/s on
   `v1/range/docking`). Consumers overwrite state, don't accumulate.

4. **REP handler** services one request per 50 ms tick — acceptable for a read-only status probe.

## LiDAR leveling config (interacts with the dashboard HUD)

```yaml
lidar_world_frame: world
lidar_transform_latest: true      # matches wall-time TF convention (see lidar_hud_leveling skill)
lidar_level_translation: false    # rotation-only: LiDAR stays at origin, tree leveled (HUD-friendly)
lidar_stride: 1
```

- `lidar_transform_latest: true` uses the current arm pose at processing time (correct for the
  wall-time-TF interactive sim). Flip to `false` only when TF and the sensor share simulation time.
- `lidar_level_translation: false` produces a "leveled sensor frame" (rotation only, LiDAR at origin)
  which is what the sensor-relative HUD needs. `true` moves points to absolute world coords (tree at
  x≈8.5), which pushed the tree to the HUD edge and broke the left/right/iso views.

## Build

```bash
colcon build --packages-select harvester_telemetry_gateway harvester_telemetry_contract \
  --merge-install --symlink-install
source install/setup.bash
```

Use `--merge-install` — this workspace's `install/` was created with the merged layout and colcon
errors without it.

## Testing

Contract/encoder/recording tests are pure Python:
```bash
PYTHONPATH=src/harvester_telemetry_contract:src/harvester_telemetry_gateway \
  python3 -m unittest discover -s src/harvester_telemetry_contract/test
```

Gateway tests that import `sensor_msgs` must run under the ROS Foxy env (anaconda python lacks it):
```bash
source /opt/ros/foxy/setup.bash
PYTHONPATH=src/harvester_telemetry_gateway python3 -m unittest discover -s src/harvester_telemetry_gateway/test
```
