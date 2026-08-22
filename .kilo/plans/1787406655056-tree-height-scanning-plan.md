# Automated Tree Scan & Height Estimation — Execution Plan

## Goal
Run an end-to-end automated workflow: launch simulation, sweep arm, record LiDAR via gateway, replay offline, estimate trunk top, and compare to known 12.0 m ground truth.

## Prerequisite: Create `tree_scan_sweeper.py`
**File**: `src/oil_palm_harvester_description/scripts/tree_scan_sweeper.py`

The sweeper is a ROS 2 node that:
- Publishes `/harvester/joint_commands` with fixed joints + sweeping `cutting_arm_lift_joint`
- Waits for convergence on `/harvester/joint_states`
- Captures one `/harvester/lidar/raw_points` per step
- Writes merged cloud to `/tmp/tree_scan_merged.ply`
- Publishes merged cloud on `/harvester/tree/merged_cloud`

Fixed configuration:
```yaml
boom_elevation_joint: 0.785
boom_extension_1..4_joint: 1.5
rail_carriage_joint: 0.0
cutting_arm_extension_joint: 0.0
platform_level_joint: 0.0
boom_turret_joint: 0.0
```
Sweep: `cutting_arm_lift_joint` from -0.35 to +1.05 rad in 40 steps.

## Execution Steps

### Step 1: Enable gateway recording
```bash
mkdir -p ~/harvester_audits/tree_scan_001
cp ~/ros2_ws/src/harvester_telemetry_gateway/config/gateway.yaml \
   ~/harvester_audits/tree_scan_gateway.yaml
```
Edit `~/harvester_audits/tree_scan_gateway.yaml`:
```yaml
record_dir: /home/ubuntu/harvester_audits/tree_scan_001
```

### Step 2: Launch simulation + recording gateway
```bash
# Terminal 1
cd ~/ros2_ws
source /opt/ros/foxy/setup.bash
source install/setup.bash
ros2 launch oil_palm_harvester_description gazebo_harvester_and_tree.launch.py \
  harvester_collision_mode:=off articulation_control_mode:=kinematic

# Terminal 2
cd ~/ros2_ws
source /opt/ros/foxy/setup.bash
source install/setup.bash
ros2 launch harvester_telemetry_gateway gateway.launch.py \
  config:=/home/ubuntu/harvester_audits/tree_scan_gateway.yaml
```

Verify: `ls ~/harvester_audits/tree_scan_001/v1/lidar/raw/` should be empty initially.

### Step 3: Run sweeper
```bash
# Terminal 3
cd ~/ros2_ws
source /opt/ros/foxy/setup.bash
source install/setup.bash
python3 src/oil_palm_harvester_description/scripts/tree_scan_sweeper.py
```

Expected duration: ~80–100 s (40 steps × 2 s convergence).

### Step 4: Stop and verify recording
Press `Ctrl-C` on Terminals 1, 2, 3.

Verify:
```bash
ls -lh ~/harvester_audits/tree_scan_001/v1/lidar/raw/
```
Expect ~40 MessagePack files.

### Step 5: Offline replay + analysis script
**File**: `/tmp/analyze_tree_scan.py`

This script:
1. Reads all `v1/lidar/raw/*.msgpack` from the audit directory
2. Decodes canonical `lidar_xyz_f32` packets (reuses `harvester_telemetry_gateway.encoders`)
3. Merges into a single `Nx3` numpy array in `world` frame
4. Transforms to `tree_base` frame (world offset: x=8.5, y=0, z=0)
5. Downsample (voxel 0.05 m), remove ground (z > 0.5 m)
6. Extract trunk: points within r < 0.8 m of (0, 0) in `tree_base` XY
7. Histogram along `tree_base` z with 0.1 m bins
8. Detect trunk top: highest bin with ≥ 20 points and radius 0.25–0.35 m
9. Compute docking point: `trunk_top_z - 1.0` clamped to [2.0, trunk_top_z - 0.3]
10. Print results and save `/tmp/tree_scan_result.json`

Run:
```bash
cd ~/ros2_ws
PYTHONPATH=src/harvester_telemetry_contract:src/harvester_telemetry_gateway \
  python3 /tmp/analyze_tree_scan.py ~/harvester_audits/tree_scan_001
```

### Step 6: Validation
- Ground truth trunk top: **12.0 m**
- Accept if estimated within **±1.0 m**
- Reject and retry with adjusted `boom_extension_*` or `rail_carriage_joint` if:
  - Trunk points < 5000
  - Estimated height < 10.0 m or > 13.5 m
  - No clear density drop detected

## Failure Recovery
| Symptom | Fix |
|---|---|
| No `lidar/raw` files recorded | Gateway config missing `record_dir`; restart gateway with correct config |
| < 5000 trunk points | Increase `boom_extension_*` to 1.8 m or add `rail_carriage_joint` sub-steps |
| Trunk top < 10.0 m | Arm too low; increase `boom_elevation_joint` to 1.0 rad |
| Trunk top > 13.0 m | Crown branches misclassified; apply RANSAC cylinder on lower 75% |
| Convergence timeouts | Increase `convergence_timeout` to 3.0 s in sweeper |

## Out of Scope
- Physical cutting / FFB detection
- Force-accurate vehicle dynamics
- Online real-time scanning during approach
- Orin hardware adapter
