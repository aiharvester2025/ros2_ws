# Tree Height Measurement via Arm-Mounted LiDAR Sweep

**Date:** 2026-08-23
**Workspace:** `/home/ubuntu/ros2_ws`
**Status:** Validated in simulation (Gazebo Classic 11 + ROS 2 Foxy)

## Goal

Estimate the oil-palm tree height and locate the trunk-to-crown transition by
sweeping only the `cutting_arm_lift_joint` while the arm-mounted LiDAR scans
the tree from ground to canopy. The transition height is the reference for
placing the c-channel platform below the crown for optimum branch/FFB
reachability.

## Approach

1. Hold every other joint fixed at its initial (centered) position.
2. Sweep only `cutting_arm_lift_joint` from `-0.35 rad` (down/ground) to
   `+1.05 rad` (up/canopy) in 40 steps.
3. The telemetry gateway records every LiDAR scan as a canonical
   `v1/lidar/raw` packet in the **world frame** (full translation + rotation).
4. Offline analysis merges all clouds, isolates the trunk cylinder, and
   detects the trunk-to-crown density transition.

## How to run (scan + height estimation)

Three phases: (1) prepare recording, (2) run the sweep while recording,
(3) analyze the recording offline.

### Prerequisites

The sweep script and gateway config already exist:

- `src/oil_palm_harvester_description/scripts/tree_scan_sweeper.py`
- `~/harvester_audits/tree_scan_gateway.yaml`

Build (only if the sweep script changed):

```bash
cd ~/ros2_ws
source /opt/ros/foxy/setup.bash
colcon build --packages-select oil_palm_harvester_description --symlink-install
source install/setup.bash
```

### Phase 1 — prepare the recording config

```bash
mkdir -p ~/harvester_audits/tree_scan_001
cp ~/ros2_ws/src/harvester_telemetry_gateway/config/gateway.yaml \
   ~/harvester_audits/tree_scan_gateway.yaml
```

Edit `~/harvester_audits/tree_scan_gateway.yaml` and set:

```yaml
lidar_stride: 1              # full 107x64 resolution
lidar_level_translation: true  # true world coordinates (tree at x=8.5)
record_dir: "/home/ubuntu/harvester_audits/tree_scan_001"
```

### Phase 2 — run the sweep while recording

Start the simulation (Terminal 1):

```bash
cd ~/ros2_ws
source /opt/ros/foxy/setup.bash
source install/setup.bash
ros2 launch oil_palm_harvester_description gazebo_harvester_and_tree.launch.py \
  harvester_collision_mode:=off articulation_control_mode:=kinematic
```

Wait for the log line `Harvester kinematic bridge ready`, then **stop the GUI**
so it cannot override the sweep targets (Terminal 2):

```bash
pkill -f joint_state_publisher_gui
```

Start the recording gateway (Terminal 2):

```bash
cd ~/ros2_ws
source /opt/ros/foxy/setup.bash
source install/setup.bash
ros2 launch harvester_telemetry_gateway gateway.launch.py \
  config:=/home/ubuntu/harvester_audits/tree_scan_gateway.yaml
```

Confirm recording started (`Canonical packet recording enabled: ...`), then run
the sweep (Terminal 3):

```bash
cd ~/ros2_ws
source /opt/ros/foxy/setup.bash
source install/setup.bash
python3 src/oil_palm_harvester_description/scripts/tree_scan_sweeper.py
```

The sweeper moves only `cutting_arm_lift_joint` from `-0.35 rad` (ground) up to
`+1.05 rad` (canopy) in 40 steps (~2–3 min). It logs `captured N LiDAR msgs` at
each step. Stop the gateway (`Ctrl-C`) when the sweep prints `Lift-only sweep
complete.`.

### Phase 3 — analyze the recording offline

```bash
cd ~/ros2_ws
PYTHONPATH=src/harvester_telemetry_contract:src/harvester_telemetry_gateway \
  python3 analyze_tree_scan.py ~/harvester_audits/tree_scan_001
```

The analysis merges all world-frame clouds, isolates the trunk cylinder
(`r < 0.45 m` around `world (8.5, 0, 0)`), and reports the trunk-to-crown
transition and total tree height. Results are also written to
`/tmp/tree_scan_result.json`.

### Troubleshooting

| Symptom | Cause / fix |
|---|---|
| Sweep logs `no LiDAR msg captured` every step, but topics exist | Stale `_ros2_daemon` broke DDS data-plane. `pkill -9 -f _ros2_daemon`, restart the whole stack. |
| Joint does not move (stays near zero) | `joint_state_publisher_gui` still publishing; `pkill -f joint_state_publisher_gui`. |
| Recording empty / no `v1_lidar_raw` files | Gateway `record_dir` unset; re-check `~/harvester_audits/tree_scan_gateway.yaml`. |
| Gazebo exits 255 | Port 11345 held by an old session; stop the previous launch first. |

## Key findings

- The LiDAR point cloud cleanly resolves the tree at `world (8.5, 0, 0)`.
- The trunk is a dense, uniform vertical cylinder from ground to ~9.2 m.
- The crown base (where the canopy annulus becomes densely populated) is
  measured at **9.0 m** (ground truth: 9.2 m).
- The canopy top (99th percentile) is measured at **12.26 m** (ground truth:
  `trunk_top_reference` 12.0 m).

| Metric | LiDAR measurement | Ground truth |
|---|---|---|
| Crown base (trunk→canopy) | 9.00 m | 9.2 m |
| Canopy top (99th pct) | 12.26 m | 12.0 m |
| Canopy top (max, frond tips) | 13.17 m | — |
| Total tree height | 12.26 m | 12.0 m |

The LiDAR's downward view is occluded near the trunk base (the harvester/arm
blocks it), so the lowest *measured* trunk point is ~1.84 m rather than 0.0 m.
Total height is therefore computed from the known `world z=0` tree base, not
the occluded LiDAR "ground"; the height *difference* (crown base − trunk base)
is unaffected.

## Operational details learned

### Sweeping only one joint

The Gazebo kinematic bridge (`harvester_kinematic_gazebo_bridge`) updates only
the joints named in each `/harvester/joint_commands` message. Publishing a
`JointState` containing **only** `cutting_arm_lift_joint` therefore moves just
that joint and leaves the boom/turret/platform untouched.

### The GUI overrides commands

`joint_state_publisher_gui` publishes `/harvester/joint_commands` continuously
(~10 Hz) with its default slider values. It must be stopped before a scripted
sweep, otherwise it overwrites the sweep targets.

```bash
pkill -f joint_state_publisher_gui
```

### Stale ROS daemon breaks DDS data-plane

A stale `_ros2_daemon` from a previous session caused a DDS data-plane
partition: new nodes could see the topic graph (`ros2 node list` worked) but
could not send/receive actual messages. Symptom: `ros2 topic echo` returned
nothing while topics listed publishers. Fix: kill the stale daemon and restart.

```bash
pkill -9 -f _ros2_daemon
```

## Recorded artifacts

- LiDAR recordings: `~/harvester_audits/tree_scan_001/v1_lidar_raw/*.msgpack`
  (1924 world-frame clouds, ~1.1M points).
- Analysis summary: `/tmp/tree_scan_result.json`.
- Sweep script: `src/oil_palm_harvester_description/scripts/tree_scan_sweeper.py`.
- Gateway recording config: `~/harvester_audits/tree_scan_gateway.yaml`
  (`lidar_stride: 1`, `lidar_level_translation: true`, `record_dir` set).

## Next steps

1. Promote the offline analysis into a ROS node that publishes the docking
   point (`trunk_top - offset`) as a `geometry_msgs/PointStamped` on
   `/harvester/tree/docking_estimate`.
2. Add the trunk-top estimate to the dashboard via a new ZMQ channel or ROS
   topic.
3. Validate the docking-point offset against the c-channel platform
   reachability envelope.
