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

## Key findings

- The LiDAR point cloud cleanly resolves the tree at `world (8.5, 0, 0)`.
- The trunk is a dense, uniform vertical cylinder from ground to ~9.2 m.
- The crown base (where the density drops and fronds/branches attach) is
  measured at ~9.0–9.5 m (ground truth: 9.2 m).
- The canopy top is measured at ~12.0 m (99th percentile 12.26 m), matching
  the `trunk_top_reference` ground truth of 12.0 m.

| Metric | LiDAR measurement | Ground truth |
|---|---|---|
| Crown base (trunk→canopy) | ~9.0–9.5 m | 9.2 m |
| Canopy top (99th pct) | 12.26 m | 12.0 m |
| Canopy top (max, frond tips) | 13.17 m | — |
| Total tree height | ~12 m | 12.0 m |

The LiDAR's downward view is occluded near the trunk base (the harvester/arm
blocks it), so the measured "ground" is ~1.84 m rather than 0.0 m. This does
not affect the height *difference*.

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
