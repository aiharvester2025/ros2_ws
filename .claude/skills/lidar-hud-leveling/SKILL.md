---
name: lidar-hud-leveling
description: "Diagnose and fix LiDAR point-cloud frame/orientation issues in the harvester HUD — especially the 'tree bending/leaning' artifact and world-frame vs sensor-frame vs leveled-sensor-frame transforms."
metadata:
  author: ros2_ws
  version: "1.0.0"
  status: stable
---

# LiDAR HUD Leveling & Frame Correctness

Diagnose and fix why the LiDAR point cloud in the HUD leans/bends when the arm or platform moves,
and understand the three frames (sensor, world, leveled-sensor) the telemetry pipeline can emit.

This skill records the **root-cause analysis and the frame model** that a smaller model would
otherwise re-derive, plus the sensor-inventory facts that constrain the solution on real hardware.

## When this applies

- "The tree looks tilted/bent when I move the arm/platform" in the LiDAR HUD.
- Editing the LiDAR transform logic in `harvester_telemetry_gateway/gateway_node.py`
  (`_lidar_in_world`) or the dashboard's `projection.py` / `bridge.py` / `LidarInset.qml`.
- Deciding how to level a point cloud on the Orin (hardware) side.

## Core facts (internalize before editing)

<critical>
- **The "bending tree" is a frame artifact, not real geometry.** The HUD was plotting points in the
  rotating **sensor frame** (`vehicle_lidar_link`); the side view's "up" was the *sensor's* up, not
  the world's. Pitching up tips the sensor's +z backward → tree appears to "bend back"; pitching
  down realigns → tree "straightens."
- **The fix is a "leveled sensor frame": rotation-only leveling (undo pitch/roll), LiDAR stays at the
  origin.** Full world translation (moving points to absolute world coords) is WRONG for the
  sensor-relative HUD — it pushed the tree (at x≈8.5 m) to the HUD edge and clipped the left/right/iso
  views.
- **Use stamped transforms, not "latest TF",** when the sensor and TF share a clock domain. Buffered
  clouds re-rotated through the current pose produce exactly the smearing/leaning artifact.
- **On real hardware there are no encoders** for the cutting-arm/turret/extension joints, so absolute
  LiDAR pose is unknowable — but **leveling needs only orientation**, which the **sensor IMUs** (OAK
  camera + Mid-360 LiDAR) supply directly.
</critical>

## Reference map

- **The frame model + root cause:** [references/frame-model.md](references/frame-model.md)
- **Real-hardware sensor inventory + leveling source decision:** [references/hardware-sensors.md](references/hardware-sensors.md)

## Quick workflow

1. Confirm which frame the points are actually in — check `frame_id` AND the actual coordinates
   (don't trust `frame_id` alone; it can say `vehicle_lidar_link` even after leveling, and `world`
   vs `vehicle_lidar_link` can be mixed across files).
2. Read `gateway_node.py::_lidar_in_world` and the gateway config
   (`lidar_world_frame`, `lidar_transform_latest`, `lidar_level_translation`).
3. Apply the rotation-only ("leveled sensor") transform for the HUD; keep the LiDAR at origin.
4. Verify live: `ros2 run tf2_ros tf2_echo world vehicle_lidar_link` should change smoothly while
   moving the arm; the HUD tree should stay vertical.

## Key invariants

- `lidar_transform_latest: true` matches the wall-time-TF interactive sim (the launch file's
  `lidar_timestamp_bridge.py` zeroes stamps and uses latest TF for the same reason). Flip to `false`
  only when TF and the sensor share simulation time.
- `lidar_level_translation: false` (default) = rotation-only, HUD-friendly. `true` = absolute world.
- If TF isn't available at startup, the gateway silently falls back to raw sensor-frame points (no
  crash) and recovers once TF is live.
- Build with `--merge-install` (the workspace `install/` is merged-layout; colcon errors without it).

## Related skills

- `harvester-simulation` — the URDF/TF tree that defines `vehicle_lidar_link` and its mount.
- `telemetry-gateway` — where `_lidar_in_world` and the `lidar_*` config knobs live.
- `tree-height-estimation` — the downstream consumer of the (leveled) LiDAR cloud.
