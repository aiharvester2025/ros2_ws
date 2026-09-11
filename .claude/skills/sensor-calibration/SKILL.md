---
name: sensor-calibration
description: "Implement or modify the oil-palm harvester sensor calibration: the five docking range sensors (c_channel_reference frame contract) and the cutter camera/LiDAR extrinsic + projection contract, including the nominal-vs-deployment boundary."
metadata:
  author: ros2_ws
  version: "1.0.0"
  status: stable
---

# Sensor Calibration (Range + Camera/LiDAR)

Implement and maintain the **simulation-only** calibration path for the five docking range sensors
and the cutter-camera/LiDAR extrinsic. These are perception-only corrections layered on top of the
active URDF — they never move a sensor, publish a corrective TF, or change the control path.

This skill records the **frame conventions, the transform math, and the hard nominal-vs-deployment
boundary** that a smaller model would otherwise fumble (especially the "reject the deployment
template" behavior and the "URDF is the geometry source, JSON is corrections only" rule).

## When this applies

- Editing `config/range_sensor_calibration.*.json`, `config/camera_lidar_calibration.*.json`, or the
  `scripts/range_sensor_calibration.py` / `scripts/camera_lidar_projection.py` / validators.
- Changing a range-sensor or camera/LiDAR mount, beam correction, or projection.
- Debugging "calibrated marker is wrong" or "projection node rejects my config".

## Core facts (internalize before editing)

<critical>
- **The URDF is the geometry source of truth; the JSON is corrections-only.** JSON must NOT duplicate
  mount poses. Move a sensor by editing the URDF fixed-joint origin, rebuild, relaunch — never by
  editing the JSON translation.
- **Nominal (simulation) profiles must never reach hardware.** `*.nominal.json` is `simulation_only`
  with identity corrections. `*.deployment.template.json` intentionally contains `null` survey values
  and is **rejected by the projector/validator** so it can't silently produce plausible-but-unverified
  guidance. Do not copy nominal values into a deployed harvester.
- **The five docking ranges are calibrated only in `c_channel_reference`.** The cutter-forward range
  (`/harvester/cutting_tool_left_range`) crosses moving joints (rail/lift/extension/cutter) and is
  deliberately EXCLUDED — it stays a separate yellow marker, not part of the five-sensor estimator.
- **Camera/LiDAR fusion uses only the cutter camera + arm LiDAR** (both rigid on `cutting_arm_base_link`),
  raw sim-time topics only. The docking camera and `/harvester/lidar/points` are never fusion inputs.
- **Do not query dynamic TF at a raw Range/Gazebo timestamp in this sim** (sim-time vs wall-time
  mismatch). The projector caches fixed local transforms and preserves the source timestamp.
</critical>

## Reference map

- **Range calibration (frame tree, beam math, outputs, side-pair trunk estimate):** [references/range-calibration.md](references/range-calibration.md)
- **Camera/LiDAR calibration (extrinsics, projection, topics, timing):** [references/camera-lidar-calibration.md](references/camera-lidar-calibration.md)

## Quick workflow

1. Read the relevant contract in `src/oil_palm_harvester_description/` (CALIBRATION_FRAME_CONTRACT.md
   or CAMERA_LIDAR_CALIBRATION_CONTRACT.md) and the `scripts/` projector + `validate_*.py` before editing.
2. Validate offline (no Gazebo needed):
   ```bash
   source /opt/ros/foxy/setup.bash && source ~/ros2_ws/install/setup.bash
   python3 "$(ros2 pkg prefix oil_palm_harvester_description)/share/oil_palm_harvester_description/scripts/validate_range_sensor_calibration.py"
   python3 "$(ros2 pkg prefix oil_palm_harvester_description)/share/oil_palm_harvester_description/scripts/validate_camera_lidar_calibration.py"
   ```
3. Launch normally and check the calibrated outputs (see the references for `ros2 topic echo` /
   `tf2_echo` commands).

## Key invariants

- Transform convention `T_parent_child` maps child→parent; a calibrated range endpoint is
  `p = T_c_channel_ref_sensor × T_sensor_calibrated_beam × [scale*raw + bias, 0,0,1]`.
- The projector checks every configured sensor frame is a **fixed URDF descendant** of
  `c_channel_reference` (prevents sensor/world-frame mix-up).
- `c_channel_reference` is the local docking datum (moves with the C-channel), NOT the world origin.
- The deployment template's `null` survey values must be filled (mount/beam correction, scale/bias,
  std-dev, valid range, calibration ID, verification record) before hardware use.
- If sensors are ever mounted on separate moving links, implement a unified `/clock`/time-consistent
  TF design FIRST — do not relax the latest-TF rule.

## Related skills

- `harvester-simulation` — the active URDF that is the geometry source for these calibration profiles.
- `lidar-hud-leveling` — the rotation-only "leveled sensor frame" that shares the same
  sim-time/wall-time TF concern.
