# Camera/LiDAR calibration + projection

Source of truth: `CAMERA_LIDAR_CALIBRATION_CONTRACT.md`, `config/camera_lidar_calibration.nominal.json`,
`scripts/camera_lidar_projection.py`.

## Scope

The cutter depth camera (`platform_depth_camera_optical_frame`) and the arm-mounted LiDAR
(`vehicle_lidar_link`) are the **only** fusion pair. Both are rigid children of
`cutting_arm_base_link`. The docking camera and `/harvester/lidar/points` are NEVER fusion inputs.

## Nominal extrinsics (derived from fixed URDF joints at startup)

```text
p_camera = R_camera_lidar * p_lidar + t_camera_lidar

t_camera_lidar = [0.000, -0.050, -0.125] m

R_camera_lidar = [ [ 0, -1,  0],
                   [ 0,  0, -1],
                   [ 1,  0,  0] ]

quaternion_xyzw = [0.5, -0.5, 0.5, 0.5]
```

Mechanical frames: `+X` toward tree, `+Y` left, `+Z` up. Camera optical frame: `+X` right, `+Y` down,
`+Z` forward. LiDAR link uses mechanical axes.

**Do not add a second static `platform_depth_camera_optical_frame -> vehicle_lidar_link` transform.**

## Correction convention (perception-only)

```text
T_camera_calibrated_lidar = T_camera_lidar_from_URDF * T_nominal_lidar_calibrated_lidar_from_JSON
```

The nominal JSON records `nominal_lidar_T_calibrated_lidar` = identity. The JSON never overrides the
URDF mount — it carries metadata + a perception-only correction.

## Config files

- `camera_lidar_calibration.nominal.json` — Gazebo-only; accepted by validator + projection node.
- `camera_lidar_calibration.deployment.template.json` — hardware commissioning checklist; REJECTED by
  the projection node (contains `null` survey values).

## Topics & timing

| Topic | Use |
|---|---|
| `/harvester/platform_camera/depth/image_raw` | RGB fusion input |
| `/harvester/platform_camera/depth/camera_info` | Authoritative synthetic intrinsics |
| `/harvester/lidar/raw_points` | Raw Gazebo LiDAR fusion input |
| `/harvester/lidar/points` | RViz-only copy; never for fusion |

The projector pairs **raw** RGB + **raw** LiDAR, rejects zero source stamps, and rejects pairs with
> `0.05 s` skew. No dynamic TF lookup needed (camera + LiDAR share `cutting_arm_base_link`). If sensors
are later mounted on separate moving links, implement a unified `/clock`/time-consistent TF design
first — do not relax this to latest-TF.

## Projection outputs (`camera_lidar_projection:=true`)

| Topic | Type | Policy |
|---|---|---|
| `/harvester/perception/camera_lidar/overlay_image` | `Image` | RGB header; coloured LiDAR dots over image |
| `/harvester/perception/camera_lidar/visible_points_raw` | `PointCloud2` | Camera optical frame, paired timestamp; algorithms only |
| `/harvester/perception/camera_lidar/visible_points` | `PointCloud2` | Camera optical frame, zero stamp; RViz only |
| `/harvester/perception/camera_lidar/status` | `String` JSON | Calibration ID, transform, pair skew, validity |

The projector is modest by design: ≤5 Hz, ≤5000 cloud points per input, pure-Python byte buffers (no
OpenCV/PCL/second RViz), to protect the Xavier.

## Validation & use

```bash
python3 "$(ros2 pkg prefix oil_palm_harvester_description)/share/oil_palm_harvester_description/scripts/validate_camera_lidar_calibration.py"
# expected: translation (0.000, -0.050, -0.125), quaternion (0.500, -0.500, 0.500, 0.500)
```

Then launch with `camera_lidar_projection:=true` and enable "Camera + LiDAR Overlay (optional)" in the
RViz Displays panel (no second RViz). Keep `camera_lidar_view:=false` on the Xavier.

## Real-machine boundary

The Gazebo profile is not physical calibration. Before hardware use: survey mounts, load real camera
intrinsics/distortion, estimate target-based camera-to-LiDAR extrinsic correction, quantify
uncertainty, and verify synchronized acquisition timestamps — then record a reviewed deployment profile.
