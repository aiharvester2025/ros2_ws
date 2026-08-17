# Camera--LiDAR Frame, Calibration, and Fusion Contract

This document defines the camera--LiDAR contract for the working Gazebo/RViz
harvester simulation. It is deliberately additive: it does **not** move a
sensor, publish a corrective TF, alter `/harvester/joint_commands`, alter
`/harvester/joint_states`, or change the Gazebo controller.

## Scope and authority

The physical nominal mounts are defined only by the active URDF:

- `cutting_arm_base_link -> platform_depth_camera_link`
- `platform_depth_camera_link -> platform_depth_camera_optical_frame`
- `cutting_arm_base_link -> vehicle_lidar_link`

The combined launch uses this same URDF to construct both the Gazebo model and
the RViz robot model. Gazebo measured joint feedback now drives
`robot_state_publisher`, so the camera and LiDAR TF frames follow the actual
Gazebo rail/lift state rather than an immediate GUI target.

The calibration JSON does **not** duplicate or override these mount poses. It
contains only metadata, a perception-only correction, timing rules, and output
topic names. This prevents a visual URDF pose and the Gazebo sensor ray pose
from diverging.

The lower-rate platform docking camera is intentionally outside this contract.
It is a second Gazebo camera stream selected for the one RViz image viewport;
it is not a camera/LiDAR fusion input. The optional projection always uses the
cutter camera (`platform_depth_camera_optical_frame`) and the arm-mounted
LiDAR (`vehicle_lidar_link`).

## Frames and nominal extrinsics

Mechanical frames use `+X` toward the tree, `+Y` left, and `+Z` upward. The
camera optical frame follows ROS optical convention: `+X` right, `+Y` down,
`+Z` forward. The LiDAR link uses mechanical axes.

For the active URDF, the fixed transform maps a LiDAR point into the camera
optical frame:

```text
p_camera = R_camera_lidar * p_lidar + t_camera_lidar

t_camera_lidar = [0.000, -0.050, -0.125] m

R_camera_lidar = [ [ 0, -1,  0],
                   [ 0,  0, -1],
                   [ 1,  0,  0] ]

quaternion_xyzw = [0.5, -0.5, 0.5, 0.5]
```

This relationship is derived at startup from fixed URDF joints. Do **not** add
a second static `platform_depth_camera_optical_frame -> vehicle_lidar_link`
transform.

## Calibration profiles

| File | Purpose | Runtime use |
|---|---|---|
| `config/camera_lidar_calibration.nominal.json` | Gazebo-only nominal profile | Accepted by validator and projection node |
| `config/camera_lidar_calibration.deployment.template.json` | Physical-hardware commissioning checklist | Intentionally rejected by the projection node |

The nominal profile records that the URDF is the geometry source and that its
`nominal_lidar_T_calibrated_lidar` correction is identity. The future hardware
profile must include a surveyed transform correction, uncertainty, camera and
LiDAR serial numbers, a target/calibration method, reprojection error, and a
verified clock offset before it can be approved for deployment.

The correction convention is:

```text
T_camera_calibrated_lidar = T_camera_lidar_from_URDF
                            * T_nominal_lidar_calibrated_lidar_from_JSON
```

It is used by perception only; it never changes the URDF or TF tree.

## Topics and timestamp policy

| Topic | Use |
|---|---|
| `/harvester/platform_camera/depth/image_raw` | RGB fusion input |
| `/harvester/platform_camera/depth/camera_info` | Authoritative synthetic camera intrinsics |
| `/harvester/lidar/raw_points` | Raw Gazebo LiDAR fusion input |
| `/harvester/lidar/points` | RViz-only copy; never use for fusion |

The separate docking-camera streams are
`/harvester/docking_camera/depth/image_raw` and
`/harvester/docking_camera/depth/camera_info`. They are forwarded only when
the RViz panel selects `docking` on `/harvester/camera_view/select`; the
selector preserves headers and publishes no TF. They must not be substituted
into the cutter camera/LiDAR calibration profile.

The Gazebo camera and raw LiDAR use acquisition timestamps in Gazebo simulation
time. The LiDAR bridge intentionally writes a zero stamp to
`/harvester/lidar/points` so RViz can request its latest TF. That is correct
for a display but invalid for time-correlated fusion.

The optional projector pairs only raw RGB images and raw LiDAR clouds, rejects
zero source stamps, and rejects a pair with more than `0.05 s` skew. It needs
no dynamic TF lookup because the camera and LiDAR are rigidly attached to the
same moving `cutting_arm_base_link`. If sensors are later mounted on separate
moving links, first implement a unified `/clock`/time-consistent TF design;
do not relax this rule to a latest-TF transform.

## Optional projection outputs

With `camera_lidar_projection:=true`, the projector publishes:

| Topic | Type | Timestamp/frame policy |
|---|---|---|
| `/harvester/perception/camera_lidar/overlay_image` | `sensor_msgs/Image` | RGB image header; coloured LiDAR dots over the image |
| `/harvester/perception/camera_lidar/visible_points_raw` | `sensor_msgs/PointCloud2` | Camera optical frame and paired acquisition timestamp; algorithms only |
| `/harvester/perception/camera_lidar/visible_points` | `sensor_msgs/PointCloud2` | Camera optical frame with zero stamp; RViz only |
| `/harvester/perception/camera_lidar/status` | `std_msgs/String` JSON | Calibration ID, transform, pair skew, and validity state |

The current projector is intentionally modest: at most 5 Hz and 5,000 cloud
points per input cloud. It uses pure Python byte buffers rather than OpenCV,
PCL, or another RViz process, protecting the Xavier from the earlier
multi-window resource issue.

## Use and verification

First verify the contract without running Gazebo:

```bash
source /opt/ros/foxy/setup.bash
source ~/ros2_ws/install/setup.bash
python3 "$(ros2 pkg prefix oil_palm_harvester_description)/share/oil_palm_harvester_description/scripts/validate_camera_lidar_calibration.py"
```

The expected transform printed by the validator has translation
`(0.000, -0.050, -0.125)` m and quaternion `(0.500, -0.500, 0.500, 0.500)`.

Start the normal low-resource simulation unchanged:

```bash
ros2 launch oil_palm_harvester_description gazebo_harvester_and_tree.launch.py
```

Enable the optional projection only when ready to inspect registered data:

```bash
ros2 launch oil_palm_harvester_description gazebo_harvester_and_tree.launch.py \
  camera_lidar_projection:=true
```

This does not start a second RViz. In the existing RViz, enable **Camera +
LiDAR Overlay (optional)** in the Displays panel. Keep
`camera_lidar_view:=false` on the Xavier. The raw colour camera display remains
available for comparison.

## Real-machine commissioning boundary

The Gazebo profile is not a physical calibration. Before using a real camera
and Livox Mid-360 for docking, survey the mounts, load the real camera
intrinsics/distortion calibration, estimate the target-based camera-to-LiDAR
extrinsic correction, quantify uncertainty, and verify synchronized acquisition
timestamps. Record those results in a reviewed deployment profile; do not use
the simulation nominal profile on hardware.
