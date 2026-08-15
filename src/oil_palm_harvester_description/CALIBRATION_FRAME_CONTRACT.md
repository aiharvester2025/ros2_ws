# Docking Sensor Frame and Calibration Contract

This document defines the **simulation-only** calibration path for the five
Gazebo docking range sensors. It adapts the supplied real-machine frame advice
without changing the working Gazebo/RViz control architecture.

## Scope and review of the external advice

The following principles are adopted:

- one documented right-handed coordinate convention;
- an explicit transform convention;
- per-sensor beam correction, scale, bias, valid range, uncertainty, and
  calibration status;
- preservation of a source measurement timestamp and frame ID;
- a strict separation between nominal simulation data and surveyed deployment
  calibration.

The proposed stationary `rail_frame` tree is deliberately **not** added to
this simulation. The range sensors move with the harvester's C-channel, not a
station. Adding a static `rail_frame -> base_link` or reparenting the sensor
array would conflict with the working movable `world -> base_link` transform.

## Active simulation frame tree

```text
world
├── tree_base                                      static environment
└── base_link                                      movable harvester
    └── boom / platform chain
        └── c_channel_platform_link
            ├── c_channel_reference               docking datum
            ├── left_45_range_sensor_link
            ├── right_45_range_sensor_link
            ├── left_side_range_sensor_link
            └── right_side_range_sensor_link
                (centre sensor joins through front_sensor_mount_link)
```

`c_channel_reference` is the common local frame for range-based docking. It
moves with the C-channel. Its intended physical meaning is the selected trunk
centreline / docking datum, not the global `world` origin. Before real-machine
automation, survey and place that datum at the actual desired trunk/cutter
relationship.

Mechanical frames use metres and a right-handed convention:

- `+X`: through the C-channel opening toward the tree;
- `+Y`: left while looking in `+X`;
- `+Z`: up.

The transform notation `T_parent_child` maps a point written in `child` into
`parent`. A valid calibrated range endpoint is therefore:

```text
p_c_channel = T_c_channel_reference_sensor_link
              × T_sensor_link_calibrated_beam
              × [scale × raw_range + bias, 0, 0, 1]
```

The first transform comes from the active URDF / `robot_state_publisher`; the
second and the scalar corrections come from the calibration JSON.

## Source-of-truth rule

The active URDF remains the source of **physical Gazebo ray origin and
direction**. Edit the relevant fixed joint origin in
`urdf/oil_palm_harvester_kinematic.urdf`, rebuild, and relaunch if a simulated
sensor must move.

The JSON configuration must not duplicate those mount poses. It stores only
calibration corrections and measurement metadata. The calibration projector
checks that every configured sensor frame is a fixed URDF descendant of
`c_channel_reference`; this prevents an accidental sensor/world-frame mix-up.

Current nominal transforms, obtained from the active URDF, are:

| Sensor | `T_c_channel_reference_sensor` translation (m) | Beam yaw |
|---|---:|---:|
| Centre | `[-0.470, 0.000, 0.315]` | `0°` |
| Left 45° | `[0.420, 0.630, 0.100]` | `-45°` |
| Right 45° | `[0.420, -0.630, 0.100]` | `+45°` |
| Left side | `[0.950, 0.950, 0.100]` | `-90°` inward |
| Right side | `[0.950, -0.950, 0.100]` | `+90°` inward |

## Configuration files

- `config/range_sensor_calibration.nominal.json` is the launch default. It is
  explicitly `simulation_only`, uses identity beam corrections, and is never
  valid for deployment.
- `config/range_sensor_calibration.deployment.template.json` intentionally
  contains `null` survey values. The simulation projector rejects it, so it
  cannot silently create plausible but unverified guidance.

For each real sensor, a future surveyed configuration must fill and approve:

- `sensor_T_calibrated_beam` — mounting / beam-axis correction;
- `range_scale` and `range_bias_m`;
- `standard_deviation_m` and `valid_range_m`;
- a unique calibration ID and verification record.

Do not copy nominal simulation values into a deployed harvester.

## Runtime outputs

The `range_sensor_calibration.py` node is launched by default with
`range_calibration:=true`. It leaves the five original Range topics unchanged
and publishes:

| Topic | Type | Frame / purpose |
|---|---|---|
| `/harvester/docking/range_hits/<sensor>` | `geometry_msgs/PointStamped` | Calibrated endpoint in `c_channel_reference`; preserves source Range timestamp. |
| `/harvester/docking/range_markers` | `visualization_msgs/MarkerArray` | Thin rays, hit spheres, and a side-pair trunk footprint in the existing RViz window. Marker stamp is zero solely for latest-TF RViz rendering. |
| `/harvester/docking/trunk_center` | `geometry_msgs/PoseWithCovarianceStamped` | Side-pair trunk centre estimate in `c_channel_reference`, only when both side returns are fresh and geometrically consistent. |
| `/harvester/docking/calibration_status` | `std_msgs/String` JSON | Calibration ID, reference frame, per-sensor raw/corrected value, validity, age, and estimate status. |

The side-pair estimate is a geometric diagnostic, not yet a docking command.
It uses two opposed side hits only when their timestamp skew, X-plane match,
and inferred diameter pass the configuration gates.

## Time policy

Gazebo Range messages use simulation time while the current GUI/RViz transform
path uses system time. Do **not** query a dynamic TF transform at the raw Range
timestamp in this simulation. All five sensors are rigid relative to
`c_channel_reference`, so the projector caches only their local fixed
transforms from TF and retains the original timestamp on algorithm outputs.

For a real machine, use synchronized device/localisation clocks and lookup the
dynamic transform at the measurement timestamp. Reject stale or unverified
transforms before geometry-dependent automation.

## Validation and use

After a build, validate the nominal file against the active URDF:

```bash
source /opt/ros/foxy/setup.bash
source ~/ros2_ws/install/setup.bash
python3 "$(ros2 pkg prefix oil_palm_harvester_description)/share/oil_palm_harvester_description/scripts/validate_range_sensor_calibration.py"
```

Then launch normally. RViz displays **Calibrated Docking Range Rays** in the
same window; no second RViz is created.

Useful checks while Gazebo is running:

```bash
ros2 run tf2_ros tf2_echo c_channel_reference left_side_range_sensor_link
ros2 topic echo --once /harvester/docking/range_hits/left_side
ros2 topic echo --once /harvester/docking/trunk_center
ros2 topic echo --once /harvester/docking/calibration_status
```

If a raw range changes but its calibrated marker remains incorrect, first
verify the URDF mount joint and this local transform chain. Do not change
`world -> base_link`, the tree transform, or the Gazebo controller as part of
range-sensor calibration.
