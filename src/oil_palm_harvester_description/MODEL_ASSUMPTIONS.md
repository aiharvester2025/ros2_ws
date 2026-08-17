# Estimated Geometry and Kinematic Assumptions

The supplied top, side, front and isometric drawings do not contain a dimensional scale. The meshes in this package are therefore **engineering placeholders**, not manufacturing geometry.

| Item | Estimated value used |
|---|---:|
| Vehicle overall length | 4.30 m |
| Vehicle body width | 1.75 m |
| Wheel diameter / width | 0.88 m / 0.30 m |
| Boom pivot height | 1.76 m above `base_link` ground plane |
| Boom nested stage visual length | approximately 2.5 m each |
| Number of telescopic stages | 4 prismatic extensions after the main stage |
| Extension per stage | 0 to 2.40 m |
| Maximum pivot-to-tip reach | approximately 12.0 m |
| Boom elevation range | 0 to 75 degrees |
| Turret yaw range | ±20 degrees |
| Platform outer radius | 1.20 m |
| Platform inner radius | 0.55 m |
| C-opening at inner tips | approximately 0.82 m |
| Reference trunk diameter | 0.60 m |
| Platform body height | 0.38 m plus top rail |
| Cutting-arm maximum extension | 0.375 m from its retracted stop |
| Docking range sensors | centre, left/right 45°, left/right side; each is a 20 Hz single ray with 0.05–3.0 m range |
| Cutter range sensor | one forward-facing sensor on the left side of `cutting_tool_link`; its raw topic is separate from the docking estimator |
| Cutting-arm LiDAR | Gazebo Mid-360 coverage approximation: 120° horizontal, approximately -7° to +52° vertical, 107 × 64 rays |
| Cutting-arm depth camera | estimated 640 × 400, 80° horizontal FOV |
| Docking depth camera | estimated 320 × 240, 80° horizontal FOV, 8 Hz |

## Coordinate convention

- `base_link`: X points toward the boom/platform, Y points left, Z points upward.
- The C-channel opening faces +X.
- Each range-sensor link uses +X as its measurement direction.
- `platform_depth_camera_optical_frame` follows the ROS optical convention: Z forward, X right, Y down.
- The legacy `platform_depth_camera_*` frame names identify the arm-mounted
  depth camera.  Its fixed joint is parented to `cutting_arm_base_link` at
  `(0.125, 0, 0.25)` m: the current manual camera placement on the arm-base
  block. It therefore follows the rail-carriage yaw and cutting-arm lift, but not the
  extension stroke. To tune it manually, edit the `xyz` and `rpy` attributes
  of `platform_depth_camera_joint` in `urdf/oil_palm_harvester_kinematic.urdf`.
- The legacy `vehicle_lidar_link` name identifies the arm-mounted LiDAR. Its
  fixed joint is parented to `cutting_arm_base_link` at `(0, 0, 0.30)` m:
  centred above the visible arm base. It follows the
  rail-carriage yaw and cutting-arm lift, but not the extension stroke. Tune
  `vehicle_lidar_joint` in the same kinematic URDF to adjust this mount.
- `front_sensor_mount_link` is the compact platform-top carrier for the centre
  docking range sensor and docking camera. Tune
  `front_sensor_mount_joint` in the active kinematic URDF to move the carrier
  as one unit. The child joints `center_range_sensor_joint` and
  `front_depth_camera_joint` are the precise per-device adjustments.
- `cutting_tool_left_range_sensor_joint` is the forward trunk-facing cutter
  range mount. Its local `xyz` and `rpy` are expressed in `cutting_tool_link`:
  `+X` is forward toward the trunk, `+Y` is left, and `+Z` is up. It follows
  rail, lift, extension, and cutter motion, so it must remain outside the
  fixed C-channel docking-range calibration profile.

## Simulation-control assumptions

The supported combined launch runs the harvester in rate-limited kinematic
articulation mode at 20 Hz, using the URDF joint limits and velocity limits.
The turret is capped at 0.05 rad/s. The model remains non-static and its base
can move through `/harvester/cmd_vel`, but harvester collisions are off by
default. This is a sensor-development configuration, not a physically valid
contact, mass, or cutting model.

## Half-length cutting-arm envelope

The cutting-arm base, extension link, and cutter use half of their original
length in X only.  Their Y and Z dimensions are unchanged.  To keep the chain
assembled, the extension origin is 0.31 m, the extension stroke is 0 to
0.375 m, and the cutter attachment is 0.46 m beyond the extension link.

## Required information for an accurate conversion

For a geometry-faithful URDF, provide the SolidWorks assembly or STEP/Parasolid exports, the joint axes, retracted and extended stop positions, component masses/centres of gravity, collision clearances, and the exact sensor mounting transforms. Orthographic screenshots alone cannot recover hidden geometry or reliable dimensions.
