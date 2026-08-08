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
| Five range sensors | centre, left/right 45°, left/right side |
| Vehicle LiDAR | estimated 360° × 60° field of view |
| Platform depth camera | estimated 640 × 400, 80° horizontal FOV |

## Coordinate convention

- `base_link`: X points toward the boom/platform, Y points left, Z points upward.
- The C-channel opening faces +X.
- Each range-sensor link uses +X as its measurement direction.
- `platform_depth_camera_optical_frame` follows the ROS optical convention: Z forward, X right, Y down.

## Required information for an accurate conversion

For a geometry-faithful URDF, provide the SolidWorks assembly or STEP/Parasolid exports, the joint axes, retracted and extended stop positions, component masses/centres of gravity, collision clearances, and the exact sensor mounting transforms. Orthographic screenshots alone cannot recover hidden geometry or reliable dimensions.
