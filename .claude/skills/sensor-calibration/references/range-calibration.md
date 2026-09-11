# Range-sensor calibration (five docking sensors)

Source of truth: `CALIBRATION_FRAME_CONTRACT.md`, `config/range_sensor_calibration.nominal.json`,
`scripts/range_sensor_calibration.py`.

## Scope

Calibrates the five **fixed** docking ranges into `c_channel_reference`. Explicitly excludes the
cutter-forward sensor (`/harvester/cutting_tool_left_range`) — it is a fixed child of
`cutting_tool_link` and moves with rail/lift/extension/cutter, so it cannot satisfy the rigid
`c_channel_reference -> sensor` contract. It keeps its own yellow marker on
`/harvester/cutter/range_markers`.

## Frame tree (active)

```text
world
├── tree_base                     static environment
└── base_link                     movable harvester
    └── boom / platform chain
        └── c_channel_platform_link
            ├── c_channel_reference            docking datum
            ├── left_45_range_sensor_link
            ├── right_45_range_sensor_link
            ├── left_side_range_sensor_link
            └── right_side_range_sensor_link
                (centre sensor joins through front_sensor_mount_link)
```

Convention (mechanical, right-handed): `+X` through the C-opening toward the tree, `+Y` left, `+Z` up.
Each range-sensor link uses `+X` as its measurement direction.

## Calibrated endpoint math

```text
p_c_channel = T_c_channel_reference_sensor_link
              × T_sensor_link_calibrated_beam
              × [scale × raw_range + bias, 0, 0, 1]
```

First transform from the URDF/`robot_state_publisher`; the beam correction + scale/bias from JSON.

## Nominal transforms (from active URDF)

| Sensor | `T_c_channel_reference_sensor` translation (m) | Beam yaw |
|---|---|---|
| Centre | `[-0.470, 0.000, 0.315]` | 0° |
| Left 45° | `[0.420, 0.630, 0.100]` | −45° |
| Right 45° | `[0.420, -0.630, 0.100]` | +45° |
| Left side | `[0.950, 0.950, 0.100]` | −90° inward |
| Right side | `[0.950, -0.950, 0.100]` | +90° inward |

## Config files

- `range_sensor_calibration.nominal.json` — launch default; `simulation_only`, identity corrections.
- `range_sensor_calibration.deployment.template.json` — `null` survey values; the projector REJECTS it.

## Runtime outputs (`range_calibration:=true`)

| Topic | Type | Purpose |
|---|---|---|
| `/harvester/docking/range_hits/<sensor>` | `PointStamped` | Calibrated endpoint in `c_channel_reference`; preserves source timestamp. |
| `/harvester/docking/range_markers` | `MarkerArray` | Rays + hit spheres + side-pair trunk footprint; zero stamp for RViz only. |
| `/harvester/docking/trunk_center` | `PoseWithCovarianceStamped` | Side-pair trunk centre in `c_channel_reference`, gated on fresh + consistent side returns. |
| `/harvester/docking/calibration_status` | `String` JSON | Calibration ID, per-sensor raw/corrected, validity, age, status. |

The side-pair trunk estimate is a **geometric diagnostic**, not a docking command: it requires both
opposed side hits fresh, X-plane matched, and inferred diameter within gates.

## Validation & checks

```bash
python3 "$(ros2 pkg prefix oil_palm_harvester_description)/share/oil_palm_harvester_description/scripts/validate_range_sensor_calibration.py"
# while Gazebo runs:
ros2 run tf2_ros tf2_echo c_channel_reference left_side_range_sensor_link
ros2 topic echo --once /harvester/docking/range_hits/left_side
ros2 topic echo --once /harvester/docking/trunk_center
ros2 topic echo --once /harvester/docking/calibration_status
```

If a raw range changes but its calibrated marker stays wrong, verify the URDF mount joint and the
local transform chain — do NOT touch `world -> base_link`, the tree transform, or the Gazebo controller.
