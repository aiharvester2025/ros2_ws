# Boom Kinematics (closed-form)

Source of truth: `src/harvester_boom_plan/harvester_boom_plan/kinematics.py`. Read that file
alongside this reference; this page records the constants and the *why* behind the formulas so
you can reason about them without re-deriving from the URDF.

## Kinematic chain (from `oil_palm_harvester_kinematic.urdf`)

The boom is a planar 2-link system: a rigid telescopic boom plus a c-channel platform at its tip.

| Joint | origin (in parent link) | axis / type | limit |
|---|---|---|---|
| `boom_turret_joint` | `(-1.05, 0, 1.02)` in base_link | yaw | ±0.35 rad |
| `boom_elevation_joint` | `(0.18, 0, 0.74)` in turret | `(0,-1,0)` revolute | 0 … 1.309 rad (~75°) |
| `boom_extension_1..4_joint` | prismatic X | each 0 … 2.4 m | 4 stages |
| `platform_level_joint` | `(2.40, 0, 0)` in stage-4 | `(0,+1,0)` revolute | ±1.57 rad |
| `platform_fixed_joint` | `(1.02, 0, 0)` in mount | fixed | → `c_channel_reference` |

## Derived constants (as resolved by `kinematics.py`)

```python
TURRET_ORIGIN   = (-1.05, 0, 1.02)
ELEVATION_ORIGIN = (0.18, 0, 0.74)
BOOM_PIVOT_HEIGHT = TURRET_ORIGIN[2] + ELEVATION_ORIGIN[2]   # 1.76 m (in base_link)
BOOM_FIXED_LENGTH = LEVEL_ORIGIN[0]                          # 2.40 m
PLATFORM_TAIL     = PLATFORM_ORIGIN[0]                       # 1.02 m
EXTENSION_PER_STAGE = 2.4 m
STAGE_COUNT        = 4
MAX_STROKE         = 9.6 m
MAX_BOOM_LENGTH    = BOOM_FIXED_LENGTH + MAX_STROKE = 12.0 m

# Gazebo parks base_link at world z = 0.05 m, so pivot world z is:
BASE_LINK_Z_WORLD  = 0.05
BOOM_PIVOT_WORLD_Z = BASE_LINK_Z_WORLD + BOOM_PIVOT_HEIGHT  # 1.81 m
```

The turret origin X plus elevation origin X is the fixed X offset from base_link to the pivot:
`-1.05 + 0.18 = -0.87 m`. At zero yaw this collapses to a single X offset (`boom_pivot_world_xy`).

## Closed-form inverse kinematics

Goal: place `c_channel_reference` (the C-opening, +X toward the tree) exactly at the docking
point (trunk centreline at `H_dock`).

```python
dx = d_horiz_m - PLATFORM_TAIL          # level joint sits PLATFORM_TAIL behind the C-opening
dz = h_dock_m  - BOOM_PIVOT_WORLD_Z

L_needed = hypot(dx, dz)
theta_b  = atan2(dz, dx)                # boom elevation from horizontal
e        = max(0, L_needed - BOOM_FIXED_LENGTH)   # total prismatic stroke
leveling = +theta_b   if leveling_mode == "active" else 0.0
```

- `d_horiz_m` = horizontal distance from the **boom pivot** to the trunk centre (world X).
- `h_dock_m`  = docking height above ground (world z).
- `d_b` (boom horizontal distance) = `d_horiz - PLATFORM_TAIL`.

### Leveling sign (critical)

`platform_level_joint` axis is `(0,+1,0)`, the **opposite** of the boom elevation axis
`(0,-1,0)`. A positive boom angle pitches the tip UP; to counter-rotate the platform mount back
to horizontal, command the level joint **`+theta_b`**. Verified against the live TF chain:
`-theta_b` leaves the c-channel ~89° tilted and ~1.47 m off-target; `+theta_b` recovers
`(8.500, 0, 10.050)` world. The same sign-convention care applies to the real hydraulic valves.

`leveling_mode` is a config flag (`boom_docking.yaml`):
- `active`  (sim default): publish `theta_L = +theta_b`.
- `passive` (unconfirmed hardware): publish `theta_L = 0`, assume a gravity stabilizer.

### Feasibility / `InfeasibleDock`

If `L_needed > MAX_BOOM_LENGTH`, the dock is unreachable. `InfeasibleDock` carries
`needed_length_m`, `deficit_m`, and `recommended_advance_m` — how much closer the base must move
so `L_needed == MAX_BOOM_LENGTH` (dz fixed):

```python
max_dx = sqrt(MAX_BOOM_LENGTH^2 - dz^2)
required_d_horiz = max_dx + PLATFORM_TAIL
advance = d_horiz_m - required_d_horiz
```

`IKResult.feasible` is also `False` when `theta_b` or `leveling` falls outside the joint limits
(returned as a result, NOT raised — callers must check `ik.feasible`, and the orchestrator guards
the empty `extension_per_stage_m` list on infeasible results).

## Forward kinematics (for verification)

```python
L = BOOM_FIXED_LENGTH + extension_total_m
x = pivot_x + L * cos(theta_b) + PLATFORM_TAIL
z = pivot_z + L * sin(theta_b)
```

Used to round-trip-verify that `inverse_kinematics` recovers the target.

## `split_extension`

Total stroke is split **evenly** across the four stages and clamped to `[0, MAX_STROKE]`:
`per = total / 4`. The orchestrator maps `extension_per_stage_m[i]` → `boom_extension_{i+1}_joint`.

## `docking_lower_angle` (theta_d, diagram step 5)

Extra boom-lower angle to descend from the raised/leveled pose onto `H_dock`; returns 0 when
already at height:

```python
boom_length = BOOM_FIXED_LENGTH + extension_total_m
current_z   = pivot_z + boom_length * sin(theta_b)
dz = current_z - h_dock_m
theta_d = max(0, atan2(-dz, boom_length * cos(theta_b)))
```

## URDF resolution behavior

`kinematics.py` resolves constants from the installed URDF at import time via
`ament_index_python.get_package_share_directory("oil_palm_harvester_description")`, and falls
back to the documented literals when the URDF is unavailable. `test_urdf_constants_consistent`
asserts the literals still match the URDF — keep it green; a model geometry change should update
the URDF, not silently edit the literals.
