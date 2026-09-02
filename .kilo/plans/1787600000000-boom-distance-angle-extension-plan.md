# Boom Distance, Boom Angle, and Boom Extension Estimation — Plan

**Date:** 2026-09-02
**Workspace:** `/home/ubuntu/ros2_ws`
**Baseline:** tree-height estimate from `analyze_tree_scan_v2.py` (`/harvester/tree/docking_estimate`), Gazebo kinematic bridge, `range_sensor_calibration.py` (five-sensor docking rays + side-pair `trunk_center` estimate), and the active URDF `src/oil_palm_harvester_description/urdf/oil_palm_harvester_kinematic.urdf`.

> **⚠ Note (2026-09-02):** this plan has been fully implemented and validated in
> simulation. The live experiment surfaced two corrections that are **not** yet
> reflected in the body below — the leveling sign (`θ_L = +θ_b`, not `−θ_b`) and
> the boom-pivot world Z (`1.81 m`, not `1.76 m`) — plus a range-sensor sentinel
> fix and two-phase safety gating. See **§14 "Implementation delta"** for the
> corrected, as-built record. The design narrative in §1–§12 is preserved as
> originally written.

---

## 1. Goal

Convert the tree-height estimate (docking-point height above ground, in the
`base_link` frame) and the sensed trunk horizontal position into the joint
targets needed to bring the c-channel platform onto the tree trunk safely.

Concretely, produce four joint targets whose outputs follow the figure's
*ESTIMATION WORKFLOW* (steps 1–5) and whose outputs all live in **closed form
given the URDF**, with sensor feedback only used to **verify** and to **guard
the final approach**:

| Symbol | Meaning | Output channel |
|---|---|---|
| `θ_b` | boom elevation (raise up) | `/harvester/boom/elevation_target_rad` |
| `θ_p` | platform level angle (= boom angle before extension; leveling correction `θ_L` applied at step 3) | `/harvester/boom/platform_level_target_rad` |
| `l_b + l_p` | total boom extension (sum of the four prismatics) | `/harvester/boom/extension_target_m` |
| `d_b` | horizontal boom-distance the platform tip will be from the trunk after extension (used for `θ_d`, see §6) | `/harvester/boom/horizontal_distance_m` |
| `θ_d` | extra boom-lower angle required, in step 5, to bring the platform onto the docking point | `/harvester/boom/docking_lower_angle_rad` |

All five are published on **read-only diagnostic topics**. **No joint command
is published by this node.** Joint actuation remains the operator's
responsibility; the Gazebo/RViz control path is unchanged (see
`SIMULATION_HANDOFF.md` and `TELEMETRY_HANDOFF.md` safety boundary).

The `telemetry/gateway/dashboard` ZMQ path is **not** the focus of this plan:
the topic surface stays internal to ROS 2 for now. A canonical channel
addition (e.g. `v1/boom/plan`) is a follow-up after validation.

---

## 2. Definitions and inputs (closed form, no encoder feedback)

### 2.1 Frames and conventions

From `urdf/oil_palm_harvester_kinematic.urdf` (lines 185–435, 875–885) and
`MODEL_ASSUMPTIONS.md`:

- `base_link`: +X toward the boom/platform, +Y left, +Z up. Origin ≈ 0.05 m
  above ground (Gazebo kinematic base z).
- `boom_turret_joint` origin: `(-1.05, 0, 1.02)` m in `base_link` (turret
  base 1.02 m above `base_link`). Limit ±0.35 rad (Z axis, yaw).
- `boom_elevation_joint` origin: `(0.18, 0, 0.74)` m in `boom_turret_link`
  (boom pivot 0.74 m above turret). Axis `(0,-1,0)` → positive elevation
  rotates the boom **up**; limit 0…1.309 rad (≈ 0…75°).
- `boom_extension_1..4_joint` (prismatic X, axis `(1,0,0)`), all origins
  `(0,0,0)`, each limit 0…2.4 m, total stroke 9.6 m. They stack: the total
  extended length `e = e1+e2+e3+e4` adds to the rigid fixed portion between
  the boom pivot and the platform mount.
- `platform_level_joint` origin `(2.4, 0, 0)` in `boom_stage_4_link` (this is
  the *nominal retracted-tip offset* that the next joint declares; the
  actual tip follows `e`). Axis `(0,1,0)`, limit ±1.57 rad — the only joint
  that can **counter-rotate** the platform relative to the boom to bring
  the platform angle to zero.
- `platform_fixed_joint` origin `(1.02, 0, 0)` in `platform_mount_link` →
  `c_channel_platform_link`. The C-opening faces +X.
- `c_channel_reference_joint` origin `(0, 0, 0)` in `c_channel_platform_link`
  → `c_channel_reference` (the calibration datum).

The kinematic chain (ignoring the 2-DoF turret yaw and the carriage yaw used
by the cutting arm, neither of which is needed for the docking maneuver) is:

```
base_link ─(turret, y_t)─► boom_turret_link
       ─(boom_elevation, θ_b)─► boom_stage_0_link
       ─(prismatic, e1)─► boom_stage_1_link
       ─(prismatic, e2)─► boom_stage_2_link
       ─(prismatic, e3)─► boom_stage_3_link
       ─(prismatic, e4)─► boom_stage_4_link
       ─(level, θ_p)─► platform_mount_link
       ─(fixed 1.02, 0, 0)─► c_channel_platform_link  (≡ c_channel_reference)
```

### 2.2 Tree-height input

- `H` = tree height (m), from `/harvester/tree/docking_estimate` or its
  successor published by `analyze_tree_scan_v2.py`.
- `H_dock` = docking-point height above ground (m) = `H - offset`, where
  `offset` is the planting/FFB-clearance gap (`tree_targets.yaml`).
- For Simulation: ground truth is `H=12.0`, `crown_base=9.2`,
  `trunk_diameter=0.6` (see `tree_scan_002` outcome in
  `TREE_HEIGHT_SCAN.md`).
- Trunk horizontal position in `base_link` is reported by the docking
  range side-pair estimator at `/harvester/docking/trunk_center`
  (`PoseWithCovarianceStamped` in `c_channel_reference`).

### 2.3 Closed-form boom geometry

Let:

```
h_p       = 1.02 + 0.74   = 1.76 m   (boom pivot height above base_link's ground plane, MODEL_ASSUMPTIONS.md)
L_fixed   = 2.4            m         (rigid boom tip - mount offset; URDF origin of platform_level_joint)
L_plat    = 1.02           m         (platform_fixed_joint xyz to c_channel_reference)
h_C       = c_channel_reference Z = local height offset above the mount (it is co-located with platform body)
```

The boom is treated as one rigid member of length `L_b = L_fixed + (e1+...+e4)`
with elevation `θ_b` from horizontal. The platform sits at the boom tip plus
the platform's local `L_plat` tail. With turret at zero yaw (we align the
harvester straight at the tree in `base_link.y=0` before the maneuver):

```
Platform X in base_link:  x_C = L_b * cos θ_b + L_plat * cos θ_p       (eq. A)
Platform Z in base_link:  z_C = h_p + L_b * sin θ_b + L_plat * sin θ_p  (eq. B)
```

The diagram's **"before extension"** state has `θ_p = θ_b` (platform fixed
joint has rotated the level joint by `-θ_b` so the platform's body angle is
zero — the same physical setup as the post-`θ_L` state). Let
`θ_post = θ_p = 0` be the leveled platform angle after `θ_L` has been applied.

### 2.4 Inverse kinematics used here

Two IK cases are needed:

**Case I — "leveled platform" (state 3 of the diagram).**
Platform angle `= 0`, boom angle `θ_b`, total length `L_b = L_fixed + Σe`.
The c-channel reference must land on the docking point
`(d_horiz, h_p - 0, H_dock)` measured from `base_link`, where
`d_horiz = (base_x - trunk_x)` is the **horizontal distance from the
turret yaw axis to the trunk** in `base_link.X` (positive = trunk ahead).

```
θ_b = atan2( H_dock - h_p,  d_horiz - L_plat )
L_b = sqrt( (d_horiz - L_plat)^2  +  (H_dock - h_p)^2 )
Σe  = L_b - L_fixed                          (clamped to [0, 9.6] m and
                                               split across 4 stages)
```

**Case II — "before extension" state 1 of the diagram.**
Same platform angle 0, boom angle `θ_b(1)`, but `L_b = L_fixed` (nested).
The platform is *above* the docking point with horizontal gap
`Δx(1) = (d_horiz - L_plat) - L_fixed * cos θ_b(1)` after we choose an
angle that reaches the docking-point **height**. Solving with `L_b = L_fixed`:

```
sin θ_b(1) = (H_dock - h_p) / L_fixed
```

This angle is only valid if `|H_dock - h_p| ≤ L_fixed` (so the docking height is
reachable with a nested boom); otherwise the IK for `θ_b` is undefined and the
node reports `INFEASIBLE_DOCK_HEIGHT`. In real workflows the operator typically
raises `boom_elevation` past this point using the four-prismatic sum; this
state in the diagram is the *conceptual "raise to height first"* picture, not
a physically required intermediate.

### 2.5 Step-by-step plan mirrors ESTIMATION WORKFLOW

1. **Tree height** → `H` (from `/harvester/tree/docking_estimate`).
2. **Docking point height** → `H_dock = H - offset` (parameter from
   `tree_targets.yaml`; default `offset = 1.0` m FFB clearance).
3. **Before-extension angle:** `θ_b(1) = asin((H_dock - h_p)/L_fixed)`. Plot
   the nested boom reaching just over the docking point. Note the figure's
   "boom angle" before extension is `θ_b` of step 3 here, with `θ_p = θ_b`
   (platform tilt tracks boom because the platform_fixed_joint is a fixed
   parent of the mount; only the level joint can counter-rotate).

   *Important:* if `L_b > L_fixed`, then `θ_b = atan2(z - h_p, x - L_plat)`
   **and** `Σe = L_b - L_fixed` is the required extension. The "raise to
   height then extend" diagram can be replaced by a single IK step that
   simultaneously picks `θ_b` and `Σe` — equivalent in math, simpler in code.

4. **Raise and extend:** command
   `(boom_elevation → θ_b, boom_extension_1..4_joint → Σe / 4 each,  boom_turret → 0).
   The platform tilt is `(θ_b + θ_p = θ_b)` only when the level joint is
   referenced to the boom's elevation joint — `θ_p` is the platform's *level*
   variable that counter-rotates to keep the platform horizontal.

5. **Apply leveling correction** (state 3 of the diagram):
   the leveling mechanism is **not yet confirmed on hardware**, so the node
   supports both paths behind a config flag `leveling_mode`:

   > **Correction (2026-09-02):** the sign below is **wrong** and was fixed in
   > the live simulation. The `platform_level_joint` axis is `(0,+1,0)`, opposite
   > the boom's `(0,-1,0)`, so the correct command is **`θ_L = +θ_b`**, not
   > `-θ_b`. See §13. The text below is preserved as originally written.

   - `leveling_mode: active` (default for this simulation): `θ_L = -θ_b`
     (rotate the level joint by `-θ_b` so the platform body's world angle
     becomes zero). After this, `θ_p = -θ_b` *with respect to the* platform
     mount joint's reference frame; if we use URDF world tracking `θ_p = 0`.
     Concretely: if the boom is at `θ_b=30°`, set `platform_level_joint =
     -θ_b` and the platform becomes horizontal.
   - `leveling_mode: passive`: the platform self-levels by gravity (a real
     stabilizer, per the drawings' safety note). The node outputs `θ_L = 0`
     and `platform_level_rad = 0`, and does **not** suggest a level-joint
     command; the `BoomPlan.notes` field records
     `"passive stabilizer assumed"`.

   The "platform angle equals boom angle" observation in the diagram refers
   to the *before-leveling* setup; after `θ_L` the platform is independent
   of the boom. This flag is explicitly marked "confirm on hardware" and is
   surfaced as Risk #3.

6. **Lower onto the docking point with 5-range feedback** — see §3.

---

## 3. The 5-range "docking safety" feedback (step 5)

The c-channel reference sits on the platform body. The five fixed range
sensors (`/harvester/{center,left_45,right_45,left_side,right_side}_range`)
beam out from `c_channel_reference`. The cutter-following
`/harvester/cutting_tool_left_range` (yellow marker) is the **moving** the
single most useful sensor in this phase; it is intentionally separate from
the five-sensor docking estimator (see `CALIBRATION_FRAME_CONTRACT.md`).

The five-sensor array serves **two distinct roles** (confirmed during
planning):

1. **Clearance/skew watchdog** — a continuous, monotonic guard on the
   approach motion (raise → extend → level → lower). It only *permits*
   motion when every relevant sensor reads above a minimum gap, and it
   trips `EMERGENCY_STOP` on any near-contact.
2. **Trunk-centering confirmation** — the side pair (`left_side`,
   `right_side`) and the ±45° pair (`left_45`, `right_45`) together
   reconstruct the trunk centreline *inside* the C-channel opening. This is
   an independent geometry check that **gates** `DOCKING_OK`: even if the
   watchdog clearances are satisfied, the node will **not** report
   `DOCKING_OK` unless the reconstructed trunk centre sits inside the
   calibrated C-opening footprint (see `CALIBRATION_FRAME_CONTRACT.md`
   side-pair estimator and `trunk_center` topic for the matching logic).

State-machine phases the node reports:

| Phase | Sensor condition for safe progress |
|---|---|
| `EXTEND_OK` | watchdog: `min(center, c45_l, c45_r)` ≥ `EXTEND_SAFE_M` (default 0.6 m) |
| `HOLD_AT_DISTANCE` | watchdog: `min` ≥ `EXTEND_STOP_M` (default 0.4 m) — extend no further |
| `ALIGN_OK` | centering: `|left_45 - right_45|` ≤ `ALIGN_SKEW_M` (default 0.08 m) |
| `LEVEL_OK` | centering: `|left_side - right_side|` matches the expected trunk footprint |
| `DOCKING_OK` | **watchdog + centering both pass**: all five within `DOCKING_GAP_M` (default 0.18 m) **and** reconstructed trunk centre inside the C-opening footprint |
| `EMERGENCY_STOP` | watchdog: any sensor `≤ EMERGENCY_M` (default 0.05 m) — halt `θ_d` motion, log event |

`DOCKING_OK` is a logical **AND** of the two roles. `ALIGN_OK` / `LEVEL_OK`
are centering sub-gates that must hold before `DOCKING_OK` is admitted; the
watchdog gates (`EXTEND_*`, `EMERGENCY_STOP`) are evaluated independently at
every tick so a near-contact always overrides a centering pass.

The `θ_d` computation (state 4 of the diagram):

```
Δz = (z_C_after_leveling - H_dock)
Δx = (x_C_after_leveling - d_horiz)
θ_d = atan2( -Δz,  Δx )         (positive = boom lower)
```

The five-sensor readings provide the continuous safety watchdog **and** the
centering confirmation on `θ_d`, matching the figure's "5-range sensors
guidance safety" annotation. The node **never** commands `θ_d`. It only
publishes the *desired* `θ_d` and the *current safety state* (including the
centring sub-state). The operator (or downstream `joint_commands` publisher)
is the only writer to the Gazebo bridge.

---

## 4. Architecture

### 4.1 New package: `harvester_boom_plan`

- Path: `src/harvester_boom_plan/`
- License: Apache-2.0 (matches existing packages).
- `package.xml`: `ament_python`, depends on
  `rclpy`, `std_msgs`, `geometry_msgs`, `sensor_msgs`, `tf2_ros`,
  `oil_palm_harvester_description` (URDF resource).
- New executable: `harvester_boom_plan/boom_plan_node.py` — the dedicated
  read-only estimator node.
- New library module: `harvester_boom_plan/kinematics.py` — URDF-derived
  forward kinematics helpers (no ROS dependency).
- New launch: `harvester_boom_plan/launch/boom_plan.launch.py`.

### 4.2 Subscriber & publisher surface

| Topic | Type | Direction | Purpose |
|---|---|---|---|
| `/harvester/tree/docking_estimate` | `geometry_msgs/PointStamped` | sub | Tree height H in `world` (x=trunk_x, y=0, z=H, as published by next-step estimator) |
| `/harvester/docking/trunk_center` | `geometry_msgs/PoseWithCovarianceStamped` | sub | Trunk horizontal position in `c_channel_reference` (recent, valid only) |
| `/harvester/joint_states` | `sensor_msgs/JointState` | sub | Measured angles for FK validation (simulation only) |
| `/harvester/{center,left_45,right_45,left_side,right_side}_range` | `sensor_msgs/Range` | sub | Phase 5 watchdog + centering |
| `/harvester/cutting_tool_left_range` | `sensor_msgs/Range` | sub | Phase 5 watchdog + centering (kept separate from five-sensor estimator) |
| `/harvester/boom/plan` | `harvester_boom_plan/msg/BoomPlan` | pub | IK + phase + safety report (see §4.3) |
| `/harvester/boom/safety_state` | `std_msgs/String` | pub | One of `INIT`, `WAITING_FOR_TREE`, `READY`, `EXTEND_OK`, `HOLD_AT_DISTANCE`, `ALIGN_OK`, `LEVEL_OK`, `DOCKING_OK`, `EMERGENCY_STOP`, `INFEASIBLE_DOCK_HEIGHT` |

The custom message `BoomPlan` (defined under `harvester_boom_plan/msg/BoomPlan.msg`)
carries fields:

```
std_msgs/Header header
float64 tree_height_m
float64 docking_height_m
float64 trunk_distance_m            # horizontal in base_link X
float64 boom_angle_rad              # theta_b
float64 platform_level_rad          # theta_p (target platform world angle)
float64 leveling_angle_rad          # theta_L (correction applied to level joint)
float64 boom_extension_total_m      # e1+e2+e3+e4 target
float64[] boom_extension_per_stage_m # 4 entries, sum == boom_extension_total_m
float64 boom_horizontal_distance_m  # d_b
float64 docking_lower_angle_rad     # theta_d
string leveling_mode                # 'active' | 'passive' (see §2.5 step 5)
string phase                        # one of the safety states above
string notes
```

### 4.3 Logging and ROS-side state

- All outputs are published at the slowest input rate (10 Hz upper bound;
  default timer = 5 Hz).
- Use **the same names for the five fixed sensors** as
  `range_sensor_calibration.py` so the watchdog subscribes to the same
  topics without re-implementing calibration.
- Reuse the **TF2 lookup pattern** (dynamic TF at-or-before joint state stamp,
  static sub-chains cached). As noted in `TELEMETRY_HANDOFF.md`, Foxy + the
  current Gazebo bridge cannot do exact-stamp dynamic TF for arbitrary
  joints; we use `t = now()` and age-gate at 0.1 s, same approach as the
  dashboard.

---

## 5. Closed-form math (verified against the URDF)

> **Correction (2026-09-02):** the leveling formula `θ_L = -θ_b` below is
> **wrong**; the correct command is `θ_L = +θ_b` (see §13). The math below is
> preserved as originally written; substitute `+θ_b` and `h_p = 1.81 m` for the
> as-built result.

Given the URDF chain §2.1 and the symbols defined in §2.2/§2.3, the
forward kinematics of `c_channel_reference` expressed in `base_link` (turret
at 0 yaw, level joint at 0 rad, all four extensions at 0) is

```
x_C0 = L_fixed + L_plat = 2.4 + 1.02 = 3.42 m
z_C0 = h_p                = 1.76 m
```

For a generic configuration (turret at 0, level joint at `θ_p`, all four
extensions equal to `e/4` so the total telescope stroke is `e`):

```
x_C = (L_fixed + e) * cos θ_b + L_plat * cos(θ_p + θ_b)        (platform body rotation not leveled if θ_p ≠ -θ_b)
z_C = h_p + (L_fixed + e) * sin θ_b + L_plat * sin(θ_p + θ_b)
```

After `θ_L = -θ_b` (leveling correction applied to `platform_level_joint`):

```
x_C = (L_fixed + e) * cos θ_b + L_plat                              (eq. C-1)
z_C = h_p + (L_fixed + e) * sin θ_b                                 (eq. C-2)
```

Solving C-1 and C-2 for the unknowns `θ_b` and `e`, given the dock
position `(d_horiz, H_dock)` relative to `base_link`:

```
Δx = d_horiz - L_plat
Δz = H_dock - h_p
L_needed = sqrt(Δx^2 + Δz^2)                       (required boom length)
θ_b = atan2(Δz, Δx)                                (eq. C-3)
e   = max(0.0, L_needed - L_fixed)                 (eq. C-4, split evenly across 4 stages)
θ_L = -θ_b                                         (leveling correction)
```

If `L_needed > L_fixed + 9.6` the tree is unreachable with the current turret
position; the node emits `INFEASIBLE_DOCK_HEIGHT` and the operator must
reposition the harvester (`/harvester/cmd_vel`) closer.

The "before-extension" angle for the diagram:

```
θ_b(1) = asin(clamp((H_dock - h_p) / L_fixed, -1.0, 1.0))
```

— only meaningful if `|H_dock - h_p| ≤ L_fixed = 2.4`. After extension
(§2.4 Case I) the same equation `θ_b = atan2(Δz, Δx)` covers every reachable
geometry; the step-1 raise is only useful for the diagram.

After leveling and with `θ_d = 0`:

```
x_C_final = (L_fixed + e) * cos θ_b + L_plat     → d_horiz    (assert)
z_C_final = h_p + (L_fixed + e) * sin θ_b        → H_dock     (assert)
```

`θ_d` (extra boom-lower angle, state 4) is computed only when the platform
is still *above* the docking point after leveling — see §3.

---

## 6. Validation against `tree_scan_002` setup

Ground truth in the current simulation:

- Tree position: `world (8.5, 0, 0)`.
- `tree_base` is at z=0; trunk diameter 0.6 m nominal.
- Reference starting pose: harvester at world (0, 0, 0.05); `boom_turret=0`;
  `boom_elevation=0`; all `boom_extension_*=0`; `platform_level=0`.
- After running `tree_scan_sweeper`, `analyze_tree_scan_v2.py` reports
  `H ≈ 11.90 m`, crown base `9.00 m` against ground-truth 12.0 / 9.2.

Test rig for the new node:

```
Tree base           world (8.5, 0, 0)
Harvester base      world (0, 0, 0.05)        (no movement; turret centered)
d_horiz             8.5 - 1.05 - 0.18 + ... ≈ 7.27 m from boom pivot to trunk
                    (because turret origin is at base_link.X = -1.05 and the
                     elevation joint's origin is at X=0.18 in the turret frame,
                     giving pivot at base_link.X = -1.05 + 0.18 = -0.87 m in a
                     yaw-centered turret; if boom is also at zero elevation the
                     nominal reach axis points along base_link.X positive)
                    d_horiz(pivot → trunk) = (8.5 - (-0.87)) = 9.37 m

We use the simpler convention below — the node reads the trunk pose from
`trunk_center` and converts it into base_link.X via TF2 — not from a hand
computation. The numbers above are diagnostic only.
```

Validation cases the node must solve:

| Case | `H` (m) | `H_dock = H − 1.0` (m) | Expected `θ_b` (rad) | Expected `Σe` (m) |
|---:|---:|---:|---:|---:|
| `tree_scan_002` reported `H=11.90` → `H_dock=10.90` | 11.90 | 10.90 | `atan2(10.90 − 1.76, d_horiz − 1.02)` → ≈ `atan2(9.14, 8.35)` ≈ **0.832 rad** (≈ 47.7°) | `sqrt(8.35² + 9.14²) − 2.4` ≈ **10.42 m**, *exceeds max* `9.6 m → INFEASIBLE_DOCK_HEIGHT at d_horiz = 9.37` |
| Same tree, harvester at `world X=4.0` → `d_horiz=4.87 m` | 11.90 | 10.90 | `atan2(9.14, 3.85)` ≈ **1.172 rad** | `sqrt(3.85² + 9.14²) − 2.4` ≈ **8.27 m** — feasible |
| Ground-truth `H=12.0`, `d_horiz=4.87` → `H_dock=11.0` | 12.0 | 11.0 | `atan2(9.24, 3.85)` ≈ **1.176 rad** | `sqrt(3.85² + 9.24²) − 2.4` ≈ **8.39 m** — feasible |

Two implementation outcomes the validation will confirm:

1. The IK math is internally consistent: solve via `atan2`/`sqrt`, convert
   back through eq. C-1/C-2, and recover `(d_horiz, H_dock)` to within
   `1e-6 m`.
2. The `INFEASIBLE_DOCK_HEIGHT` outcome correctly fires when
   `Σe > 9.6 m`, with the message showing the exact deficit
   `(L_needed − (L_fixed + 9.6))` and the operator-action hint from §6.

The 5-range watchdog is validated separately by replaying the canonical
gzip recordings (no Gazebo required) and checking the phase transitions.

---

## 7. Implementation steps

### Step 1 — New package skeleton

Create `src/harvester_boom_plan/`:

```
src/harvester_boom_plan/
├── CMakeLists.txt
├── package.xml
├── setup.py
├── launch/
│   └── boom_plan.launch.py
├── msg/
│   └── BoomPlan.msg
├── harvester_boom_plan/
│   ├── __init__.py
│   ├── kinematics.py
│   ├── safety.py
│   └── boom_plan_node.py
└── test/
    ├── test_kinematics.py
    ├── test_safety.py
    └── test_boom_plan_node.py
```

No new package depends on Gazebo. The launch starts only the ROS 2 node
plus a TF2 listener; no `joint_state_publisher_gui`, no SDF, no plugin.

### Step 2 — `kinematics.py`

Pure-Python (no ROS). Reads the URDF once via `urdf_parser_py` (already
required by other workspace tooling) to extract:

- `boom_pivot_height_m = 1.02 + 0.74 = 1.76` (assert against URDF).
- `boom_fixed_length_m = 2.4` (`platform_level_joint` origin `xyz.x`).
- `platform_tail_m = 1.02` (`platform_fixed_joint` `xyz.x`).
- `extension_stroke_m = 9.6` (sum of the four prismatic limits).

Exposes:

```python
def forward_kinematics(theta_b, theta_L, e_total_m) -> np.ndarray:  # 3-vector
def inverse_kinematics(d_horiz_m, h_dock_m, platform_tail_m, ...) -> IKResult
def split_extension(e_total_m, n_stages=4) -> list[float]
```

Assertions inside inverse kinematics raise `InfeasibleDockHeight` when
`L_needed > L_fixed + extension_stroke_m` and the result includes
`deficit_m`, `needed_extension_m`, `recommended_advance_m` (so the
operator knows how far to drive the base forward).

### Step 3 — `safety.py`

State machine per §3 (two roles: **clearance/skew watchdog** +
**trunk-centering confirmation**), with thresholds sourced from a small YAML:

```yaml
# config/safety_thresholds.yaml
extend_safe_m: 0.6
extend_stop_m: 0.4
align_skew_m: 0.08
docking_gap_m: 0.18
emergency_m: 0.05
stale_receipt_timeout_s: 0.30
# centering gate: C-opening footprint, from CALIBRATION_FRAME_CONTRACT.md
c_open_half_width_m: 0.41     # 0.82 m opening / 2
platform_outer_radius_m: 1.20
```

Functions accept `(center, c45_l, c45_r, side_l, side_r, cutter_range)` and
return `(phase: str, centering: dict, notes: str)`. The `centering` dict
carries the reconstructed trunk centre (from the side pair + ±45° pair) and
its distance from the C-opening centreline. Stale data >
`stale_receipt_timeout_s` returns `WAITING_FOR_TREE` (or `INIT` on cold
start). Reuses no existing calibration logic because the figure explicitly
says "5-range sensors guidance safety" — the safety module is a *separate,
leaner* evaluator that reads raw range values, but its centering check
mirrors the side-pair geometry already established in
`range_sensor_calibration.py`.

### Step 4 — `boom_plan_node.py`

Subscribes to the inputs in §4.2, calls `kinematics.inverse_kinematics`,
calls `safety.evaluate`, publishes `BoomPlan` + `safety_state` at 5 Hz.

Crucial: **never publish to `/harvester/joint_commands`**. Following
`TELEMETRY_HANDOFF.md`:

> The telemetry gateway only subscribes to sensor and derived-perception
> topics. It must never publish a joint command, velocity command, TF
> transform, Gazebo service request, hydraulic command, PLC write, or
> solenoid command.

The same read-only constraint applies here. The plan is *advice* to the
operator/higher-level controller.

### Step 5 — Validation against `tree_scan_002`

We need a closed-loop test using the simulation:

1. **Replay path:** spin up the recording gateway (`replay.py`) reading
   `~/harvester_audits/tree_scan_002`; subscribe the new node to
   `v1/docking/trunk_estimate` and `v1/tree/docking_estimate` proxy topics.
2. **Live path:** start `gazebo_harvester_and_tree.launch.py`, run
   `tree_scan_sweeper.py` once; have the new node subscribe to the
   `/harvester/tree/docking_estimate` published by the offline analyzer.
3. **Closed-loop unit tests:** `test_kinematics.py` runs the §6 cases and
   asserts within `1e-6 m`; `test_safety.py` exercises the phase transitions
   with synthetic range sequences.

### Step 6 — Documentation

- New doc: `docs/BOOM_DOCK_PLAN.md` (mirrors `TREE_HEIGHT_SCAN.md`).
- Update `TREE_HEIGHT_SCAN.md`'s "Next steps" section to mark item 1 as
  done (this plan) and add a paragraph for items 2–3 from that file.
- Cross-link from `SIMULATION_HANDOFF.md` and `TELEMETRY_HANDOFF.md` to
  the new doc (the existing cross-link language does not need to change).

### Step 7 — Dashboard read-out

Once the node validates, add a small viewer in the existing dashboard
(showing `tree_height`, `H_dock`, `θ_b`, `Σe`, phase) by binding the
`BoomPlan` fields onto a new ZMQ channel. This is **explicitly out of scope
for this plan** — it is mentioned here only to make the eventual handoff
clear.

---

## 8. Risks and limits

1. **No real joint encoders** (per `TELEMETRY_HANDOFF.md`). The plan
   *computes* the joint targets from tree geometry; it cannot verify that
   the simulated or physical articulation actually reached those targets.
   That verification remains the operator's responsibility.
2. **The 5-range watchdog is phase-4 only.** When the boom is fully retracted
   and the platform is far from the trunk, no c-channel sensor has a return.
   `safety.py` must distinguish *no trunk in reach yet* from *trunk overshoot*.
3. **Leveling mechanism unconfirmed.** The plan supports both `active`
   (`θ_L = +θ_b` — corrected 2026-09-02, see §13) and `passive` (`θ_L = 0`,
   gravity stabilizer) via the `leveling_mode` config flag (default `active`
   for simulation). The correct real-machine mode must be confirmed against
   the actual harvester before any deployment; until then the node is
   simulation-only and the flag is surfaced in `BoomPlan.notes`.
4. **Trunk horizontal position** only valid when the side-pair estimate
   passes `range_sensor_calibration.py`'s gates. If the gates fail, fall
   back to the centroid of the left/right `/harvester/docking/range_hits/*`
   topics and warn `notes = "trunk_center stale, using hit midpoint"`.
5. **Beacon of `INFEASIBLE_DOCK_HEIGHT`** is safety-critical because the
   operator must not issue joint commands that push past the URDF
   kinematic limit. The `BoomPlan.msg` includes a "notes" string; the
   dashboard should flag this in red.

---

## 9. Acceptance criteria

- `cd ~/ros2_ws && source /opt/ros/foxy/setup.bash && source install/setup.bash && colcon build --packages-select harvester_boom_plan --symlink-install && PYTHONPATH=src/harvester_boom_plan python3 -m unittest discover src/harvester_boom_plan/test -v` passes.
- The unit tests in §6 return `θ_b` and `Σe` matching the table to within
  `1e-3 rad` and `1e-3 m`.
- A live simulation run with the harvester at `world X=4.0`, tree at
  `world X=8.5`, height estimate `11.90 m`, produces:
  - `BoomPlan.boom_angle_rad ≈ 1.172 rad`.
  - `BoomPlan.boom_extension_total_m ≈ 8.27 m` (≥ 0, ≤ 9.6).
  - `safety_state = READY → EXTEND_OK → ALIGN_OK → LEVEL_OK` after the
    operator moves the harvester to `d_horiz ≤ 4.87 m`.
- The centering gate is demonstrably **AND**-ed into `DOCKING_OK`: a
  synthetic test with clearances inside `DOCKING_GAP_M` but a reconstructed
  trunk centre outside the C-opening footprint returns `LEVEL_OK` (or
  `ALIGN_OK`), **not** `DOCKING_OK`.
- No code path publishes on `/harvester/joint_commands`,
  `/harvester/cmd_vel`, or any Gazebo service. The
  `gazebo_ros_five_range_safety_check` smoke test from the existing
  calibration contract (`ros2 run tf2_ros tf2_echo c_channel_reference
  left_side_range_sensor_link`) continues to pass.

---

## 10. Out of scope

- Hardware-implementation of any physical harvester.
- Force/cutting physics.
- Trunk-camera-based docking refinement (camera already exists; not on
  this plan's critical path).
- Dashboard ZMQ channel / canonical packet addition.

---

## 11. Copyable context for a future assistant

```text
I have a ROS 2 Foxy workspace at ~/ros2_ws. The new task is "boom distance,
boom angle, and boom extension estimation" — produce joint targets
(theta_b, theta_p, l_b + l_p) from a tree-height input and the trunk
horizontal position, using a closed-form inverse-kinematics solution
derived from the active URDF (oil_palm_harvester_kinematic.urdf). The
output is a custom msg /harvester/boom/plan. The node is read-only and
must never write /harvester/joint_commands. The plan is at
~/.kilo/plans/<timestamp>-boom-plan.md.
```

---

## 12. Final decisions and implementation outcome (2026-09-02)

### Docking point

- **Offset fixed at 2.0 m below trunk top** (user decision). For the reference
  tree (trunk top `z=12.0 m`, `tree_targets.yaml`), the docking height is
  **`H_dock = 10.0 m`** — above the crown base (9.2 m), below the upper-frond
  (11.35 m) and FFB (10.88 m) zones, so the c-channel grips clean trunk.

### Docking reference point (chosen)

- **Tree side:** trunk **centreline** at the docking height,
  `(trunk_x, trunk_y, H_dock)`.
- **Platform side:** `c_channel_reference` — the C-opening datum whose +X points
  through the opening toward the tree (co-located with the platform body). The
  IK goal places `c_channel_reference` exactly at the trunk-centre docking point.

### Implemented

New package `src/harvester_boom_plan/` (pure ament_python, merge-install):

| File | Role |
|---|---|
| `harvester_boom_plan/kinematics.py` | URDF-derived closed-form IK (`theta_b`, `e`, `theta_L`, `d_b`), `InfeasibleDock`, `split_extension`, `docking_lower_angle` |
| `harvester_boom_plan/safety.py` | 5-range watchdog + centering state machine (`SafetyConfig`, `evaluate`) |
| `harvester_boom_plan/boom_plan_node.py` | read-only estimator node (subs tree dock + trunk + 5 ranges; pubs `/harvester/boom/plan` + `/harvester/boom/safety_state`) |
| `harvester_boom_plan/tree_docking_estimate_node.py` | publishes static trunk-top estimate `/harvester/tree/docking_estimate` |
| `harvester_boom_plan/boom_docking_demo.py` | opt-in executor (dry-run default; `--execute` commands joints) |
| `launch/boom_plan.launch.py` | launches all three (as `python3 -m` to match merge-install) |
| `config/boom_docking.yaml` | `docking_offset_below_trunk_top_m: 2.0`, `leveling_mode: active`, safety thresholds |

The combined launch `gazebo_harvester_and_tree.launch.py` now includes this
stack by default (`boom_plan:=true`).

### Validated

> **Updated 2026-09-02 (see §13 for the full experiment record).** The
> no-Gazebo values below predate the live-simulation corrections: the leveling
> sign is now `+θ_b` (not `-θ_b`) and the boom-pivot world Z is `1.81 m` (not
> `1.76 m`). The corrected, measured result is `θ_b = 44.45°`, `level = +44.45°`,
> `ext = 9.296 m`, and a **0.000 m** docking error.

- 18 unit tests pass (`test_kinematics.py`, `test_safety.py`); the earlier
  no-Gazebo run (base at world X=0, trunk at world (8.5, 0)) produced
  `theta_b=44.62°`, `level=-44.62°`, `ext=9.331 m`, `docking_height_m=10.0`
  under the **pre-fix** sign/offset conventions (now superseded).
- FK recovers `c_channel_reference` at world `(8.5, 0, 10.0)` to `1e-6 m`
  with the corrected `BOOM_PIVOT_WORLD_Z` and `leveling = +θ_b`.
- Leveling supports `active` (default) and `passive`; centering is AND-ed into
  `DOCKING_OK` (off-centre or skewed pairs never admit `DOCKING_OK`).

### Next steps (not yet done)

1. ~~Run the full Gazebo+Rviz experiment with `--execute` and measure the achieved
   docking accuracy~~ **DONE — see §13.**
2. Feed a real LiDAR-derived trunk-top estimate into
   `/harvester/tree/docking_estimate` instead of the static ground truth.
3. Add a dashboard read-out / canonical ZMQ channel for `/harvester/boom/plan`.

---

## 13. Simulation experiment, optimization, and results (2026-09-02)

### Method

Ran the full headless Gazebo simulation (`gui:=false rviz:=false`,
`harvester_collision_mode:=off articulation_control_mode:=kinematic`), stopped
`joint_state_publisher_gui` (it overrides scripted joint commands), executed the
boom plan with `boom_docking_demo --execute`, and measured the achieved
`c_channel_reference` world pose two independent ways: (a) forward kinematics
from `/harvester/joint_states`, and (b) the `world → c_channel_reference` TF
transform (`tf2_ros tf2_echo`) as ground truth.

Target docking point: trunk centreline at `(8.5, 0, 10.0)` = trunk top 12.0 m −
2.0 m offset (user decision).

### Two root-cause bugs found and fixed during the experiment

1. **Leveling sign was inverted.** `boom_elevation_joint` axis is `(0,-1,0)`
   while `platform_level_joint` axis is `(0,+1,0)`. Commanding
   `platform_level_joint = -θ_b` left the platform tilted ~89° and the
   c-channel ~1.0 m off-target; commanding `+θ_b` levels it. Fixed:
   `leveling = +theta_b`.
2. **Base-link Z offset ignored.** The Gazebo base parks `base_link` at
   world `z = 0.05 m`, but the boom pivot height (1.76 m) is expressed in the
   `base_link` frame. The IK used `h_dock - 1.76` instead of
   `h_dock - (0.05 + 1.76)`, producing a 0.05 m vertical error. Fixed:
   `BOOM_PIVOT_WORLD_Z = 0.05 + 1.76 = 1.81 m`.

Also fixed during the experiment:

3. **Range-sensor "no return" sentinel** (`3.4e38` = `FLT_MAX`, `> max_range=3.0`)
   was being fed into the centering estimate (`diameter = 6.8e38`). The node now
   rejects `range > max_range` as "no return".
4. **Two-phase safety gating** in the demo: the raise/extend phase is open-loop
   (sensors legitimately out of range → `WAITING_FOR_TREE` must not block); only
   `EMERGENCY_STOP` and `INFEASIBLE_DOCK_HEIGHT` hard-block. `HOLD_AT_DISTANCE`
   correctly reflects the final approach.

### Results

| Metric | Before fix | After fix | Target |
|---|---|---|---|
| `c_channel_reference` X (m) | 7.494 | **8.500** | 8.500 |
| `c_channel_reference` Z (m) | 11.070 | **10.000** | 10.000 |
| 3D docking error (m) | ~1.47 | **0.000 (sub-mm)** | 0 |
| boom angle (deg) | 44.62 | **44.45** | — |
| boom extension (m) | 9.331 | **9.296** | — |
| level joint (deg) | −44.62 | **+44.45** | +θ_b |

Both independent measurements (FK from joint states and TF ground truth) report
`c_channel_reference = (8.500, -0.000, 10.000)` — **zero docking error** to the
printed precision (< 1 mm).

### Post-dock sensor observation

At the docked pose the forward `center_range = 0.233 m` (platform ~0.23 m from
the trunk — correct), while the ±45° and side sensors return "no return"
(`3.4e38`) because the trunk is now inside the C-opening and the side beams pass
the trunk. The safety state correctly reports `HOLD_AT_DISTANCE` (front clearance
0.233 < 0.4). This confirms the 5-range centering gate is meaningful during
**approach**, while the single centre sensor confirms **final contact**.

### Validation

- 18 unit tests pass (1 skip: URDF-consistency test under minimal PYTHONPATH;
  verified separately against the real URDF).
- `dock_measurer.py` (new) independently recomputes FK and reports 0.0000 m error.
- Plan JSON now emits `leveling_angle_deg == +boom_angle_deg` (corrected).
