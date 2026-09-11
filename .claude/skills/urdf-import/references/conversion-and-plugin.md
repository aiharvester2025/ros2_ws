# Conversion pipeline + plugin/launch constraints

Source of truth: `launch/gazebo_harvester_and_tree.launch.py`,
`src/harvester_kinematic_gazebo_plugin.cpp`.

## The `gz sdf -p` conversion path

The combined launch does NOT use `spawn_entity.py` (Foxy's large-URDF spawn path was unreliable). It:

1. Runs `gz sdf -p <urdf>` to convert the active URDF to SDF at launch time.
2. Writes the result to `/tmp/oil_palm_harvester_dynamic_scene.world`.
3. Rewrites every `model://oil_palm_harvester_description/...` URI to an absolute `file://` path in the
   installed `share/` directory.
4. Sets `<static>false</static>`, `<kinematic>true</kinematic>` on `base_link`, and disables gravity on
   every link.
5. Inserts the `harvester_kinematic_gazebo_bridge` plugin with `articulation_control_mode`.
6. Includes the static tree at `(8.5, 0, 0)` and a local `GAZEBO_MODEL_DATABASE_URI`.

Consequences for a real URDF:
- The URDF must be **convertible by `gz sdf -p`** with no errors. Validate offline first:
  `gz sdf -p path/to/new.urdf > /dev/null` (and `check_urdf path/to/new.urdf`).
- Meshes referenced by `package://` under this package's `meshes/` resolve automatically (they get
  rewritten). Meshes referenced by relative path or a different `package://` need the corresponding
  `GAZEBO_MODEL_PATH` entry.
- Fixed-joint lumping: Gazebo Classic's converter lumps fixed joints into a single link; the plugin's
  `GetScopedName()` vs `GetName()` handling already tolerates this — but verify joint names survive
  conversion (see below).

## Plugin name-coupling (the thing that breaks silently)

`harvester_kinematic_gazebo_plugin.cpp`:

- Builds `controller_joint_names_[joint->GetName()] = joint->GetScopedName()` on load.
- Special-cases **exactly one** joint by name: `kTurretJointName = "boom_turret_joint"` (driven via
  `SetPosition` on the turret subtree instead of generic `SetPositionTarget`).
- On `/harvester/joint_commands`, looks up `controller_joint_names_.find(message->name[index])` — an
  **unknown joint name is silently ignored** (`continue`), so a renamed joint just doesn't move.
- Publishes `world -> base_link` TF with hardcoded `"world"` / `"base_link"` frame names.

So a real URDF must:
1. Keep the root link named `base_link`.
2. Keep every joint name identical to the current set (the turret especially, but also the extension
   and arm joints the bridge ramps).
3. Keep `boom_turret_joint` as a **revolute** joint (the plugin's `SetPosition` path assumes revolute
   semantics).

If a rename is genuinely required (e.g. the vendor names differ), edit the plugin's joint-name matching
and `kTurretJointName` in the same change, and re-test the whole control path.

## Collision / physics mode

The plugin is a **kinematic articulation** bridge, not a force controller. With
`harvester_collision_mode:=off` (default) all harvester collision bodies are removed from the generated
SDF. Adding real masses/inertials therefore has **no effect** until:

- `harvester_collision_mode:=on` is enabled (which the handoff says to keep off until a proper physical
  base + joint controller exists), AND
- the physical model is actually run under `ros2_control` / force dynamics.

Do not enable collision mode just because the real URDF has inertials — the flying-robot /
`ODESliderJoint` instability is a direct consequence of driving a physics model with a kinematic
position loop.

## What the `oil_palm_harvester_estimated.urdf` template already shows

It demonstrates the shape of a physical model: `<inertial>` on every link (base 3100 kg, wheels 85 kg,
etc.), `<collision>` boxes, and `ros2_control` `transmission` blocks with
`hardware_interface/PositionJointInterface`. Use it as a reference for where inertials/collisions go,
but note its `gazebo_ros2_control` plugin path is **not** wired into the combined launch and must not
be silently enabled.
