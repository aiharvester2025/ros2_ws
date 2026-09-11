# Gazebo joint-command execution pitfalls

The single most time-consuming part of the session was getting the autonomous dock to actually
move the boom in Gazebo. The `harvester_kinematic_gazebo_plugin.cpp` has several non-obvious
behaviors. Read `src/oil_palm_harvester_description/src/harvester_kinematic_gazebo_plugin.cpp`
for the authoritative logic.

## Four independent failure modes (all were hit in sequence)

When "joints stay at 0" despite publishing commands, check each in order:

### 1. Gazebo must be unpaused

The plugin consumes commands in its `OnJointStateUpdate` callback, which only fires while Gazebo
is **unpaused**. A paused simulation silently drops joint commands. `ros2 topic hz` on the
command topic may also fail/return nothing because the plugin never processes. **Unpause Gazebo.**

### 2. The topic name is namespaced by the model

The plugin subscribes to `joint_commands` **relative to the model namespace**. The actual topic is
`/bal/joint_commands` (where `bal` is the model name — the service call returns `bal`), NOT
`/harvester/joint_commands` and NOT `/model/bal/joint_commands`. Publishing to the wrong topic
does nothing even though `ros2 topic list` may show a similarly-named topic.

The ROS-side orchestrator (`dock_orchestrator.py`) publishes to `/harvester/joint_commands` — a
**bridge/relay** must translate that to `/bal/joint_commands` (the plugin side). Know which side
of the bridge you are on before editing a topic name.

### 3. `enable_execution` service is one-shot and must precede the command

The plugin's `execute` flag starts `false`; the `enable_execution` service sets it `true`, and in
the **next** `OnJointStateUpdate` callback the plugin processes all queued commands and resets
`execute = false`. Consequences:

- Call `enable_execution` **before** publishing the joint command(s), or the command is ignored.
- It processes **all** queued commands in that single callback — so batch the full 7-DOF command
  into one `JointCommand`/`JointState` message with all `joint_names`/`joint_positions`, not one
  message per joint. Publishing joints one-at-a-time (the original bug) leaves joints stuck at 0.

### 4. Service name is also model-namespaced

The `enable_execution` service is likewise namespaced: `/bal/enable_execution` (or
`/model/<name>/enable_execution` depending on the plugin's node setup — verify with
`ros2 service list | grep -i execut`). Calling the un-namespaced service returns an error or a
different service. Check the full service list rather than guessing.

## The `ArticulationGrasp` message

The `enable_execution` service uses a custom `ArticulationGrasp` message with many fields, but the
plugin only reads `joint_names` and `joint_positions` — those are the only fields that matter when
calling it programmatically.

## Debugging checklist

```bash
ros2 service list | grep -i execut        # find the real service name
ros2 topic list | grep joint              # find the real command topic
ros2 topic echo /bal/joint_commands       # confirm your publisher actually reaches the plugin
ros2 topic hz /bal/joint_commands         # (empty if Gazebo paused)
```

- Confirm the service **response** (not just that the call didn't throw): the plugin returns the
  model name (`bal`) on success.
- Re-run the same single command after each hypothesis; do not stack edits. The session burned
  many iterations by changing topic name, then service name, then adding delays, one at a time —
  serialize the fixes.

## Build / run conventions (merge-install layout)

- Build: `colcon build --packages-select <pkg> --merge-install --symlink-install` then
  `source install/setup.bash`.
- Scripts are launched as `python3 -m <package>.<module>` (not `ros2 run` console scripts), because
  `--merge-install` puts console scripts in `install/bin`, which `launch_ros.actions.Node` does not
  search. The launch files use `ExecuteProcess` with `python3 -m ...`.
- `ros2 pkg executables` returning empty is a known misleading quirk in this layout — the console
  script exists in `install/lib/<pkg>/` and runs fine when invoked directly. Don't chase that.

## Read-only safety boundary (do not violate)

`harvester_boom_plan` (`boom_plan_node`, `tree_docking_estimate_node`) are **strictly read-only**:
they never publish to `/harvester/joint_commands`, `/harvester/cmd_vel`, or call any Gazebo
service. Only `harvester_dock`'s orchestrator and `boom_docking_demo --execute` may command
joints. Keep that separation — it is a hard constraint in `docs/TELEMETRY_HANDOFF.md`.
