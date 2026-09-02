"""Launch the read-only boom docking-plan stack.

This launches three processes:
  1. tree_docking_estimate_node  -- publishes /harvester/tree/docking_estimate
  2. boom_plan_node              -- read-only IK + safety estimator
  3. boom_docking_demo           -- dry-run executor (opt-in --execute)

None of these nodes publishes a joint command unless the demo is started with
--execute.  The estimator itself is strictly read-only.

The nodes are launched as ``python3 -m harvester_boom_plan.<module>`` to match
the workspace's ``--merge-install`` layout (console scripts live in
``install/bin``, which launch_ros.actions.Node does not search).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration


def _python_module(module: str, *args: str) -> list:
    """Build a ``python3 -m <module>`` command with optional trailing args."""
    return ['python3', '-m', module, *args]


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory("harvester_boom_plan"),
        "config", "boom_docking.yaml")

    config_file = LaunchConfiguration("config_file")
    declare_config = DeclareLaunchArgument(
        "config_file",
        default_value=default_config,
        description="Path to boom_docking.yaml")

    execute = LaunchConfiguration("execute")
    declare_execute = DeclareLaunchArgument(
        "execute", default_value="false",
        description="Pass --execute to the demo to actually command joints")

    trunk_top = LaunchConfiguration("trunk_top_height_m")
    declare_trunk_top = DeclareLaunchArgument(
        "trunk_top_height_m", default_value="12.0",
        description="Trunk-top world height for the static docking estimate")

    trunk_x = LaunchConfiguration("trunk_x_m")
    declare_trunk_x = DeclareLaunchArgument(
        "trunk_x_m", default_value="8.5",
        description="Trunk axis world X")

    tree_estimate = ExecuteProcess(
        cmd=_python_module(
            "harvester_boom_plan.tree_docking_estimate_node",
            "--ros-args",
            "-p", ["trunk_top_height_m:=", trunk_top],
            "-p", ["trunk_x_m:=", trunk_x],
            "-p", "trunk_y_m:=0.0",
        ),
        output="screen",
    )

    plan_node = ExecuteProcess(
        cmd=_python_module(
            "harvester_boom_plan.boom_plan_node",
            "--ros-args",
            "-p", ["config_file:=", config_file],
        ),
        output="screen",
    )

    # Dry-run by default (execute:=false); with execute:=true the demo commands
    # the boom joints.  This is a conscious, opt-in choice.
    demo_dry_run = ExecuteProcess(
        cmd=_python_module("harvester_boom_plan.boom_docking_demo"),
        condition=UnlessCondition(execute),
        output="screen",
    )
    demo_execute = ExecuteProcess(
        cmd=_python_module("harvester_boom_plan.boom_docking_demo", "--execute"),
        condition=IfCondition(execute),
        output="screen",
    )

    return LaunchDescription([
        declare_config,
        declare_execute,
        declare_trunk_top,
        declare_trunk_x,
        tree_estimate,
        plan_node,
        demo_dry_run,
        demo_execute,
    ])
