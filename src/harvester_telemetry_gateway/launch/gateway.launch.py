from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from pathlib import Path


def generate_launch_description():
    share = Path(get_package_share_directory('harvester_telemetry_gateway'))
    default_config = str(share / 'config' / 'gateway.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=default_config),
        # Existing simulation scripts use the active ``python3`` command so
        # they run in the user's ROS/Conda environment.  Do the same here:
        # generated ament Python entry points are pinned to /usr/bin/python3,
        # which lacks this Xavier's MessagePack/ZeroMQ modules.
        ExecuteProcess(
            cmd=['python3', '-m', 'harvester_telemetry_gateway.gateway_node',
                 '--ros-args', '--params-file', LaunchConfiguration('config')],
            output='screen',
        ),
    ])
