from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory('oil_palm_harvester_description'))
    gazebo_share = Path(get_package_share_directory('gazebo_ros'))
    urdf = (share / 'urdf' / 'oil_palm_harvester_estimated.urdf').read_text()

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(gazebo_share / 'launch' / 'gazebo.launch.py')),
        launch_arguments={'world': str(share / 'worlds' / 'docking.world')}.items(),
    )
    state_pub = Node(package='robot_state_publisher', executable='robot_state_publisher',
                     parameters=[{'robot_description': urdf, 'use_sim_time': True}])
    spawn = Node(package='gazebo_ros', executable='spawn_entity.py',
                 arguments=['-topic', 'robot_description', '-entity', 'oil_palm_harvester', '-z', '0.05'],
                 output='screen')
    jsb = Node(package='controller_manager', executable='spawner',
               arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'])
    boom = Node(package='controller_manager', executable='spawner',
                arguments=['boom_controller', '--controller-manager', '/controller_manager'])
    return LaunchDescription([
        gazebo, state_pub, spawn,
        TimerAction(period=4.0, actions=[jsb]),
        TimerAction(period=5.0, actions=[boom]),
    ])
