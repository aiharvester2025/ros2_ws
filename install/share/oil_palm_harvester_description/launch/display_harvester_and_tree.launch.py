from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():
    harvester_share = Path(get_package_share_directory('oil_palm_harvester_description'))
    tree_share = Path(get_package_share_directory('oil_palm_tree_description'))
    publisher_script = Path(__file__).resolve().parent.parent / 'scripts' / 'publish_urdf.py'

    harvester_urdf = (harvester_share / 'urdf' / 'oil_palm_harvester_kinematic.urdf').read_text()
    tree_urdf = (tree_share / 'urdf' / 'oil_palm_tree_lowpoly.urdf').read_text()

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='harvester_state_publisher',
            parameters=[{'robot_description': harvester_urdf, 'use_sim_time': False}],
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='tree_state_publisher',
            parameters=[{'robot_description': tree_urdf, 'use_sim_time': False}],
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='harvester_to_tree_tf',
            arguments=['5.0', '0.0', '0.0', '0', '0', '0', 'base_link', 'tree_base'],
        ),
        ExecuteProcess(
            cmd=['python3', str(publisher_script), str(harvester_share / 'urdf' / 'oil_palm_harvester_kinematic.urdf'), '/robot_description', 'harvester_description_publisher'],
            output='screen',
        ),
        ExecuteProcess(
            cmd=['python3', str(publisher_script), str(tree_share / 'urdf' / 'oil_palm_tree_lowpoly.urdf'), '/tree_description', 'tree_description_publisher'],
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', str(harvester_share / 'rviz' / 'harvester_tree_combined.rviz')],
            output='screen',
        ),
    ])
