from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory('oil_palm_harvester_description'))
    urdf = (share / 'urdf' / 'oil_palm_harvester_kinematic.urdf').read_text()
    # Use the script from the package source path so it runs without requiring install
    src_dir = Path(__file__).resolve().parent.parent
    publisher_script = src_dir / 'scripts' / 'publish_urdf.py'
    urdf_path = src_dir / 'urdf' / 'oil_palm_harvester_kinematic.urdf'
    return LaunchDescription([
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': urdf, 'use_sim_time': False}]),
        Node(package='joint_state_publisher_gui', executable='joint_state_publisher_gui'),
        # Publish the URDF on the /robot_description topic (transient local)
        ExecuteProcess(
            cmd=['python3', str(publisher_script), str(urdf_path)],
            output='screen'
        ),
        Node(package='rviz2', executable='rviz2', output='screen'),
    ])
