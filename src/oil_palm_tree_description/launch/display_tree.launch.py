from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from pathlib import Path

def generate_launch_description():
    pkg = Path(get_package_share_directory('oil_palm_tree_description'))
    urdf = (pkg / 'urdf' / 'oil_palm_tree_lowpoly.urdf').read_text()
    rviz = str(pkg / 'rviz' / 'oil_palm_tree.rviz')
    return LaunchDescription([
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': urdf}]),
        Node(package='rviz2', executable='rviz2', arguments=['-d', rviz])
    ])
