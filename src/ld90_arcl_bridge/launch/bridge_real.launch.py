from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg = get_package_share_directory('ld90_arcl_bridge')
    cfg = os.path.join(pkg, 'config', 'bridge_config.yaml')
    urdf = os.path.join(pkg, 'urdf', 'ld90_base.urdf')
    rviz = os.path.join(pkg, 'rviz', 'ld90_bridge.rviz')

    with open(urdf, 'r') as f:
        robot_desc = f.read()

    # Tip: set the real LD90 IP/password in the YAML before launching.
    return LaunchDescription([
        Node(
            package='ld90_arcl_bridge',
            executable='ld90_bridge',
            name='ld90_bridge',
            parameters=[cfg],
            output='screen'
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_desc}]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz]
        ),
    ])
