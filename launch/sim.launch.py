from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='cbf_pc_selector',
            executable='pc_selector_node',
            name='cbf_pc_selector',
            parameters=[{
                'config': os.path.join(
                    get_package_share_directory('cbf_pc_selector'),
                    'config', 'sim.yaml'
                ),
            }],
            output='screen'
        )
    ])
