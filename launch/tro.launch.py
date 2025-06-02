from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    param_file = os.path.expanduser(
        '~/Desktop/trajectory_optimizer/params/tro.yaml'
    )

    return LaunchDescription([
        Node(
            package='trajectory_optimizer',
            executable='tro_node',
            name='tro',
            parameters=[param_file],
            output='screen'
        )
    ])