from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    param_file = os.path.expanduser(
        '~/Desktop/trajectory_optimizer/params/optimizer.yaml'
    )

    return LaunchDescription([
        Node(
            package='trajectory_optimizer',
            executable='optimizer_node',
            name='optimizer',
            output='screen',
            parameters=[param_file]
        )
    ])