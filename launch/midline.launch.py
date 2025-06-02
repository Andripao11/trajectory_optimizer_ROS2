from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    param_file = os.path.expanduser(
        '~/Desktop/trajectory_optimizer/params/gro.yaml'
    )

    return LaunchDescription([
        Node(
            package='trajectory_optimizer',
            executable='midline_node',  # conferma che sia questo il nome esatto
            name='midline_node',
            output='screen',
            parameters=[param_file]
        )
    ])