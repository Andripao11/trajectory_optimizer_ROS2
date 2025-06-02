from launch import LaunchDescription
from launch.actions import TimerAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_traj_opt = get_package_share_directory('trajectory_optimizer')

    # Nodo midline lanciato subito
    midline_node = Node(
        package='trajectory_optimizer',
        executable='midline_node',
        name='midline_node',
        output='screen',
        parameters=[os.path.join(pkg_traj_opt, 'params', 'gro.yaml')]
    )

    # Optimizer lanciato dopo 2s
    optimizer_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_traj_opt, 'launch', 'optimizer.launch.py')
        )
    )

    delayed_optimizer = TimerAction(
        period=5.0,
        actions=[optimizer_launch]
    )

    return LaunchDescription([
        midline_node,
        delayed_optimizer
    ])