"""Staged A*/B-spline reference and differential MPC, with PP comparison mode."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    share=Path(get_package_share_directory('maze_bringup'))
    return LaunchDescription([
        DeclareLaunchArgument('controller_mode',default_value='mpc',
                              description='mpc, optimized_pp, or pp baseline'),
        DeclareLaunchArgument('motion_speed',default_value='0.60'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(share/'launch/maze2.launch.py')),
            launch_arguments={key:LaunchConfiguration(key) for key in ('controller_mode','motion_speed')}.items()),
    ])
