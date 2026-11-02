"""PID wall tracing and exit stopping in the merged maze."""
from importlib.util import module_from_spec, spec_from_file_location
from pathlib import Path
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, OpaqueFunction,
                            SetEnvironmentVariable, UnsetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = Path(get_package_share_directory('maze_wall_pid'))
    spec = spec_from_file_location('maze_wall_pid_simulation', share / 'launch_support/simulation.py')
    simulation = module_from_spec(spec)
    spec.loader.exec_module(simulation)
    slam = Path(get_package_share_directory('slam_toolbox'))
    try:
        terminal_device = os.ttyname(0)
    except OSError:
        terminal_device = '/dev/tty'
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value=str(share/'world/merged_maze.world')),
        DeclareLaunchArgument('x', default_value='-2.45'),
        DeclareLaunchArgument('y', default_value='0.6701'),
        DeclareLaunchArgument('z', default_value='0.01'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        DeclareLaunchArgument('gazebo_gui', default_value='true'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('software_rendering', default_value='true'),
        DeclareLaunchArgument('mapping', default_value='false', description='SLAM for display; motion does not require a map'),
        DeclareLaunchArgument('start_autonomy', default_value='true'),
        DeclareLaunchArgument('wait_for_start', default_value='true'),
        DeclareLaunchArgument('terminal_device', default_value=terminal_device),
        DeclareLaunchArgument('debug_window', default_value='true'),
        DeclareLaunchArgument('rviz_config', default_value=str(share/'rviz/merged_left2.rviz')),
        DeclareLaunchArgument('wall_params', default_value=str(share/'config/merged_left2.yaml')),
        DeclareLaunchArgument('guard_linear_speed', default_value='1.5'),
        DeclareLaunchArgument('slam_params', default_value=str(share/'config/slam.yaml')),
        UnsetEnvironmentVariable('GTK_PATH'),
        UnsetEnvironmentVariable('GTK_EXE_PREFIX'),
        SetEnvironmentVariable('LIBGL_ALWAYS_SOFTWARE','1',condition=IfCondition(LaunchConfiguration('software_rendering'))),
        GroupAction(actions=[OpaqueFunction(function=simulation.setup)]),
        Node(package='maze_wall_pid',executable='maze_odometry.py',
             parameters=[{'use_sim_time':True}],output='screen'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(slam/'launch/online_async_launch.py')),
                                 launch_arguments={'use_sim_time':'true',
                                                   'slam_params_file':LaunchConfiguration('slam_params')}.items(),
                                 condition=IfCondition(LaunchConfiguration('mapping'))),
        GroupAction(actions=[
            Node(package='maze_wall_pid', executable='left2_lidar',
                 parameters=[LaunchConfiguration('wall_params'),{
                     'wait_for_start':ParameterValue(LaunchConfiguration('wait_for_start'),value_type=bool),
                     'enabled':ParameterValue(PythonExpression([
                         "'",LaunchConfiguration('wait_for_start'),"'.lower() != 'true'"]),value_type=bool),
                     'debug_window':ParameterValue(LaunchConfiguration('debug_window'),value_type=bool),
                 }],output='screen'),
            Node(package='maze_wall_pid',executable='wall_start.py',
                 parameters=[{'terminal_device':ParameterValue(LaunchConfiguration('terminal_device'),value_type=str)}],
                 output='screen',condition=IfCondition(LaunchConfiguration('wait_for_start'))),
        ], condition=IfCondition(LaunchConfiguration('start_autonomy'))),
        Node(package='rviz2',executable='rviz2',arguments=['-d',LaunchConfiguration('rviz_config')],
             parameters=[{'use_sim_time':True}], condition=IfCondition(LaunchConfiguration('use_rviz'))),
    ])
