"""PP exploration and return in maze2."""
from importlib.util import module_from_spec, spec_from_file_location
from pathlib import Path
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument,GroupAction,IncludeLaunchDescription,OpaqueFunction,
                            SetEnvironmentVariable,UnsetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration,PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share=Path(get_package_share_directory('maze_bringup'))
    spec = spec_from_file_location('maze_bringup_simulation', share / 'launch_support/simulation.py')
    simulation = module_from_spec(spec)
    spec.loader.exec_module(simulation)
    slam=Path(get_package_share_directory('slam_toolbox'))
    try:
        terminal_device=os.ttyname(0)
    except OSError:
        terminal_device='/dev/tty'
    return LaunchDescription([
        DeclareLaunchArgument('world',default_value=str(share/'world/maze2.world')),
        DeclareLaunchArgument('x',default_value='-2.275'),
        DeclareLaunchArgument('y',default_value='-3.8'),
        DeclareLaunchArgument('z',default_value='0.01'),
        DeclareLaunchArgument('yaw',default_value='1.5707963267948966'),
        DeclareLaunchArgument('gazebo_gui',default_value='true'),
        DeclareLaunchArgument('use_rviz',default_value='true'),
        DeclareLaunchArgument('software_rendering',default_value='true'),
        DeclareLaunchArgument('guard_linear_speed',default_value='0.65'),
        DeclareLaunchArgument('mapping',default_value='true'),
        DeclareLaunchArgument('planning_frame',default_value=PythonExpression([
            "'map' if '",LaunchConfiguration('mapping'),"'.lower() == 'true' else 'odom'"])),
        DeclareLaunchArgument('start_autonomy',default_value='true'),
        DeclareLaunchArgument('wait_for_start',default_value='true',
                              description='Keep robot stopped until start + Enter in the launch terminal'),
        DeclareLaunchArgument('terminal_device',default_value=terminal_device),
        DeclareLaunchArgument('controller_mode',default_value='pp'),
        DeclareLaunchArgument('motion_speed',default_value='-1.0'),
        DeclareLaunchArgument('planner_params',default_value=str(share/'config/polyline.yaml')),
        DeclareLaunchArgument('slam_params',default_value=str(share/'config/slam.yaml')),
        UnsetEnvironmentVariable('GTK_PATH'),UnsetEnvironmentVariable('GTK_EXE_PREFIX'),
        SetEnvironmentVariable('LIBGL_ALWAYS_SOFTWARE','1',condition=IfCondition(LaunchConfiguration('software_rendering'))),
        GroupAction(actions=[OpaqueFunction(function=simulation.setup)]),
        Node(package='maze_bringup',executable='maze_odometry.py',parameters=[{'use_sim_time':True}],output='screen'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(slam/'launch/online_async_launch.py')),
            launch_arguments={'use_sim_time':'true','slam_params_file':LaunchConfiguration('slam_params')}.items(),
            condition=IfCondition(LaunchConfiguration('mapping'))),
        Node(package='maze_bringup',executable='polyline_follower.py',
             parameters=[LaunchConfiguration('planner_params'),{'planning_frame':ParameterValue(
                 LaunchConfiguration('planning_frame'),value_type=str),
                 'controller_mode':ParameterValue(LaunchConfiguration('controller_mode'),value_type=str),
                 'wait_for_start':ParameterValue(LaunchConfiguration('wait_for_start'),value_type=bool),
                 'terminal_device':ParameterValue(LaunchConfiguration('terminal_device'),value_type=str),
                 'motion_speed':ParameterValue(LaunchConfiguration('motion_speed'),value_type=float)}],output='screen',
             condition=IfCondition(LaunchConfiguration('start_autonomy'))),
        Node(package='rviz2',executable='rviz2',arguments=['-d',str(share/'rviz/polyline.rviz'),
                                                      '-f',LaunchConfiguration('planning_frame')],
             parameters=[{'use_sim_time':True}],condition=IfCondition(LaunchConfiguration('use_rviz'))),
    ])
