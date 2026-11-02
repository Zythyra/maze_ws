"""Internal Gazebo setup for the controller entry point."""
from pathlib import Path
import tempfile
import xml.etree.ElementTree as ET

import xacro
from ament_index_python.packages import get_package_share_directory
from launch.actions import (EmitEvent, ExecuteProcess,
                            GroupAction, IncludeLaunchDescription, LogInfo, OpaqueFunction,
                            RegisterEventHandler, SetEnvironmentVariable, UnsetEnvironmentVariable)
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def after_success(process, actions, stage):
    def on_exit(event, context):
        if context.is_shutdown:
            return []
        if event.returncode != 0:
            reason = f'{stage} failed (exit {event.returncode})'
            return [LogInfo(msg=reason), EmitEvent(event=Shutdown(reason=reason))]
        return actions
    return RegisterEventHandler(OnProcessExit(target_action=process, on_exit=on_exit))


def setup(context):
    share = Path(get_package_share_directory('maze_bringup'))
    gazebo = Path(get_package_share_directory('gazebo_ros'))
    tb3_gazebo = Path(get_package_share_directory('turtlebot3_gazebo'))
    tb3_description = Path(get_package_share_directory('turtlebot3_description'))
    enabled = lambda key: LaunchConfiguration(key).perform(context).lower() == 'true'
    description = xacro.process_file(str(tb3_description / 'urdf/turtlebot3_burger.urdf')).toxml()

    # Keep the official source untouched. Apply project topic/odometry settings
    # to a temporary SDF so every startup uses the same Burger model.
    model = ET.parse(tb3_gazebo / 'models/turtlebot3_burger/model.sdf')
    # The vendored Gazebo SDF has legacy CAD inertias hundreds of times larger
    # than the matching official URDF. They cause yaw lag and wheel slip even
    # with accurate commands. Use the URDF tensors in the temporary model;
    # existing SDF inertial poses already use the matching component axes.
    urdf = ET.fromstring(description)
    for name in ('base_link', 'wheel_left_link', 'wheel_right_link'):
        tensor = urdf.find(f"link[@name='{name}']/inertial/inertia")
        sdf_tensor = model.find(f"model/link[@name='{name}']/inertial/inertia")
        for component, value in tensor.attrib.items():
            sdf_tensor.find(component).text = value
    # The legacy SDF explicitly marks its 100000 friction coefficients as
    # unreliable. Keep finite tire friction for the corrected inertias.
    for name in ('wheel_left_link', 'wheel_right_link'):
        friction=model.find(f"model/link[@name='{name}']/collision/surface/friction/ode")
        for component in ('mu','mu2'):
            friction.find(component).text='1.0'
    # Resolve tire/ball penetration with damping rather than a deep, weakly
    # damped contact layer that creeps while wheel targets are zero.
    for name in ('wheel_left_link', 'wheel_right_link', 'caster_back_link'):
        contact=model.find(f"model/link[@name='{name}']/collision/surface/contact/ode")
        for component,value in (('kd','100'),('min_depth','0.0001'),('max_vel','0.1')):
            contact.find(component).text=value
    # The 5 mm support ball's CAD tensor is 0.001 kg*m^2, about 20,000
    # times its solid-sphere inertia. Its ball joint must spin freely as the
    # chassis moves; that tensor otherwise makes the support skid and drag.
    caster = model.find("model/link[@name='caster_back_link']")
    radius = float(caster.find('collision/geometry/sphere/radius').text)
    mass = float(caster.find('inertial/mass').text)
    for component in caster.find('inertial/inertia'):
        component.text = str(.4 * mass * radius * radius) if component.tag in ('ixx', 'iyy', 'izz') else '0'
    # SDF supplies 1 kg / 1 kg*m^2 defaults when inertial is absent, even for
    # coordinate-only links. Confirmed in Gazebo's loaded model: footprint and
    # IMU each added a phantom inertia far larger than the whole real chassis.
    # Explicit small positive values keep these fixed frames dynamically inert.
    for link in model.findall('model/link'):
        if link.find('inertial') is not None:
            continue
        inertial = ET.SubElement(link, 'inertial')
        ET.SubElement(inertial, 'mass').text = '0.0001'
        tensor = ET.SubElement(inertial, 'inertia')
        for component in ('ixx', 'ixy', 'ixz', 'iyy', 'iyz', 'izz'):
            ET.SubElement(tensor, component).text = '1e-7' if component in ('ixx', 'iyy', 'izz') else '0'
    drive = model.find("model/plugin[@name='turtlebot3_diff_drive']")
    ros = drive.find('ros')
    if ros is None:
        ros = ET.SubElement(drive, 'ros')
    ET.SubElement(ros, 'remapping').text = 'cmd_vel:=/burger/cmd_vel'
    odometry = drive.find('odometry_source')
    if odometry is None:
        odometry = ET.SubElement(drive, 'odometry_source')
    odometry.text = '0'  # Encoder odometry, with origin at the entrance.
    # Integrated controllers always use the fused odometry node.
    ET.SubElement(ros, 'remapping').text = 'odom:=/wheel/odom'
    drive.find('publish_odom_tf').text = 'false'
    drive.find('odometry_frame').text = 'wheel_odom'
    model.find("model/link[@name='base_scan']/sensor/visualize").text = 'false'
    model.find("model/link[@name='base_scan']/sensor/update_rate").text = '10'
    resources = tempfile.TemporaryDirectory(prefix='maze_burger_')
    model_file = Path(resources.name) / 'burger.sdf'
    model.write(model_file, encoding='unicode', xml_declaration=False)
    # An empty database URI becomes "//database.config" in Gazebo Classic.
    # With the desktop HTTP proxy this can stall GUI initialization: physics
    # and ROS keep running, but the client displays the initial robot pose and
    # blank time statistics. Use an actual local, empty catalogue instead.
    (Path(resources.name) / 'database.config').write_text(
        '<?xml version="1.0"?><database><name>Maze local models</name>'
        '<models/></database>\n', encoding='utf-8')
    # The Insert panel scans every child of GAZEBO_MODEL_PATH as a model.
    # A ROS install/share root or the autorace collection is not a model and
    # produces Missing model.config errors. Expose only this scene's models.
    client_models = Path(resources.name) / 'models'
    client_models.mkdir()
    for directory in (tb3_gazebo / 'models/turtlebot3_burger',
                      tb3_gazebo / 'models/turtlebot3_common',
                      Path('/usr/share/gazebo-11/models/ground_plane'),
                      Path('/usr/share/gazebo-11/models/sun')):
        if (directory / 'model.config').is_file():
            (client_models / directory.name).symlink_to(directory, target_is_directory=True)

    # The GUI starts from an OnProcessExit callback after the include's launch
    # environment scope has ended. Capture its complete environment now:
    # gazebo_ros/gzclient.launch.py otherwise rebuilds model paths from the
    # launch Python process's os.environ and restores unrelated workspaces;
    # the scoped local database URI is also lost, triggering online lookups.
    client_env = dict(context.environment)
    for key in ('HTTP_PROXY', 'HTTPS_PROXY', 'ALL_PROXY',
                'http_proxy', 'https_proxy', 'all_proxy'):
        client_env.pop(key, None)
    for key in ('GTK_PATH', 'GTK_EXE_PREFIX'):
        if '/snap/' in client_env.get(key, ''):
            client_env.pop(key)
    client_env.update(GAZEBO_MODEL_PATH=str(client_models),
                      GAZEBO_MODEL_DATABASE_URI=Path(resources.name).as_uri(),
                      TURTLEBOT3_MODEL='burger', ALSOFT_DRIVERS='null')
    if enabled('software_rendering'):
        client_env['LIBGL_ALWAYS_SOFTWARE'] = '1'

    def cleanup(context, *args, **kwargs):
        resources.cleanup()
        return []

    actions = [UnsetEnvironmentVariable(key) for key in ('GTK_PATH', 'GTK_EXE_PREFIX')
               if '/snap/' in context.environment.get(key, '')]
    model_paths = [str(tb3_gazebo / 'models'), '/usr/share/gazebo-11/models',
                   context.environment.get('GAZEBO_MODEL_PATH', '')]
    actions += [
        SetEnvironmentVariable('TURTLEBOT3_MODEL', 'burger'),
        SetEnvironmentVariable('GAZEBO_MODEL_DATABASE_URI', Path(resources.name).as_uri()),
        SetEnvironmentVariable('GAZEBO_MODEL_PATH', ':'.join(p for p in model_paths if p)),
        SetEnvironmentVariable('ALSOFT_DRIVERS', 'null'),
        RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(function=cleanup)])),
    ]
    if enabled('software_rendering'):
        actions.append(SetEnvironmentVariable('LIBGL_ALWAYS_SOFTWARE', '1'))

    spawn = Node(package='gazebo_ros', executable='spawn_entity.py',
                 arguments=['-file', str(model_file), '-entity', 'turtlebot3_burger', '-timeout', '90',
                            '-x', LaunchConfiguration('x'), '-y', LaunchConfiguration('y'),
                            '-z', LaunchConfiguration('z'), '-Y', LaunchConfiguration('yaw')],
                 output='screen')
    unpause = ExecuteProcess(cmd=['ros2', 'service', 'call', '/unpause_physics',
                                 'std_srvs/srv/Empty', '{}'], output='screen')
    ready = [LogInfo(msg='Burger ready at maze entrance: /cmd_vel, /odom, /scan')]
    if enabled('gazebo_gui'):
        # Connect the client only after the world/model/services are ready.
        # Starting it alongside a large world's server can leave the initial
        # scene visible but live pose/statistics subscriptions uninitialized.
        ready.append(ExecuteProcess(
            cmd=['gzclient', '--gui-client-plugin=libgazebo_ros_eol_gui.so', '--verbose'],
            env=client_env, output='screen',
            on_exit=EmitEvent(event=Shutdown(reason='Gazebo client closed'))))
    actions += [
        # Gazebo uses only local resources here. Keep inherited desktop proxy
        # settings out of its server/client without changing the user's shell
        # or the environment of the other ROS nodes.
        GroupAction([
            *[UnsetEnvironmentVariable(key) for key in (
                'HTTP_PROXY', 'HTTPS_PROXY', 'ALL_PROXY',
                'http_proxy', 'https_proxy', 'all_proxy')],
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(gazebo / 'launch/gazebo.launch.py')),
                launch_arguments={'world': LaunchConfiguration('world'),
                                  'gui': 'false', 'pause': 'true',
                                  'verbose': 'true', 'server_required': 'true',
                                  'gui_required': 'true'}.items()),
        ]),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': ParameterValue(description, value_type=str),
                          'use_sim_time': True}], output='screen'),
        Node(package='maze_bringup', executable='velocity_guard.py',
             parameters=[str(share / 'config/burger.yaml'), {'use_sim_time': True,
                 'max_linear_speed': ParameterValue(LaunchConfiguration('guard_linear_speed'), value_type=float)}],
             output='screen', on_exit=EmitEvent(event=Shutdown(reason='Velocity guard stopped'))),
        after_success(spawn, [unpause], 'Spawn TurtleBot3 Burger'),
        after_success(unpause, ready, 'Unpause physics'),
        spawn,
    ]
    return actions
