"""Bring up the arm.

  ros2 launch arm_start arm.launch.py                       # joint simulator + web panel + RViz
  ros2 launch arm_start arm.launch.py use_hardware:=true    # real arm through NanoBee
  ros2 launch arm_start arm.launch.py simulator:=gazebo     # Gazebo table + banana
  ros2 launch arm_start arm.launch.py camera:=false         # without cameras

Web page (state, cameras, jog, go-to, homing, gamepad) at http://localhost:8080 (web_port:=...).
A gamepad controls the simple simulator and hardware (browser -> /joy -> joy_teleop -> /jog).
Gazebo supports previewed plans, joint jogging and STOP.
MoveIt executes plans through /arm_controller/follow_joint_trajectory, served by the simulator
or, with use_hardware:=true, by the NanoBee bridge (planned at 0.1 speed scaling, 0.3 in sim).
"""

import os
import uuid

import xacro
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess, GroupAction,
                            IncludeLaunchDescription, LogInfo, OpaqueFunction,
                            RegisterEventHandler, SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackagePrefix, FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def _on_gazebo_panel_exit(event, context):
    if event.returncode == 0 or context.is_shutdown:
        return []
    port = LaunchConfiguration('web_port').perform(context)
    return [EmitEvent(event=Shutdown(
        reason=f'Gazebo web panel failed on port {port}. If the port is in use, '
               'stop the previous launch or set web_port to a free port.'))]


def _launch(context):
    use_hardware = LaunchConfiguration('use_hardware')
    gazebo = LaunchConfiguration('simulator').perform(context) == 'gazebo'
    cameras_enabled = IfCondition(LaunchConfiguration('camera')).evaluate(context)
    if gazebo and use_hardware.perform(context) == 'true':
        raise ValueError('simulator:=gazebo cannot be combined with use_hardware:=true')
    use_rviz = LaunchConfiguration('rviz')
    web_port = LaunchConfiguration('web_port')
    tcp_storage_path = LaunchConfiguration('tcp_storage_path')
    model = LaunchConfiguration('model')
    controller_ns = '/arm_gazebo' if gazebo else PythonExpression([
        "'/nanobee_bridge' if '", use_hardware, "' == 'true' else '/arm_sim'"])
    simulation = PythonExpression([
        "'false' if '", use_hardware, "' == 'true' else 'true'"])
    # Planned motion runs slower on the real arm until it has been validated there.
    plan_scaling = PythonExpression([
        "'0.1' if '", use_hardware, "' == 'true' else '0.3'"])

    xacro_file = PathJoinSubstitution(
        [FindPackageShare('arm_description'), 'urdf', model])
    bridge_config = PathJoinSubstitution(
        [FindPackageShare('arm_start'), 'config', 'nanobee_bridge.yaml'])
    rviz_config = PathJoinSubstitution(
        [FindPackageShare('arm_description'), 'rviz', 'arm.rviz'])
    robot_description = ParameterValue(Command(['xacro ', xacro_file]), value_type=str)
    camera_count = 0
    camera_actions = []
    gazebo_actions = []
    web_scene = {}
    description_remappings = []
    if gazebo:
        from arm_gazebo.scene import add_robot_cameras, approach_target, load_scene
        gazebo_share = get_package_share_directory('arm_gazebo')
        scene_path = (LaunchConfiguration('scene_config').perform(context) or
                      os.path.join(gazebo_share, 'config', 'scene.yaml'))
        scene = load_scene(scene_path)
        target = approach_target(scene)
        cameras = scene['cameras'] if cameras_enabled else []
        camera_count = len(cameras)
        # The embedded controller manager reads the URDF independently from Gazebo's spawn file.
        # Give it this run's publisher so retained messages from other launches cannot reach it.
        description_topic = f'/arm_gazebo/robot_description_{uuid.uuid4().hex}'
        description_remappings = [('robot_description', description_topic)]
        description = xacro.process_file(xacro_file.perform(context), mappings={
            'gazebo': 'true',
            'gazebo_controllers': os.path.join(gazebo_share, 'config', 'controllers.yaml'),
            'gazebo_robot_description_topic': description_topic,
        }).toxml()
        description = add_robot_cameras(description, cameras)
        robot_description = ParameterValue(description, value_type=str)
        domain = LaunchConfiguration('gazebo_ros_domain_id')
        gazebo_actions.extend([
            SetEnvironmentVariable('ROS_DOMAIN_ID', domain),
            LogInfo(msg=['Gazebo ROS domain: ', domain]),
        ])
        gazebo_actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                gazebo_share, 'launch', 'simulation.launch.py')),
            launch_arguments={'scene_config': scene_path,
                              'robot_description': description,
                              'robot_description_topic': description_topic,
                              'camera': LaunchConfiguration('camera'),
                              'gazebo_gui': LaunchConfiguration('gazebo_gui')}.items()))
        web_scene = {
            'direct_motion_available': False,
            'jog_available': True,
            'max_jog_speed': 120.0,
            'require_scene_ready': True,
            'demo_target_mm': [float(v * 1000) for v in target],
        }
    elif cameras_enabled and IfCondition(use_hardware).evaluate(context):
        config = LaunchConfiguration('usb_cameras_config').perform(context)
        with open(config, encoding='utf-8') as stream:
            devices = yaml.safe_load(stream)
        if not isinstance(devices, list):
            raise ValueError('USB camera configuration must be a list')
        for number, spec in enumerate(devices, 1):
            camera_actions.extend(_usb_camera(number, spec))
        camera_count = len(devices)
    moveit_config = (
        MoveItConfigsBuilder('printed_arm_concept', package_name='arm_moveit_config')
        .robot_description(file_path=os.path.join(
            get_package_share_directory('arm_description'), 'urdf',
            'arm_concept.urdf.xacro'))
        .robot_description_semantic(
            file_path='config/arm.srdf.xacro',
            mappings={'robot_name': PythonExpression([
                "'printed_arm_concept' if '", model,
                "'.startswith('arm_concept') else 'printed_arm'"])})
        .planning_pipelines(pipelines=['ompl'])
        .trajectory_execution(file_path='config/moveit_controllers.yaml')
        .to_moveit_configs()
    )
    # Keep one URDF source for TF, MoveIt, and the web panel.
    moveit_config.robot_description = {'robot_description': robot_description}

    # Hardware: first make sure /dev/ttyUSB0 exists (WSL: auto "usbipd attach"), then start the bridge.
    attach_board = ExecuteProcess(
        cmd=[PathJoinSubstitution(
            [FindPackagePrefix('arm_start'), 'lib', 'arm_start', 'attach_board.sh'])],
        name='attach_board',
        output='screen',
    )
    bridge = Node(
        package='arm_nanobee_bridge',
        executable='nanobee_bridge',
        name='nanobee_bridge',
        parameters=[bridge_config, {'robot_description': robot_description}],
        output='screen',
    )
    hardware = GroupAction(
        condition=IfCondition(use_hardware),
        actions=[
            attach_board,
            RegisterEventHandler(OnProcessExit(target_action=attach_board, on_exit=[bridge])),
        ],
    )

    return [
        *gazebo_actions,
        Node(
            package='moveit_ros_move_group',
            executable='move_group',
            name='move_group',
            parameters=[moveit_config.to_dict(), {
                # All backends serve /arm_controller/follow_joint_trajectory.
                'allow_trajectory_execution': True,
                'use_sim_time': gazebo,
            }],
            output='screen',
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            remappings=description_remappings,
            parameters=[{'robot_description': robot_description, 'use_sim_time': gazebo}],
        ),
        Node(
            package='arm_sim',
            executable='joint_simulator',
            name='arm_sim',
            parameters=[{'robot_description': robot_description}],
            condition=IfCondition(PythonExpression([
                "'", use_hardware, "' != 'true' and ", str(not gazebo)])),
        ),
        Node(
            package='arm_web',
            executable='web_viewer',
            name='web_viewer',
            on_exit=_on_gazebo_panel_exit if gazebo else None,
            parameters=[{
                'port': ParameterValue(web_port, value_type=int),
                'bridge_ns': controller_ns,
                'simulation': ParameterValue(simulation, value_type=bool),
                'camera_count': camera_count,
                'tcp_storage_path': tcp_storage_path,
                'robot_description': robot_description,
                'max_velocity_scaling': ParameterValue(plan_scaling, value_type=float),
                'max_acceleration_scaling': ParameterValue(plan_scaling, value_type=float),
                'use_sim_time': gazebo,
                **web_scene,
            }],
            output='screen',
        ),
        Node(
            package='arm_teleop',
            executable='joy_teleop',
            name='joy_teleop',
            parameters=[{'stop_service': [controller_ns, '/stop']}],
            condition=IfCondition(str(not gazebo).lower()),
            output='screen',
        ),
        hardware,
        *camera_actions,
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config],
            parameters=[moveit_config.robot_description,
                        moveit_config.robot_description_semantic,
                        {'use_sim_time': gazebo}],
            condition=IfCondition(use_rviz),
        ),
    ]


def _usb_camera(number, spec):
    device = spec.get('device') if isinstance(spec, dict) else None
    if not isinstance(device, str) or not device.startswith('/dev/'):
        raise ValueError(f'camera {number} needs a device path under /dev/')
    parameters = {'device': device}
    for key in ('width', 'height', 'fps', 'fourcc', 'exposure'):
        if key in spec:
            parameters[key] = spec[key]
    node = Node(package='arm_camera', executable='camera_node', name=f'camera_{number}',
                parameters=[parameters],
                remappings=[('image_raw', f'/camera_{number}/image_raw'),
                            ('image_raw/compressed', f'/camera_{number}/image_raw/compressed')],
                output='screen')
    if not spec.get('usb_id'):
        return [node]
    attach = ExecuteProcess(
        cmd=[PathJoinSubstitution(
            [FindPackagePrefix('arm_start'), 'lib', 'arm_start', 'attach_board.sh']),
            device, spec['usb_id']],
        name=f'attach_camera_{number}', output='screen')
    return [RegisterEventHandler(OnProcessExit(target_action=attach, on_exit=[node])), attach]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_hardware', default_value='false', choices=['true', 'false'],
                              description='Connect to the NanoBee board instead of simulation'),
        DeclareLaunchArgument('simulator', default_value='simple', choices=['simple', 'gazebo']),
        DeclareLaunchArgument('gazebo_gui', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('gazebo_ros_domain_id', default_value='42',
                              description='ROS domain used by the Gazebo session'),
        DeclareLaunchArgument('scene_config', default_value='',
                              description='Gazebo scene YAML; empty uses the tabletop demo'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('model', default_value='arm_concept.urdf.xacro',
                              description='Xacro model from arm_description/urdf'),
        DeclareLaunchArgument('web_port', default_value='8080'),
        DeclareLaunchArgument('tcp_storage_path', default_value='~/.config/ros2_arm/tcps.json'),
        DeclareLaunchArgument('camera', default_value='true', choices=['true', 'false'],
                              description='Enable cameras for the selected backend'),
        DeclareLaunchArgument('usb_cameras_config', default_value=PathJoinSubstitution(
            [FindPackageShare('arm_start'), 'config', 'usb_cameras.yaml']),
            description='YAML list of camera devices for the physical arm'),
        OpaqueFunction(function=_launch),
    ])
