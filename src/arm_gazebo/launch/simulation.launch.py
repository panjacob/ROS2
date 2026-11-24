"""Gazebo only; the common arm_start launch supplies MoveIt, TF and the panel."""

import os
import shutil
import tempfile
import uuid
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess, LogInfo,
                            OpaqueFunction, RegisterEventHandler, SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import FindExecutable, LaunchConfiguration
from launch_ros.actions import Node

from arm_gazebo.scene import load_scene, write_world


def _start(context):
    share = Path(get_package_share_directory('arm_gazebo'))
    scene_path = LaunchConfiguration('scene_config').perform(context)
    scene = load_scene(scene_path)
    temporary = Path(tempfile.mkdtemp(prefix='arm-gazebo-'))
    camera = IfCondition(LaunchConfiguration('camera')).evaluate(context)
    cameras = scene['cameras'] if camera else []
    world = write_world(scene, temporary / 'tabletop.sdf', camera=camera)
    description = LaunchConfiguration('robot_description', default='').perform(context)
    description_topic = LaunchConfiguration('robot_description_topic', default='robot_description')
    model_source = ['-topic', 'robot_description']
    if description:
        # Spawn this launch's model even if another ROS node publishes robot_description.
        model = temporary / 'arm.urdf'
        model.write_text(description, encoding='utf-8')
        model_source = ['-file', str(model)]
    partition = 'arm_gazebo_' + uuid.uuid4().hex
    # Run server and GUI directly, so launch owns and signals both processes.
    # The combined shell launcher can leave a server behind after startup fails.
    resources = str(Path(get_package_share_directory('arm_description')).parent)
    environment = {
        'GZ_SIM_RESOURCE_PATH': os.pathsep.join(filter(None, [
            os.environ.get('GZ_SIM_RESOURCE_PATH'), resources])),
        'GZ_SIM_SYSTEM_PLUGIN_PATH': os.pathsep.join(filter(None, [
            os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH'), os.environ.get('LD_LIBRARY_PATH')])),
    }
    render_args = ['--headless-rendering'] if cameras and not context.environment.get('DISPLAY') else []
    server = ExecuteProcess(
        cmd=[FindExecutable(name='gz'), 'sim', '-s', '-r', *render_args, str(world)],
        name='gazebo_server', output='screen', additional_env=environment,
        on_exit=[EmitEvent(event=Shutdown(reason='Gazebo server exited'))])
    gui = ExecuteProcess(
        cmd=[FindExecutable(name='gz'), 'sim', '-g'],
        name='gazebo_gui', output='screen', additional_env=environment,
        condition=IfCondition(LaunchConfiguration('gazebo_gui')),
        on_exit=[EmitEvent(event=Shutdown(reason='Gazebo window closed'))])
    spawn = Node(
        package='ros_gz_sim', executable='create', output='screen',
        arguments=['-world', 'tabletop', '-name', 'arm', *model_source,
                   '-allow_renaming', 'false'],
    )
    controllers = Node(
        package='controller_manager', executable='spawner', output='screen',
        arguments=['joint_state_broadcaster', 'gazebo_arm_controller',
                   '--param-file', str(share / 'config/controllers.yaml'),
                   '--controller-manager-timeout', '90', '--switch-timeout', '30'],
        # Lyrical forwards the spawner's ROS parameter files to controllers.
        # A generated use_sim_time-only file would replace their configuration.
        # Gazebo/controller_manager already supply simulation time to controllers.
    )

    def after_spawn(event, _context):
        if event.returncode != 0:
            return [EmitEvent(event=Shutdown(reason='Cannot spawn Gazebo arm'))]
        return [controllers]

    def after_controllers(event, _context):
        if event.returncode != 0:
            return [EmitEvent(event=Shutdown(reason='Cannot activate Gazebo controllers'))]
        return [Node(
            package='arm_gazebo', executable='scene_manager', name='arm_gazebo',
            remappings=[('robot_description', description_topic)],
            parameters=[{'scene_config': scene_path, 'use_sim_time': True}], output='screen')]

    image_bridge = []
    if cameras:
        image_bridge.append(Node(package='ros_gz_image', executable='image_bridge',
                                 arguments=[f'/camera_{i}/image_raw' for i in range(1, len(cameras) + 1)],
                                 parameters=[{'use_sim_time': True}], output='screen'))

    return [
        SetEnvironmentVariable('GZ_PARTITION', partition),
        LogInfo(msg=f'Gazebo transport partition: {partition}'),
        RegisterEventHandler(OnProcessExit(target_action=server,
            on_exit=lambda _event, _context: shutil.rmtree(temporary, ignore_errors=True))),
        RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=after_spawn)),
        RegisterEventHandler(OnProcessExit(target_action=controllers, on_exit=after_controllers)),
        server,
        gui,
        *image_bridge,
        Node(package='ros_gz_bridge', executable='parameter_bridge',
             arguments=['/world/tabletop/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
             remappings=[('/world/tabletop/clock', '/clock')],
             output='screen'),
        spawn,
    ]


def generate_launch_description():
    share = Path(get_package_share_directory('arm_gazebo'))
    return LaunchDescription([
        DeclareLaunchArgument('gazebo_gui', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('camera', default_value='true'),
        DeclareLaunchArgument('scene_config', default_value=str(share / 'config/scene.yaml')),
        DeclareLaunchArgument('robot_description', default_value='',
                              description='URDF to spawn; empty reads the robot_description topic'),
        DeclareLaunchArgument('robot_description_topic', default_value='robot_description',
                              description='Topic used by Gazebo motion to read the URDF'),
        OpaqueFunction(function=_start),
    ])
