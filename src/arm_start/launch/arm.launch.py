"""Bring up the arm.

  ros2 launch arm_start arm.launch.py                       # joint simulator + web panel + RViz
  ros2 launch arm_start arm.launch.py use_hardware:=true    # real arm through FluidNC
  ros2 launch arm_start arm.launch.py simulator:=gazebo     # Gazebo table + banana

Web page (state, jog, go-to, homing, gamepad) at http://localhost:8080 (web_port:=...).
A gamepad controls the simple simulator and hardware (browser -> /joy -> joy_teleop -> /jog).
Gazebo supports previewed plans, joint jogging and STOP.
MoveIt executes plans through /arm_controller/follow_joint_trajectory, served by the simulator
or, with use_hardware:=true, by the FluidNC bridge (planned at 0.1 speed scaling, 0.3 in sim).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, GroupAction,
                            IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackagePrefix, FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def _launch(context):
    use_hardware = LaunchConfiguration('use_hardware')
    gazebo = LaunchConfiguration('simulator').perform(context) == 'gazebo'
    if gazebo and use_hardware.perform(context) == 'true':
        raise ValueError('simulator:=gazebo cannot be combined with use_hardware:=true')
    use_rviz = LaunchConfiguration('rviz')
    web_port = LaunchConfiguration('web_port')
    tcp_storage_path = LaunchConfiguration('tcp_storage_path')
    model = LaunchConfiguration('model')
    controller_ns = '/arm_gazebo' if gazebo else PythonExpression([
        "'/fluidnc_bridge' if '", use_hardware, "' == 'true' else '/arm_sim'"])
    simulation = PythonExpression([
        "'false' if '", use_hardware, "' == 'true' else 'true'"])
    # Planned motion runs slower on the real arm until it has been validated there.
    plan_scaling = PythonExpression([
        "'0.1' if '", use_hardware, "' == 'true' else '0.3'"])

    xacro_file = PathJoinSubstitution(
        [FindPackageShare('arm_description'), 'urdf', model])
    bridge_config = PathJoinSubstitution(
        [FindPackageShare('arm_start'), 'config', 'fluidnc_bridge.yaml'])
    rviz_config = PathJoinSubstitution(
        [FindPackageShare('arm_description'), 'rviz', 'arm.rviz'])
    xacro_command = ['xacro ', xacro_file]
    gazebo_actions = []
    web_scene = {}
    if gazebo:
        from arm_gazebo.scene import approach_target, load_scene
        gazebo_share = get_package_share_directory('arm_gazebo')
        scene_path = (LaunchConfiguration('scene_config').perform(context) or
                      os.path.join(gazebo_share, 'config', 'scene.yaml'))
        target = approach_target(load_scene(scene_path))
        xacro_command += [' gazebo:=true gazebo_controllers:=',
                          os.path.join(gazebo_share, 'config', 'controllers.yaml')]
        gazebo_actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                gazebo_share, 'launch', 'simulation.launch.py')),
            launch_arguments={'scene_config': scene_path,
                              'gazebo_gui': LaunchConfiguration('gazebo_gui')}.items()))
        web_scene = {
            'direct_motion_available': False,
            'jog_available': True,
            'max_jog_speed': 120.0,
            'require_scene_ready': True,
            'demo_target_mm': [float(v * 1000) for v in target],
        }
    robot_description = ParameterValue(Command(xacro_command), value_type=str)
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
        package='arm_fluidnc_bridge',
        executable='fluidnc_bridge',
        name='fluidnc_bridge',
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
            parameters=[{
                'port': ParameterValue(web_port, value_type=int),
                'bridge_ns': controller_ns,
                'simulation': ParameterValue(simulation, value_type=bool),
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


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_hardware', default_value='false', choices=['true', 'false'],
                              description='Connect to FluidNC instead of simulation'),
        DeclareLaunchArgument('simulator', default_value='simple', choices=['simple', 'gazebo']),
        DeclareLaunchArgument('gazebo_gui', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('scene_config', default_value='',
                              description='Gazebo scene YAML; empty uses the tabletop demo'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('model', default_value='arm_concept.urdf.xacro',
                              description='Xacro model from arm_description/urdf'),
        DeclareLaunchArgument('web_port', default_value='8080'),
        DeclareLaunchArgument('tcp_storage_path', default_value='~/.config/ros2_arm/tcps.json'),
        OpaqueFunction(function=_launch),
    ])
