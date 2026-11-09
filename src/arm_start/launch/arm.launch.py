"""Bring up the arm.

  ros2 launch arm_start arm.launch.py                       # simulation: joints held at zero
  ros2 launch arm_start arm.launch.py use_hardware:=true    # real arm through FluidNC

Web page (state, jog, go-to, homing, gamepad) at http://localhost:8080 (web_port:=...).
A gamepad connected to the browser always works (browser -> /joy -> joy_teleop -> /jog).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, GroupAction, RegisterEventHandler
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackagePrefix, FindPackageShare


def generate_launch_description():
    use_hardware = LaunchConfiguration('use_hardware')
    use_rviz = LaunchConfiguration('rviz')
    web_port = LaunchConfiguration('web_port')

    xacro_file = PathJoinSubstitution(
        [FindPackageShare('arm_description'), 'urdf', 'arm.urdf.xacro'])
    bridge_config = PathJoinSubstitution(
        [FindPackageShare('arm_start'), 'config', 'fluidnc_bridge.yaml'])
    robot_description = ParameterValue(Command(['xacro ', xacro_file]), value_type=str)

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
        parameters=[bridge_config],
        output='screen',
    )
    hardware = GroupAction(
        condition=IfCondition(use_hardware),
        actions=[
            attach_board,
            RegisterEventHandler(OnProcessExit(target_action=attach_board, on_exit=[bridge])),
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_hardware', default_value='false',
                              description='Connect to FluidNC instead of simulated joints'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('web_port', default_value='8080',
                              description='Port of the web page'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
        ),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            condition=UnlessCondition(use_hardware),
        ),
        Node(
            package='arm_web',
            executable='web_viewer',
            name='web_viewer',
            parameters=[{
                'port': ParameterValue(web_port, value_type=int),
            }],
            output='screen',
        ),
        Node(
            package='arm_teleop',
            executable='joy_teleop',
            name='joy_teleop',
            output='screen',
        ),
        hardware,
        Node(
            package='rviz2',
            executable='rviz2',
            condition=IfCondition(use_rviz),
        ),
    ])
