"""Bring up the arm.

  ros2 launch arm_start arm.launch.py                       # joint simulator + web panel + RViz
  ros2 launch arm_start arm.launch.py use_hardware:=true    # real arm through NanoBee
  ros2 launch arm_start arm.launch.py camera:=false         # without the USB camera
  ros2 launch arm_start arm.launch.py camera2:=false        # without the phone camera (Telescope)
  ros2 launch arm_start arm.launch.py camera_width:=640 camera_height:=480  # native Linux host

Web page (state, camera, jog, go-to, homing, gamepad) at http://localhost:8080 (web_port:=...).
A gamepad connected to the browser always works (browser -> /joy -> joy_teleop -> /jog).
MoveIt executes plans through /arm_controller/follow_joint_trajectory, served by the simulator
or, with use_hardware:=true, by the NanoBee bridge (planned at 0.1 speed scaling, 0.3 in sim).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, GroupAction, RegisterEventHandler
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackagePrefix, FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    use_hardware = LaunchConfiguration('use_hardware')
    use_rviz = LaunchConfiguration('rviz')
    web_port = LaunchConfiguration('web_port')
    tcp_storage_path = LaunchConfiguration('tcp_storage_path')
    model = LaunchConfiguration('model')
    controller_ns = PythonExpression([
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

    # Camera: attach the USB camera to WSL if needed; the node retries until frames arrive.
    attach_camera = ExecuteProcess(
        cmd=[PathJoinSubstitution(
            [FindPackagePrefix('arm_start'), 'lib', 'arm_start', 'attach_board.sh']),
            LaunchConfiguration('camera_device'), LaunchConfiguration('camera_usb_id')],
        name='attach_camera',
        output='screen',
    )
    camera_node = Node(
        package='arm_camera',
        executable='camera_node',
        name='camera',
        parameters=[{'device': LaunchConfiguration('camera_device'),
                     'width': ParameterValue(LaunchConfiguration('camera_width'), value_type=int),
                     'height': ParameterValue(LaunchConfiguration('camera_height'), value_type=int),
                     'exposure': ParameterValue(
                         LaunchConfiguration('camera_exposure'), value_type=int)}],
        output='screen',
    )
    camera = GroupAction(
        condition=IfCondition(LaunchConfiguration('camera')),
        actions=[
            attach_camera,
            RegisterEventHandler(OnProcessExit(target_action=attach_camera, on_exit=[camera_node])),
        ],
    )

    # Second camera: phone through Telescope. WSL can't open Windows webcams, so the bridge runs on
    # Windows (WSL interop) and serves the camera as MJPEG; camera2 reads it and retries until it's up.
    camera2_bridge = ExecuteProcess(
        cmd=['bash', '-c',
             'command -v py.exe >/dev/null || { echo "py.exe not found: needs WSL with Windows '
             'Python (or camera2_bridge:=false and start the bridge yourself)"; exit 1; }; '
             'exec py.exe "$(wslpath -w "$(realpath "$0")")" --camera "$1"',
             PathJoinSubstitution(
                 [FindPackageShare('arm_camera'), 'windows', 'windows_camera_bridge.py']),
             LaunchConfiguration('camera2_name')],
        name='camera2_bridge',
        output='screen',
        condition=IfCondition(PythonExpression([
            "'", LaunchConfiguration('camera2'), "' == 'true' and '",
            LaunchConfiguration('camera2_bridge'), "' == 'true'"])),
    )
    camera2_node = Node(
        package='arm_camera',
        executable='camera_node',
        name='camera2',
        parameters=[{'device': LaunchConfiguration('camera2_device'),
                     'frame_id': 'camera2_link'}],
        remappings=[('camera/image_raw', 'camera2/image_raw'),
                    ('camera/image_raw/compressed', 'camera2/image_raw/compressed')],
        condition=IfCondition(LaunchConfiguration('camera2')),
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_hardware', default_value='false',
                              description='Connect to the NanoBee board instead of the joint simulator'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('model', default_value='arm_concept.urdf.xacro',
                              description='Xacro model from arm_description/urdf'),
        DeclareLaunchArgument('web_port', default_value='8080',
                              description='Port of the web page'),
        DeclareLaunchArgument('tcp_storage_path', default_value='~/.config/ros2_arm/tcps.json',
                              description='Writable JSON file for named TCP offsets'),
        DeclareLaunchArgument('camera', default_value='true',
                              description='Start the USB camera (camera/image_raw[/compressed])'),
        DeclareLaunchArgument('camera_device',
                              default_value='/dev/v4l/by-id/usb-045e_USB20_Camera-video-index0',
                              description='V4L2 device of the camera'),
        DeclareLaunchArgument('camera_usb_id', default_value='045e:00f4',
                              description='USB vid:pid attached to WSL with usbipd'),
        DeclareLaunchArgument('camera_width', default_value='320',
                              description='Camera width: 160, 320 or 640 '
                                          '(640x480 is unreliable over usbipd/WSL)'),
        DeclareLaunchArgument('camera_height', default_value='240',
                              description='Camera height: 120, 240 or 480'),
        DeclareLaunchArgument('camera_exposure', default_value='1000',
                              description='Camera exposure 0..6016 (lower = more fps, darker); '
                                          '-1 = auto (slow in dim light)'),
        DeclareLaunchArgument('camera2', default_value='true',
                              description='Start the phone camera (camera2/image_raw[/compressed])'),
        DeclareLaunchArgument('camera2_bridge', default_value='true',
                              description='Start windows_camera_bridge.py on Windows (WSL interop)'),
        DeclareLaunchArgument('camera2_name', default_value='Telescope',
                              description='Windows camera the bridge reads (name or its start)'),
        DeclareLaunchArgument('camera2_device',
                              default_value='http://{windows_host}:8081/stream.mjpg',
                              description='Stream URL of the phone camera (windows_camera_bridge.py); '
                                          '{windows_host} = Windows as seen from WSL'),

        Node(
            package='moveit_ros_move_group',
            executable='move_group',
            name='move_group',
            parameters=[moveit_config.to_dict(), {
                # Both backends serve /arm_controller/follow_joint_trajectory.
                'allow_trajectory_execution': True,
            }],
            output='screen',
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
        ),
        Node(
            package='arm_sim',
            executable='joint_simulator',
            name='arm_sim',
            parameters=[{'robot_description': robot_description}],
            condition=UnlessCondition(use_hardware),
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
            }],
            output='screen',
        ),
        Node(
            package='arm_teleop',
            executable='joy_teleop',
            name='joy_teleop',
            parameters=[{'stop_service': [controller_ns, '/stop']}],
            output='screen',
        ),
        hardware,
        camera,
        camera2_bridge,
        camera2_node,
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config],
            parameters=[moveit_config.robot_description,
                        moveit_config.robot_description_semantic],
            condition=IfCondition(use_rviz),
        ),
    ])
