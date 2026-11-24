"""Exercise the launch arguments through the installed spawner's parameter loader."""

import importlib.util
from pathlib import Path
from types import SimpleNamespace

from controller_manager import controller_manager_services as services
from controller_manager.spawner import parse_native_args
from launch import LaunchContext
from launch_ros.actions import Node
from launch.actions import ExecuteProcess


def test_spawner_passes_joint_configuration_to_controller(monkeypatch, tmp_path):
    package = Path(__file__).resolve().parents[1]
    spec = importlib.util.spec_from_file_location(
        'gazebo_launch_test', package / 'launch/simulation.launch.py')
    launch = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(launch)
    original_share = launch.get_package_share_directory
    monkeypatch.setattr(launch, 'get_package_share_directory', lambda name: (
        str(package) if name == 'arm_gazebo' else original_share(name)))
    monkeypatch.setattr(launch.tempfile, 'mkdtemp', lambda **_: str(tmp_path))
    nodes = []
    processes = []

    def record_node(**kwargs):
        nodes.append(kwargs)
        return Node(**kwargs)

    monkeypatch.setattr(launch, 'Node', record_node)

    def record_process(**kwargs):
        processes.append(kwargs)
        return ExecuteProcess(**kwargs)

    monkeypatch.setattr(launch, 'ExecuteProcess', record_process)
    context = LaunchContext()
    description = '<robot name="this_launch"><link name="base_link"/></robot>'
    context.launch_configurations.update(
        scene_config=str(package / 'config/scene.yaml'), gazebo_gui='false', camera='true',
        robot_description=description)
    actions = launch._start(context)
    # Avoid the combined shell parent which survived shutdown in desktop logs.
    assert processes[0]['cmd'][1:4] == ['sim', '-s', '-r']
    assert processes[1]['cmd'][1:] == ['sim', '-g']
    assert all(not process.get('shell') and process.get('on_exit') for process in processes)
    actions[0].execute(context)
    partition = context.environment['GZ_PARTITION']
    assert partition.startswith('arm_gazebo_')
    launch._start(context)[0].execute(context)
    assert context.environment['GZ_PARTITION'] != partition
    spawn = next(node for node in nodes if node['package'] == 'ros_gz_sim')
    assert '-topic' not in spawn['arguments']
    source = spawn['arguments'][spawn['arguments'].index('-file') + 1]
    assert Path(source).read_text() == description
    bridge = next(node for node in nodes if node['package'] == 'ros_gz_bridge')
    assert bridge['arguments'] == ['/world/tabletop/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock']
    assert bridge['remappings'] == [('/world/tabletop/clock', '/clock')]
    image_bridge = next(node for node in nodes if node['package'] == 'ros_gz_image')
    assert image_bridge['arguments'] == [f'/camera_{number}/image_raw' for number in range(1, 4)]
    context.launch_configurations['camera'] = 'false'
    nodes.clear()
    launch._start(context)
    assert all(node['package'] != 'ros_gz_image' for node in nodes)
    spawner = next(node for node in nodes if node['package'] == 'controller_manager')
    # A use_sim_time-only Node parameter file was forwarded by Lyrical's spawner
    # and replaced the plugin's controller params_file, dropping the joint list.
    assert not spawner.get('parameters')
    _, controllers = parse_native_args(spawner['arguments'])
    arm = next(controller for controller in controllers if controller['name'] == 'gazebo_arm_controller')
    configured = {}

    def set_parameter(_node, _manager, _controller, name, value):
        configured[name] = value
        return True

    monkeypatch.setattr(services, 'set_controller_parameters', set_parameter)
    node = SimpleNamespace(get_namespace=lambda: '/')
    assert services.set_controller_parameters_from_param_files(
        node, '/controller_manager', 'gazebo_arm_controller', arm['param_files'], '/')
    files = configured['params_file']
    assert services.get_parameter_from_param_files(
        node, 'gazebo_arm_controller', '/', files, 'joints') == [f'joint{i}' for i in range(1, 6)]
    assert services.get_parameter_from_param_files(
        node, 'gazebo_arm_controller', '/', files, 'command_interfaces') == ['position']
    assert services.get_parameter_from_param_files(
        node, 'gazebo_arm_controller', '/', files, 'state_interfaces') == ['position', 'velocity']
