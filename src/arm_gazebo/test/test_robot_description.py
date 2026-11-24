"""Both supported models opt into Gazebo without changing the default backend."""

from pathlib import Path
import math
import subprocess
import xml.etree.ElementTree as ET

import pytest

from arm_gazebo.scene import add_robot_cameras, load_scene


@pytest.mark.parametrize('name', ['arm_concept.urdf.xacro', 'arm.urdf.xacro'])
def test_gazebo_description_is_opt_in(name, tmp_path):
    root = Path(__file__).resolve().parents[2]
    model = root / 'arm_description/urdf' / name
    basic = ET.fromstring(subprocess.check_output(['xacro', str(model)], text=True))
    assert basic.find('ros2_control') is None
    assert basic.find('.//sensor') is None
    assert [float(joint.find('limit').get('velocity')) for joint in basic.findall('joint')
            if joint.get('type') == 'revolute'] == [1., .5, .7, 1., 1.]
    gazebo = ET.fromstring(subprocess.check_output([
        'xacro', str(model), 'gazebo:=true',
        'gazebo_controllers:=/tmp/arm-gazebo-controller-test.yaml',
        'gazebo_robot_description_topic:=/arm_gazebo/robot_description_test'], text=True))
    control = gazebo.find('ros2_control')
    assert control.findtext('hardware/plugin') == 'gz_ros2_control/GazeboSimSystem'
    assert [joint.get('name') for joint in control.findall('joint')] == [
        f'joint{i}' for i in range(1, 6)]
    assert all(joint.find('command_interface').get('name') == 'position'
               for joint in control.findall('joint'))
    assert gazebo.findtext('gazebo/plugin/parameters') == '/tmp/arm-gazebo-controller-test.yaml'
    assert gazebo.find('joint[@name="world_to_base"]').get('type') == 'fixed'
    # Gazebo folds the fixed tool0 link into link5. Verify the sensor follows
    # the last moving joint and looks out along the flange after conversion.
    config = root / 'arm_gazebo/config/scene.yaml'
    urdf = tmp_path / 'arm.urdf'
    description = ET.tostring(gazebo, encoding='unicode')
    urdf.write_text(add_robot_cameras(description, load_scene(config)['cameras']))
    sdf = ET.fromstring(subprocess.check_output(['gz', 'sdf', '-p', str(urdf)], text=True))
    assert sdf.findtext('.//plugin/ros/remapping') == (
        '/robot_description:=/arm_gazebo/robot_description_test')
    sensor = sdf.find(".//link[@name='link5']/sensor[@name='camera_1']")
    assert sensor.findtext('topic') == '/camera_1/image_raw'
    x, y, z, _, pitch, _ = map(float, sensor.findtext('pose').split())
    assert [x, y, z] == pytest.approx([0, 0, .035])
    assert -math.sin(pitch) == pytest.approx(1)
    disabled = ET.fromstring(add_robot_cameras(description, []))
    assert disabled.find('.//sensor') is None
    assert [float(joint.find('limit').get('velocity')) for joint in gazebo.findall('joint')
            if joint.get('type') == 'revolute'] == pytest.approx([math.radians(120)]*5)
