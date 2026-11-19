"""Both supported models opt into Gazebo without changing the default backend."""

from pathlib import Path
import math
import subprocess
import xml.etree.ElementTree as ET

import pytest


@pytest.mark.parametrize('name', ['arm_concept.urdf.xacro', 'arm.urdf.xacro'])
def test_gazebo_description_is_opt_in(name):
    root = Path(__file__).resolve().parents[2]
    model = root / 'arm_description/urdf' / name
    basic = ET.fromstring(subprocess.check_output(['xacro', str(model)], text=True))
    assert basic.find('ros2_control') is None
    assert [float(joint.find('limit').get('velocity')) for joint in basic.findall('joint')
            if joint.get('type') == 'revolute'] == [1., .5, .7, 1., 1.]
    gazebo = ET.fromstring(subprocess.check_output([
        'xacro', str(model), 'gazebo:=true',
        'gazebo_controllers:=/tmp/arm-gazebo-controller-test.yaml'], text=True))
    control = gazebo.find('ros2_control')
    assert control.findtext('hardware/plugin') == 'gz_ros2_control/GazeboSimSystem'
    assert [joint.get('name') for joint in control.findall('joint')] == [
        f'joint{i}' for i in range(1, 6)]
    assert all(joint.find('command_interface').get('name') == 'position'
               for joint in control.findall('joint'))
    assert gazebo.findtext('gazebo/plugin/parameters') == '/tmp/arm-gazebo-controller-test.yaml'
    assert gazebo.find('joint[@name="world_to_base"]').get('type') == 'fixed'
    assert [float(joint.find('limit').get('velocity')) for joint in gazebo.findall('joint')
            if joint.get('type') == 'revolute'] == pytest.approx([math.radians(120)]*5)
