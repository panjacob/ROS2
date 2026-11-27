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


@pytest.mark.parametrize('name', ['arm_concept.urdf.xacro', 'arm.urdf.xacro'])
def test_joint_limits_are_xacro_args(name):
    model = Path(__file__).resolve().parents[2] / 'arm_description/urdf' / name

    def limits(*args):
        root = ET.fromstring(subprocess.check_output(['xacro', str(model), *args], text=True))
        return [(float(j.find('limit').get('lower')), float(j.find('limit').get('upper')))
                for j in root.findall('joint') if j.get('type') == 'revolute']

    expected = [(-170, 170), (-90, 90), (-135, 135), (-100, 100), (-180, 180)]
    assert limits() == pytest.approx([tuple(map(math.radians, p)) for p in expected])
    changed = limits('joint1_min_deg:=-180', 'joint1_max_deg:=180.5', 'joint4_max_deg:=95')
    expected[0], expected[3] = (-180, 180.5), (-100, 95)
    assert changed == pytest.approx([tuple(map(math.radians, p)) for p in expected])
