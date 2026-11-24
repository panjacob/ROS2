"""Verify the physical and planning worlds use identical geometry."""

from pathlib import Path
import xml.etree.ElementTree as ET

import pytest

from arm_gazebo.scene import approach_target, collision_shapes, load_scene, write_world
from arm_gazebo.scene_manager import allow_mount_contact, planning_scene


CONFIG = Path(__file__).resolve().parents[1] / 'config/scene.yaml'


def test_gazebo_and_moveit_collision_geometry_match(tmp_path):
    scene = load_scene(CONFIG)
    world = ET.parse(write_world(scene, tmp_path / 'world.sdf')).getroot().find('world')
    moveit = planning_scene(scene)
    planned = {obj.id: obj for obj in moveit.world.collision_objects}
    indexes = {name: 0 for name in planned}
    for name, kind, dimensions, position in collision_shapes(scene):
        collision = world.find(f"model[@name='{name}']/link/collision[{indexes[name]+1}]")
        assert collision is not None
        assert list(map(float, collision.findtext('pose').split()[:3])) == position
        value = collision.findtext(f'geometry/{kind}/' + ('size' if kind == 'box' else 'radius'))
        assert list(map(float, value.split())) == dimensions
        obj, index = planned[name], indexes[name]
        assert obj.header.frame_id == 'world'
        assert obj.pose.orientation.w == 1
        assert list(obj.primitives[index].dimensions) == dimensions
        pose = obj.primitive_poses[index]
        assert [pose.position.x, pose.position.y, pose.position.z] == position
        assert pose.orientation.w == 1
        indexes[name] += 1
    assert {model.get('name') for model in world.findall('model')} == set(planned)
    assert all(model.findtext('static') == 'true' for model in world.findall('model'))
    banana = world.find("model[@name='banana']/link")
    visuals = banana.findall('visual')
    assert len(visuals) == 2 * scene['banana']['segments'] - 1
    assert all(visual.find('geometry/mesh') is None for visual in visuals)
    # The banana can render from SDF alone, without any temporary external asset.
    assert not world.findall('.//uri')
    camera = list(map(float, world.findtext('gui/plugin/camera_pose').split()))
    assert sum(v*v for v in camera[:3]) < 5


def test_approach_has_ten_centimetres_clearance():
    scene = load_scene(CONFIG)
    target = approach_target(scene)
    top = max(position[2] + dimensions[0]
              for name, _, dimensions, position in collision_shapes(scene) if name == 'banana')
    assert target[2] - top == pytest.approx(0.10)
    assert target[:2] == pytest.approx([0.28, 0.035])
    assert scene['table']['center'][2] + scene['table']['size'][2]/2 == 0


def test_mount_contact_preserves_self_collision_rules():
    from moveit_msgs.msg import AllowedCollisionEntry, AllowedCollisionMatrix
    matrix = AllowedCollisionMatrix(
        entry_names=['base_link', 'link1'],
        entry_values=[AllowedCollisionEntry(enabled=[False, True]),
                      AllowedCollisionEntry(enabled=[True, False])])
    updated = allow_mount_contact(matrix)
    assert list(updated.entry_names) == ['base_link', 'link1', 'table']
    assert [list(row.enabled) for row in updated.entry_values] == [
        [False, True, True], [True, False, False], [True, False, False]]
    assert len(matrix.entry_names) == 2
    assert allow_mount_contact(updated) == updated


@pytest.mark.parametrize('value', [0, -1, float('nan'), float('inf')])
def test_reject_invalid_clearance(tmp_path, value):
    import yaml
    scene = load_scene(CONFIG)
    scene['approach']['clearance'] = value
    config = tmp_path / 'scene.yaml'
    config.write_text(yaml.safe_dump(scene))
    with pytest.raises(ValueError):
        load_scene(config)
