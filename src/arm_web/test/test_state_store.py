import math

from sensor_msgs.msg import JointState

from arm_web.web_viewer import StateStore


def test_joint_snapshot():
    store = StateStore()
    msg = JointState(name=['joint1', 'joint2'], position=[math.pi / 2, 0.0])
    store.update_joints(msg)
    snap = store.snapshot()
    assert math.isclose(snap['joints']['joint1']['position_deg'], 90.0)
    assert snap['joints']['joint2']['velocity'] is None
    assert snap['joints_age'] is not None


def test_status_snapshot():
    store = StateStore()
    assert store.snapshot()['joints_age'] is None
    store.update_status('machine_state', 'Idle')
    assert store.snapshot()['status']['machine_state']['value'] == 'Idle'
