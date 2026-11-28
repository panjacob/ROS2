from types import SimpleNamespace

from arm_teleop.joy_teleop import JoyTeleop, joy_to_velocities, shape

CFG = {
    'axis_index': [0, 1, -1],
    'axis_scale': [1.0, -1.0, 0.0],
    'button_minus': [-1, -1, 4],
    'button_plus': [-1, -1, 5],
    'deadzone': 0.1,
    'expo': 1.0,
    'max_speed': 30.0,
    'enable_button': -1,
}


def test_shape_deadzone_and_full():
    assert shape(0.05, 0.1, 1.0) == 0.0
    assert shape(1.0, 0.1, 1.8) == 1.0
    assert shape(-2.0, 0.1, 1.0) == -1.0


def test_axes_and_inversion():
    vel = joy_to_velocities([1.0, 1.0], [0] * 6, CFG)
    assert vel == [30.0, -30.0, 0.0]


def test_buttons_drive_joint():
    assert joy_to_velocities([0.0, 0.0], [0, 0, 0, 0, 0, 1], CFG)[2] == 30.0
    assert joy_to_velocities([0.0, 0.0], [0, 0, 0, 0, 1, 0], CFG)[2] == -30.0


def test_enable_button_required():
    cfg = dict(CFG, enable_button=6)
    assert joy_to_velocities([1.0, 0.0], [0] * 8, cfg) == [0.0, 0.0, 0.0]
    assert joy_to_velocities([1.0, 0.0], [0] * 6 + [1, 0], cfg)[0] == 30.0


def test_missing_axes_are_zero():
    assert joy_to_velocities([], [], CFG) == [0.0, 0.0, 0.0]


def test_web_mapped_axes_preserve_response_curve_and_stop():
    received = []
    node = SimpleNamespace(_joints=['joint1', 'joint2', 'joint3'], _cfg=dict(CFG, expo=2.0))
    node._handle_joy = lambda msg, cfg: received.append(joy_to_velocities(msg.axes, msg.buttons, cfg))
    JoyTeleop._on_mapped_joy(node, SimpleNamespace(axes=[0.5, -1.0, 0.25], buttons=[0] * 6))
    assert received == [[7.5, -30.0, 1.875]]


def test_stop_button_remains_zero_while_held():
    warnings = []
    node = SimpleNamespace(
        _joints=['joint1', 'joint2', 'joint3'], _stop_button=1, _stop_was_down=False,
        _stop_client=SimpleNamespace(service_is_ready=lambda: False),
        get_logger=lambda: SimpleNamespace(warn=warnings.append),
    )
    msg = SimpleNamespace(axes=[1.0, 0.0], buttons=[0, 1])
    JoyTeleop._handle_joy(node, msg, CFG)
    JoyTeleop._handle_joy(node, msg, CFG)
    assert node._vel == [0.0, 0.0, 0.0]
    assert len(warnings) == 1
