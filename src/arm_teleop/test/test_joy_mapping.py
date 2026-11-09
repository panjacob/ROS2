from arm_teleop.joy_teleop import joy_to_velocities, shape

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
