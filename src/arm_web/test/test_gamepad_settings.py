import yaml
import pytest

from arm_web.gamepad_settings import DEFAULT, GamepadSettings


def test_save_reload_and_conflict(tmp_path):
    path = tmp_path / 'settings' / 'gamepad.yaml'
    store = GamepadSettings(path)
    initial = store.snapshot()
    value = initial['settings']
    value['deadzone'] = .25
    value['joints'][4] = {'control': 'lt_rt', 'invert': True}
    saved = store.save(value, initial['revision'])
    assert yaml.safe_load(path.read_text()) == value
    assert GamepadSettings(path).snapshot() == saved
    with pytest.raises(ValueError, match='changed elsewhere'):
        store.save(DEFAULT, initial['revision'])
    assert GamepadSettings(path).snapshot() == saved


@pytest.mark.parametrize('bad', [
    None, {}, {'deadzone': 1, 'joints': DEFAULT['joints']},
    {'deadzone': float('nan'), 'joints': DEFAULT['joints']},
    {'deadzone': .1, 'joints': [{'control': 'invalid', 'invert': False}] * 5},
    {'deadzone': .1, 'joints': [{'control': 'lt_rt', 'invert': 1}] * 5},
])
def test_invalid_settings_never_written(tmp_path, bad):
    path = tmp_path / 'gamepad.yaml'
    store = GamepadSettings(path)
    with pytest.raises(ValueError):
        store.save(bad, store.snapshot()['revision'])
    assert not path.exists()


def test_invalid_yaml_is_reported(tmp_path):
    path = tmp_path / 'gamepad.yaml'
    path.write_text('deadzone: [broken')
    with pytest.raises(ValueError, match='Invalid gamepad settings'):
        GamepadSettings(path)


def test_write_failure_does_not_change_live_settings(tmp_path, monkeypatch):
    path = tmp_path / 'gamepad.yaml'
    store = GamepadSettings(path)
    original = store.snapshot()
    value = store.snapshot()['settings']
    value['deadzone'] = .3
    def fail(*_):
        raise OSError('disk error')
    monkeypatch.setattr('arm_web.gamepad_settings.os.replace', fail)
    with pytest.raises(OSError, match='disk error'):
        store.save(value, original['revision'])
    assert store.snapshot() == original
    assert list(tmp_path.iterdir()) == []
