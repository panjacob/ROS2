import importlib.util
from pathlib import Path


path = Path(__file__).resolve().parents[1] / 'windows' / 'windows_camera_bridge.py'
spec = importlib.util.spec_from_file_location('windows_camera_bridge', path)
bridge = importlib.util.module_from_spec(spec)
spec.loader.exec_module(bridge)


def test_frames_expire_and_wait_does_not_invent_new_frames(monkeypatch):
    now = [100.0]
    monkeypatch.setattr(bridge.time, 'monotonic', lambda: now[0])
    frames = bridge.Frames()
    assert frames.latest() is None
    frames.put(b'jpeg')
    seq, jpeg = frames.wait_next(0, 0)
    assert (seq, jpeg) == (1, b'jpeg')
    assert frames.latest() == b'jpeg'
    now[0] += 2.0
    assert frames.latest() is None
    assert frames.wait_next(seq, 0) == (seq, b'jpeg')
