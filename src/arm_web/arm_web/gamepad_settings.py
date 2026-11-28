"""Persistent, validated browser gamepad mapping."""

import hashlib
import json
import math
import os
import tempfile
import threading
from pathlib import Path

import yaml


CONTROLS = ('none', 'left_x', 'left_y', 'right_x', 'right_y', 'lb_rb', 'lt_rt')
DEFAULT = {
    'deadzone': 0.1,
    'joints': [
        {'control': 'left_x', 'invert': False},
        {'control': 'left_y', 'invert': True},
        {'control': 'right_y', 'invert': True},
        {'control': 'right_x', 'invert': False},
        {'control': 'lb_rb', 'invert': False},
    ],
}


def validate(value):
    if not isinstance(value, dict) or set(value) != {'deadzone', 'joints'}:
        raise ValueError('Gamepad settings require deadzone and joints.')
    deadzone, joints = value['deadzone'], value['joints']
    if (isinstance(deadzone, bool) or not isinstance(deadzone, (int, float))
            or not math.isfinite(deadzone) or not 0 <= deadzone <= 0.95):
        raise ValueError('Dead zone must be between 0 and 0.95.')
    if not isinstance(joints, list) or len(joints) != 5:
        raise ValueError('Gamepad mapping requires five joints.')
    for joint in joints:
        if (not isinstance(joint, dict) or set(joint) != {'control', 'invert'}
                or joint['control'] not in CONTROLS or not isinstance(joint['invert'], bool)):
            raise ValueError('Each joint needs a valid control and invert setting.')
    return {'deadzone': float(deadzone), 'joints': [dict(joint) for joint in joints]}


def revision(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True).encode()).hexdigest()


class GamepadSettings:
    def __init__(self, path):
        self.path = Path(path).expanduser()
        self._lock = threading.Lock()
        if self.path.exists():
            try:
                self._value = validate(yaml.safe_load(self.path.read_text(encoding='utf-8')))
            except (yaml.YAMLError, ValueError) as exc:
                raise ValueError(f'Invalid gamepad settings in {self.path}: {exc}') from exc
        else:
            self._value = validate(DEFAULT)

    def snapshot(self):
        with self._lock:
            value = validate(self._value)
            return {'settings': value, 'revision': revision(value)}

    def save(self, value, expected_revision):
        value = validate(value)
        with self._lock:
            if expected_revision != revision(self._value):
                raise ValueError('Gamepad settings changed elsewhere; reload before saving.')
            self.path.parent.mkdir(parents=True, exist_ok=True)
            temporary = None
            try:
                with tempfile.NamedTemporaryFile('w', encoding='utf-8', dir=self.path.parent,
                                                 prefix='.gamepad-', suffix='.yaml', delete=False) as file:
                    temporary = Path(file.name)
                    yaml.safe_dump(value, file, sort_keys=False)
                os.replace(temporary, self.path)
            finally:
                if temporary is not None and temporary.exists():
                    temporary.unlink()
            self._value = value
            return {'settings': validate(value), 'revision': revision(value)}
