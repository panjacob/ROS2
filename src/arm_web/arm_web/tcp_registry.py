"""Named tool-centre-point offsets relative to the URDF's tool0 frame."""

import json
import math
import os
import re
import tempfile
import threading
from pathlib import Path


NAME_RE = re.compile(r'^[A-Za-z][A-Za-z0-9_-]{0,31}$')


def frame_for(name):
    return 'tool0' if name == 'tool0' else f'tcp_{name}'


def validate_pose(name, xyz_m, rpy_deg):
    if not isinstance(name, str) or name == 'tool0' or not NAME_RE.fullmatch(name):
        raise ValueError('TCP name must start with a letter and contain only letters, numbers, _ or - (max 32)')
    try:
        xyz_m = [float(value) for value in xyz_m]
        rpy_deg = [float(value) for value in rpy_deg]
    except (TypeError, ValueError) as exc:
        raise ValueError('TCP position and rotation must be numbers') from exc
    if len(xyz_m) != 3 or len(rpy_deg) != 3 or not all(map(math.isfinite, xyz_m + rpy_deg)):
        raise ValueError('TCP requires three finite position and rotation values')
    return {'xyz_m': xyz_m, 'rpy_deg': rpy_deg}


def quaternion_from_rpy_degrees(rpy_deg):
    roll, pitch, yaw = (math.radians(value) / 2 for value in rpy_deg)
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def rpy_degrees_from_quaternion(rotation):
    x, y, z, w = rotation.x, rotation.y, rotation.z, rotation.w
    roll = math.atan2(2 * (w*x + y*z), 1 - 2 * (x*x + y*y))
    sin_pitch = 2 * (w*y - z*x)
    pitch = math.asin(max(-1.0, min(1.0, sin_pitch)))
    yaw = math.atan2(2 * (w*z + x*y), 1 - 2 * (y*y + z*z))
    return [math.degrees(value) for value in (roll, pitch, yaw)]


class TcpRegistry:
    def __init__(self, path):
        self.path = Path(path).expanduser()
        self._lock = threading.Lock()
        self._tcps = {}
        self._selected = 'tool0'
        if self.path.exists():
            data = json.loads(self.path.read_text(encoding='utf-8'))
            if data.get('version') != 1 or not isinstance(data.get('tcps'), dict):
                raise ValueError(f'Invalid TCP file: {self.path}')
            for name, pose in data['tcps'].items():
                self._tcps[name] = validate_pose(name, pose['xyz_m'], pose['rpy_deg'])
            selected = data.get('selected', 'tool0')
            if selected != 'tool0' and selected not in self._tcps:
                raise ValueError(f'Unknown selected TCP in {self.path}: {selected}')
            self._selected = selected

    def _persist(self, tcps, selected):
        self.path.parent.mkdir(parents=True, exist_ok=True)
        temporary = None
        try:
            with tempfile.NamedTemporaryFile('w', encoding='utf-8',
                                             dir=self.path.parent, prefix='.tcps-',
                                             suffix='.json', delete=False) as file:
                temporary = Path(file.name)
                json.dump({'version': 1, 'selected': selected, 'tcps': tcps}, file, indent=2)
                file.write('\n')
            os.replace(temporary, self.path)
        finally:
            if temporary is not None and temporary.exists():
                temporary.unlink()

    def snapshot(self):
        with self._lock:
            entries = [{'name': 'tool0', 'frame': 'tool0',
                        'xyz_m': [0.0, 0.0, 0.0], 'rpy_deg': [0.0, 0.0, 0.0]}]
            entries += [
                {'name': name, 'frame': frame_for(name),
                 'xyz_m': list(pose['xyz_m']), 'rpy_deg': list(pose['rpy_deg'])}
                for name, pose in sorted(self._tcps.items())
            ]
            return {'selected_tcp': self._selected, 'tcps': entries}

    def select(self, name):
        with self._lock:
            if name != 'tool0' and name not in self._tcps:
                raise ValueError(f'Unknown TCP: {name}')
            self._persist(self._tcps, name)
            self._selected = name

    def save(self, name, xyz_m, rpy_deg):
        pose = validate_pose(name, xyz_m, rpy_deg)
        with self._lock:
            if name not in self._tcps and len(self._tcps) >= 32:
                raise ValueError('Maximum of 32 saved TCPs reached')
            updated = dict(self._tcps)
            updated[name] = pose
            self._persist(updated, name)
            self._tcps = updated
            self._selected = name

    def delete(self, name):
        with self._lock:
            if name not in self._tcps:
                raise ValueError(f'Unknown saved TCP: {name}')
            updated = dict(self._tcps)
            del updated[name]
            selected = 'tool0' if self._selected == name else self._selected
            self._persist(updated, selected)
            self._tcps = updated
            self._selected = selected
