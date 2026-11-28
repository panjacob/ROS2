"""NanoBee firmware protocol helpers (no ROS imports, easy to unit test).

The firmware works in motor steps; ROS uses radians. Conversion factors and limits come from
the board itself ("SETTINGS?"), so config.yaml on the board is the single source of truth.
See NanoBee/src/main.cpp for the command list.
"""

import math
import re
from dataclasses import dataclass, field

import yaml

NUM_JOINTS = 5
STEP_TICK_HZ = 20000           # firmware ISR rate: timed segments are rounded to 50 us
MAX_SEGMENT_MS = 60000.0       # longest T segment the firmware accepts
SPEED_MARGIN = 1.02            # firmware accepts T segments up to max_speed * 1.02 + 1 steps/s
HOMING_MODES = ('none', 'edge', 'center')
MAX_LINE = 255                 # firmware serial line buffer (256 incl. terminator)

STATUS_RE = re.compile(
    r'^<(?P<state>\w+) P:(?P<pos>-?\d+(?:,-?\d+)*) Q:(?P<queued>\d+)(?: F:(?P<free>\d+))?'
    r' E:(?P<enabled>[01]) L:(?P<endstops>[01]*)(?: H:(?P<referenced>[01]+))?'
    r'(?: O:(?P<powered>[01]+))?>$')
VERSION_RE = re.compile(r'^NanoBee (?P<protocol>\d+) joints=(?P<joints>\d+)$')
MIN_PROTOCOL = 2


class ProtocolError(ValueError):
    pass


def parse_status(line):
    """'<Idle P:0,0,0,0,0 Q:0 F:32 E:1 L:000 H:10000 O:11111>' -> dict, or None if not a status line.

    'enabled' is the armed state (E 1); 'powered' is per joint and also drops when the board
    switches an idle joint off (joints.jointN.idle_disable). Older firmware without O: reports 'enabled'.
    """
    m = STATUS_RE.match(line.strip())
    if not m:
        return None
    steps = [int(v) for v in m.group('pos').split(',')]
    referenced = m.group('referenced') or ''
    enabled = m.group('enabled') == '1'
    powered = m.group('powered')
    return {
        'state': m.group('state'),
        'steps': steps,
        'queued': int(m.group('queued')),
        'free': int(m.group('free')) if m.group('free') is not None else None,
        'enabled': enabled,
        'endstops': [c == '1' for c in m.group('endstops')],
        'referenced': [c == '1' for c in referenced] + [False] * (len(steps) - len(referenced)),
        'powered': ([c == '1' for c in powered] + [False] * (len(steps) - len(powered))
                    if powered is not None else [enabled] * len(steps)),
    }


def parse_version(line):
    m = VERSION_RE.match(line.strip())
    if not m and line.strip().startswith('err'):
        raise ProtocolError(
            f'the board runs an old NanoBee firmware (VER answered {line!r}); flash the current '
            'firmware from NanoBee/ (pio run -t upload) and push NanoBeeDriver/config.yaml')
    if not m:
        raise ProtocolError(f'board is not running NanoBee firmware (VER answered {line!r})')
    protocol, joints = int(m.group('protocol')), int(m.group('joints'))
    if protocol < MIN_PROTOCOL:
        raise ProtocolError(
            f'NanoBee protocol {protocol} is too old (need {MIN_PROTOCOL}+); flash the current '
            'firmware from NanoBee/')
    if joints != NUM_JOINTS:
        raise ProtocolError(f'NanoBee reports {joints} joints, expected {NUM_JOINTS}')
    return protocol


@dataclass
class BoardSettings:
    """Settings in use on the board (reply to SETTINGS?). Lists are per firmware joint."""

    steps_per_deg: list
    max_speed: list                # steps/s
    max_accel: list                # steps/s^2
    min_speed: float               # steps/s, start/stop speed
    homing: list                   # 'none' | 'edge' | 'center'
    soft_limits: bool
    min_deg: list
    max_deg: list
    blend: bool = True
    homing_enabled: bool = True
    jog_timeout_ms: int = 500
    raw: dict = field(default_factory=dict, repr=False)

    def to_steps(self, j, rad):
        return int(round(math.degrees(rad) * self.steps_per_deg[j]))

    def to_rad(self, j, steps):
        return math.radians(steps / self.steps_per_deg[j])

    def max_velocity(self, j):
        """rad/s"""
        return math.radians(self.max_speed[j] / self.steps_per_deg[j])

    def limit_steps(self, j):
        """Soft-limit range in steps, rounded like the firmware (lroundf)."""
        return (_lround(self.min_deg[j] * self.steps_per_deg[j]),
                _lround(self.max_deg[j] * self.steps_per_deg[j]))


def _lround(x):
    return int(math.floor(abs(x) + 0.5)) * (1 if x >= 0 else -1)


def _per_joint(joints, key, n, cast):
    """Values of `key` for joint1..jointN blocks, or the older form joints.<key>: [list]."""
    blocks = [joints.get(f'joint{j + 1}') for j in range(n)]
    if not any(isinstance(b, dict) for b in blocks):
        values = joints.get(key)
        if not isinstance(values, list) or len(values) != n:
            raise ProtocolError(
                f'settings: joints needs joint1..joint{n} blocks (or joints.{key} as a list '
                f'of {n} values)')
        names = [f'joints.{key}'] * n
    else:
        values, names = [], []
        for j, block in enumerate(blocks):
            name = f'joints.joint{j + 1}.{key}'
            if not isinstance(block, dict) or key not in block:
                raise ProtocolError(f'settings: {name} is missing')
            values.append(block[key])
            names.append(name)
    out = []
    for name, v in zip(names, values):
        try:
            out.append(cast(v))
        except (TypeError, ValueError):
            raise ProtocolError(f'settings: {name} has an invalid value') from None
    return out


def parse_settings(text, n=NUM_JOINTS):
    """Parse the SETTINGS? YAML dump into BoardSettings."""
    try:
        data = yaml.safe_load(text) or {}
    except yaml.YAMLError as exc:
        raise ProtocolError(f'settings are not valid YAML: {exc}') from None
    if not isinstance(data, dict):
        raise ProtocolError('settings are not a YAML mapping')
    motion = data.get('motion') or {}
    joints = data.get('joints') or {}
    homing = data.get('homing') or {}
    if not all(isinstance(sec, dict) for sec in (motion, joints, homing)):
        raise ProtocolError('settings: motion, joints and homing must be YAML mappings')
    if not joints:
        raise ProtocolError('settings have no joints section')

    def homing_mode(v):
        v = str(v).lower()
        if v not in HOMING_MODES:
            raise ValueError(v)
        return v

    s = BoardSettings(
        steps_per_deg=_per_joint(joints, 'steps_per_deg', n, float),
        max_speed=_per_joint(joints, 'max_speed', n, float),
        max_accel=_per_joint(joints, 'max_accel', n, float),
        min_speed=float(motion.get('min_speed', 100)),
        homing=_per_joint(joints, 'homing', n, homing_mode),
        soft_limits=bool(joints.get('soft_limits', False)),
        min_deg=_per_joint(joints, 'min_deg', n, float),
        max_deg=_per_joint(joints, 'max_deg', n, float),
        blend=bool(motion.get('blend', True)),
        homing_enabled=bool(homing.get('enabled', True)),
        jog_timeout_ms=int(motion.get('jog_timeout_ms', 500)),
        raw=data,
    )
    if any(v <= 0 for v in s.steps_per_deg + s.max_speed + s.max_accel):
        raise ProtocolError('settings: steps_per_deg, max_speed and max_accel must be positive')
    return s


FIRMWARE_LIMIT_DEG = 180.0     # firmware default for a missing joints.jointN.min_deg / max_deg


def config_joint_limits(text, n=NUM_JOINTS):
    """[(min_deg, max_deg)] per joint from a config.yaml, as the board will apply it.

    Accepts joint1..jointN blocks or the older joints.min_deg / max_deg lists; a missing value
    gets the firmware default (-/+180). Used by arm.launch.py for the URDF joint limits.
    """
    try:
        data = yaml.safe_load(text) or {}
    except yaml.YAMLError as exc:
        raise ProtocolError(f'config is not valid YAML: {exc}') from None
    joints = data.get('joints') if isinstance(data, dict) else None
    if not isinstance(joints, dict):
        joints = {}
    blocks = [joints.get(f'joint{j + 1}') for j in range(n)]
    limits = []
    for j in range(n):
        pair = []
        for key, default in (('min_deg', -FIRMWARE_LIMIT_DEG), ('max_deg', FIRMWARE_LIMIT_DEG)):
            block, values = blocks[j], joints.get(key)
            if isinstance(block, dict):
                value, name = block.get(key, default), f'joints.joint{j + 1}.{key}'
            elif isinstance(values, list) and len(values) == n:
                value, name = values[j], f'joints.{key}[{j}]'
            else:
                value, name = default, f'joints.joint{j + 1}.{key}'
            try:
                value = float(value)
            except (TypeError, ValueError):
                raise ProtocolError(f'{name} has an invalid value') from None
            if not math.isfinite(value):
                raise ProtocolError(f'{name} must be finite')
            pair.append(value)
        if pair[0] >= pair[1]:
            raise ProtocolError(f'joint{j + 1}: min_deg must be below max_deg')
        limits.append(tuple(pair))
    return limits


def config_upload(text):
    """Serial upload of a config.yaml ("CONFIG", lines, "END"), without the final newline.

    The firmware validates the settings; this only rejects text the line protocol can't carry.
    """
    lines = text.replace('\r\n', '\n').replace('\r', '\n').split('\n')
    while lines and not lines[-1].strip():
        lines.pop()
    if not lines:
        raise ProtocolError('config is empty')
    for n, line in enumerate(lines, 1):
        if '\t' in line:
            raise ProtocolError(f'line {n}: tabs not allowed, indent with spaces')
        if not (line.isascii() and line.isprintable()):
            raise ProtocolError(f'line {n}: only plain ASCII characters are allowed')
        if len(line) > MAX_LINE:
            raise ProtocolError(f'line {n}: longer than {MAX_LINE} characters')
        if line.strip() == 'END':
            raise ProtocolError(f"line {n}: a line may not be just 'END'")
    return 'CONFIG\n' + '\n'.join(lines) + '\nEND'


def format_steps(values):
    return ' '.join(str(int(v)) for v in values)


def format_speed(value):
    text = f'{value:.6f}'.rstrip('0')
    return text + '0' if text.endswith('.') else text


def move_line(steps, speed=None):
    line = 'M ' + format_steps(steps)
    if speed is not None:
        line += f' {format_speed(speed)}'
    return line


def segment_line(steps, dt_ms):
    return f'T {format_steps(steps)} {dt_ms:.3f}'


def jog_line(velocities):
    """velocities in steps/s"""
    return 'J ' + ' '.join(format_speed(v) for v in velocities)
