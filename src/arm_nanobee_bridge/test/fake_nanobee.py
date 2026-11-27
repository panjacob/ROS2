"""Shared fake NanoBee board for the bridge tests (no hardware)."""

import threading

# Reply to SETTINGS? from the current firmware.
SETTINGS_YAML = """# NanoBee settings. Units: steps, steps/s, steps/s^2. joint1..joint5 = driver slots X, Y, Z, E0, E1.
wifi:
  mode: ap   # ap = own network, sta = join existing
  ssid: "NanoBee"
  password: "nanobee123"
  hostname: nanobee
motion:
  default_speed: 1800
  default_accel: 20000
  min_speed: 100
  jog_timeout_ms: 500
  blend: {blend}
  idle_timeout_ms: 5000
joints:
  soft_limits: true
  joint1:
    steps_per_deg: 17.7778
    max_speed: 6400
    max_accel: 53333
    dir_invert: true
    idle_disable: true
    homing: center
    homing_dir: 1
    home_offset_deg: 0.000
    min_deg: -175.000
    max_deg: 175.000
  joint2:
    steps_per_deg: 17.7778
    max_speed: 6400
    max_accel: 53333
    dir_invert: false
    idle_disable: false
    homing: none
    homing_dir: 1
    home_offset_deg: 0.000
    min_deg: -95.000
    max_deg: 95.000
  joint3:
    steps_per_deg: 17.7778
    max_speed: 6400
    max_accel: 53333
    dir_invert: false
    idle_disable: false
    homing: none
    homing_dir: 1
    home_offset_deg: 0.000
    min_deg: -140.000
    max_deg: 140.000
  joint4:
    steps_per_deg: 17.7778
    max_speed: 6400
    max_accel: 53333
    dir_invert: false
    idle_disable: false
    homing: none
    homing_dir: 1
    home_offset_deg: 0.000
    min_deg: -105.000
    max_deg: 105.000
  joint5:
    steps_per_deg: 17.7778
    max_speed: 6400
    max_accel: 53333
    dir_invert: false
    idle_disable: true
    homing: none
    homing_dir: 1
    home_offset_deg: 0.000
    min_deg: -185.000
    max_deg: 185.000
homing:
  enabled: true
  speed: 1600
  accel: 20000
  backoff: 89
  max_travel: 7111
endstops:
  active_low: false
"""

# Older firmware: one list per joint setting.
LEGACY_SETTINGS_YAML = """# NanoBee settings. Units: steps, steps/s, steps/s^2. Lists are per joint J0..J4.
wifi:
  mode: ap   # ap = own network, sta = join existing
  ssid: "NanoBee"
  password: "nanobee123"
  hostname: nanobee
motion:
  default_speed: 1800
  default_accel: 20000
  min_speed: 100
  jog_timeout_ms: 500
  blend: {blend}
  idle_timeout_ms: 5000
joints:
  steps_per_deg: [17.7778, 17.7778, 17.7778, 17.7778, 17.7778]
  max_speed: [6400, 6400, 6400, 6400, 6400]
  max_accel: [53333, 53333, 53333, 53333, 53333]
  dir_invert: [true, false, false, false, false]
  idle_disable: [true, false, false, false, true]
  homing_dir: [1, 1, 1, 1, 1]
  homing: [center, none, none, none, none]
  home_offset_deg: [0.000, 0.000, 0.000, 0.000, 0.000]
  soft_limits: true
  min_deg: [-175.000, -95.000, -140.000, -105.000, -185.000]
  max_deg: [175.000, 95.000, 140.000, 105.000, 185.000]
homing:
  enabled: true
  speed: 1600
  accel: 20000
  backoff: 89
  max_travel: 7111
endstops:
  active_low: false
"""


class FakeNanoBee:
    """Answers '?', VER, SETTINGS? and acks commands like the firmware.

    With auto=True one queued segment/move completes per status query; otherwise motion only
    completes when the test calls complete(). Nothing models real timing or dynamics.
    """

    def __init__(self, blend=True, capacity=16):
        self.cond = threading.Condition()
        self.out = []
        self.state = 'Idle'
        self.pos = [0] * 5
        self.queue = []
        self.lines = []
        self.realtime = []
        self.enabled = True
        self.referenced = [False] * 5
        self.powered = [True] * 5
        self.endstops = [False] * 3
        self.auto = True
        self.report = True
        self.closed = False
        self.capacity = capacity
        self.blend = blend
        self.replies = {}       # command letter -> forced reply
        self.config_lines = None  # lines of a CONFIG upload while receiving
        self.config_yaml = None   # last completed CONFIG upload

    def _emit(self, text):
        self.out.append(text.encode() + b'\n')
        self.cond.notify_all()

    def emit(self, text):
        with self.cond:
            self._emit(text)

    def _status(self):
        if self.auto and self.queue and self.state == 'Run':
            self.pos = self.queue.pop(0)
            if not self.queue:
                self.state = 'Idle'
        pos = ','.join(str(v) for v in self.pos)
        ends = ''.join('1' if e else '0' for e in self.endstops)
        ref = ''.join('1' if r else '0' for r in self.referenced)
        pwr = ''.join('1' if self.enabled and p else '0' for p in self.powered)
        self._emit(f'<{self.state} P:{pos} Q:{len(self.queue)} F:{self.capacity - len(self.queue)}'
                   f' E:{int(self.enabled)} L:{ends} H:{ref} O:{pwr}>')

    def write(self, data):
        with self.cond:
            if data == b'!':
                self.realtime.append(data)
                self.queue.clear()
                self.state = 'Idle'
                self._emit('# halted')
                return
            for line in data.decode().splitlines():
                if self.config_lines is not None:
                    if line != 'END':
                        self.config_lines.append(line)
                        continue
                    self.config_yaml = '\n'.join(self.config_lines) + '\n'
                    self.config_lines = None
                    self._emit(self.replies.get('CONFIG', 'ok'))
                    continue
                if line == 'CONFIG':
                    self.lines.append(line)
                    self.config_lines = []
                    continue
                if line == '?':
                    if self.report:
                        self._status()
                    continue
                self.lines.append(line)
                self._emit(self._command(line))

    def _command(self, line):
        word, *args = line.split()
        if word in self.replies:
            return self.replies[word]
        if word == 'VER':
            return 'NanoBee 2 joints=5'
        if word == 'SETTINGS?':
            for text in SETTINGS_YAML.format(blend='true' if self.blend else 'false').splitlines():
                self._emit(text)
            return 'END'
        if word in ('T', 'M'):
            if self.state in ('Home', 'Jog'):
                return 'err jogging' if self.state == 'Jog' else 'err homing'
            if len(self.queue) >= self.capacity:
                return 'err full'
            self.queue.append([int(v) for v in args[:5]])
            self.enabled = True
            self.powered = [True] * 5
            self.state = 'Run'
            return 'ok'
        if word == 'J':
            if self.state not in ('Idle', 'Jog'):
                return 'err busy'
            self.state = 'Jog' if any(float(v) for v in args) else 'Idle'
            return 'ok'
        if word == 'S':
            self.queue.clear()
            self.state = 'Idle'
            return 'ok'
        if word == 'E':
            self.enabled = args[0] == '1'
            self.powered = [True] * 5
            if not self.enabled:
                self.referenced = [False] * 5
            return 'ok'
        if word == 'H':
            self.state = 'Home'
            return 'ok'
        if word == 'Z':
            self.pos = [int(v) for v in args]
            return 'ok'
        return 'err unknown'

    def read(self):
        with self.cond:
            if not self.out:
                self.cond.wait(0.02)
            if self.closed:
                raise ConnectionError('closed')
            data, self.out = b''.join(self.out), []
            return data

    def complete(self):
        with self.cond:
            if self.queue:
                self.pos = self.queue[-1]
            self.queue.clear()
            self.state = 'Idle'

    def close(self):
        with self.cond:
            self.closed = True

    def t_lines(self):
        with self.cond:
            return [line for line in self.lines if line.startswith('T ')]
