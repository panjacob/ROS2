"""NanoBee arm driver over USB serial (needs pyserial) or Wi-Fi (HTTP).

    from nanobee import NanoBee
    arm = NanoBee(port="COM6", config="config.yaml")      # USB
    # arm = NanoBee("192.168.4.1", config="config.yaml")  # Wi-Fi
    arm.push_config()                  # upload config.yaml to the board (only needed after editing it)
    arm.enable()
    arm.move(0, 90, speed=30)          # joint 0 to 90 deg at 30 deg/s
    arm.move_all([10, 20, 0, 0, 0], speed=45, wait=True)
    arm.stop()

Angles are in degrees, converted with joints.steps_per_deg from config.yaml.
"""

import re
import time
import urllib.error
import urllib.parse
import urllib.request
from dataclasses import dataclass
from pathlib import Path

NUM_JOINTS = 5
_STATUS_RE = re.compile(r"<(\w+) P:([-\d,]+) Q:(\d+)(?: F:(\d+))? E:(\d) L:(\d*)(?: H:([01]+))?>")
_SPD_RE = re.compile(r"^\s*steps_per_deg\s*:\s*\[([^\]]*)\]", re.MULTILINE)


class NanoBeeError(Exception):
    pass


@dataclass
class Status:
    state: str  # Idle, Run, Jog, Home
    angles: list  # degrees
    steps: list
    queued: int
    enabled: bool
    endstops: str  # e.g. "010" for J0..J2, "1" = triggered
    free: int = 0  # free slots in the move queue
    referenced: str = ""  # per joint "1" = homed / zeroed (soft limits active)

    @property
    def idle(self):
        return self.state == "Idle" and self.queued == 0


class _Http:
    def __init__(self, host, timeout, events):
        self.base = f"http://{host}"
        self.timeout = timeout
        self.events = events

    def _request(self, path, data=None):
        req = urllib.request.Request(self.base + path, data=data, method="POST" if data is not None else "GET")
        if data is not None:
            req.add_header("Content-Type", "text/plain")
        try:
            with urllib.request.urlopen(req, timeout=self.timeout) as r:
                return r.read().decode()
        except urllib.error.HTTPError as e:
            raise NanoBeeError(e.read().decode().strip() or str(e)) from None
        except urllib.error.URLError as e:
            raise NanoBeeError(f"cannot reach {self.base} ({e.reason}); is the PC on the NanoBee Wi-Fi?") from None

    def command(self, line):
        return self._request("/cmd?c=" + urllib.parse.quote(line)).strip()

    def status_line(self):
        lines = self._request("/status").splitlines()
        self.events += [l.lstrip("#").strip() for l in lines[1:] if l]
        return lines[0] if lines else ""

    def pull_config(self):
        return self._request("/config")

    def pull_settings(self):
        return self._request("/settings")

    def push_config(self, yaml):
        return self._request("/config", data=yaml.encode())

    def reboot(self):
        self._request("/reboot", data=b"")

    def close(self):
        pass


class _Serial:
    # board messages that are not replies to the last command start with "# "
    _ASYNC = "#"

    def __init__(self, port, baud, timeout, events):
        try:
            import serial
        except ImportError:
            raise NanoBeeError("USB needs pyserial: pip install -r requirements.txt") from None
        self.events = events
        self.s = serial.Serial()
        self.s.port, self.s.baudrate, self.s.timeout = port, baud, timeout
        self.s.dtr = self.s.rts = False  # don't reset the ESP32 when opening the port
        try:
            self.s.open()
        except serial.SerialException as e:
            raise NanoBeeError(f"cannot open {port}: {e}") from None
        self._handshake()

    def _handshake(self):
        # the board may be booting; retry "?" until it answers
        deadline = time.monotonic() + 8
        while True:
            self.s.reset_input_buffer()
            self.s.write(b"?\n")
            end = time.monotonic() + 0.5
            while time.monotonic() < end:
                line = self.s.readline().decode(errors="replace").strip()
                if line.startswith("<"):
                    if not _STATUS_RE.match(line):
                        raise NanoBeeError(f"board is not running NanoBee firmware (got {line!r})")
                    return
            if time.monotonic() > deadline:
                raise NanoBeeError(f"no answer on {self.s.port}; is NanoBee firmware flashed?")

    def _readline(self):
        raw = self.s.readline()
        if not raw:
            raise NanoBeeError(f"timeout waiting for board on {self.s.port}")
        return raw.decode(errors="replace").strip()

    def _reply(self):
        while True:
            line = self._readline()
            if not line:
                continue
            if line.startswith(self._ASYNC):
                self.events.append(line[1:].strip())
            else:
                return line

    def command(self, line):
        if line.strip() == "!":  # real-time halt: the board answers asynchronously ("# halted")
            self.s.write(b"!")
            return "ok"
        self.s.write((line + "\n").encode())
        return self._reply()

    def status_line(self):
        return self.command("?")

    def pull_config(self, request=b"CONFIG?\n"):
        self.s.write(request)
        lines = []
        while (line := self._readline()) != "END":
            lines.append(line)
        return "\n".join(lines) + "\n"

    def pull_settings(self):
        return self.pull_config(b"SETTINGS?\n")

    def push_config(self, yaml):
        self.s.write(b"CONFIG\n" + yaml.replace("\r", "").encode() + b"\nEND\n")
        reply = self._reply()
        if reply.startswith("err"):
            raise NanoBeeError(reply)
        return reply

    def reboot(self):
        self.command("REBOOT")

    def close(self):
        self.s.close()


class NanoBee:
    def __init__(self, host=None, port=None, config=None, timeout=3.0, baud=115200):
        """port: USB serial port (e.g. "COM6"); otherwise Wi-Fi host (default 192.168.4.1).
        config: path to config.yaml; if omitted, steps_per_deg is read from the board."""
        self.config_path = Path(config) if config else None
        self.events = []  # async messages from the board (e.g. "homed 0")
        self._target = None  # last commanded position in steps
        if port:
            self._io = _Serial(port, baud, timeout, self.events)
        else:
            self._io = _Http(host or "192.168.4.1", timeout, self.events)
        yaml = self.config_path.read_text() if self.config_path else self.pull_settings()
        self.steps_per_deg = self._parse_spd(yaml)

    def close(self):
        self._io.close()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()

    def command(self, line):
        """Send a raw firmware command (see NanoBee/src/main.cpp); raises on 'err'."""
        reply = self._io.command(line)
        if reply.startswith("err"):
            raise NanoBeeError(f"{line!r}: {reply}")
        return reply

    # ---------- config ----------

    @staticmethod
    def _parse_spd(yaml):
        m = _SPD_RE.search(yaml)
        if not m:
            raise NanoBeeError("joints.steps_per_deg missing from config")
        spd = [float(x) for x in m.group(1).split(",")]
        if len(spd) != NUM_JOINTS or any(v <= 0 for v in spd):
            raise NanoBeeError("joints.steps_per_deg needs 5 positive values")
        return spd

    def pull_config(self):
        """config.yaml as stored on the board."""
        return self._io.pull_config()

    def pull_settings(self):
        """Settings in use (every key, defaults filled in)."""
        return self._io.pull_settings()

    def push_config(self, path=None):
        """Upload YAML to the board; applied at once (machine must be idle). Wi-Fi changes need reboot()."""
        path = Path(path) if path else self.config_path
        if not path:
            raise NanoBeeError("no config path given")
        yaml = path.read_text()
        spd = self._parse_spd(yaml)
        reply = self._io.push_config(yaml)
        self.steps_per_deg = spd
        return reply

    def reboot(self):
        self._io.reboot()

    # ---------- state ----------

    def status(self):
        line = self._io.status_line()
        m = _STATUS_RE.match(line)
        if not m:
            raise NanoBeeError(f"bad status: {line!r}")
        steps = [int(v) for v in m.group(2).split(",")]
        return Status(
            state=m.group(1),
            angles=[s / k for s, k in zip(steps, self.steps_per_deg)],
            steps=steps,
            queued=int(m.group(3)),
            free=int(m.group(4) or 0),
            enabled=m.group(5) == "1",
            endstops=m.group(6),
            referenced=m.group(7) or "",
        )

    def angles(self):
        return self.status().angles

    def wait(self, timeout=None, poll=0.05):
        """Block until all queued moves / homing are finished."""
        start = time.monotonic()
        seen = len(self.events)
        while True:
            st = self.status()
            errors = [e for e in self.events[seen:] if e.startswith("err")]
            if errors:
                raise NanoBeeError(errors[0])
            if st.idle:
                return st
            if timeout is not None and time.monotonic() - start > timeout:
                raise TimeoutError("arm still moving")
            time.sleep(poll)

    # ---------- motion ----------

    def enable(self, on=True):
        self.command("E 1" if on else "E 0")
        self._target = None

    def disable(self):
        self.enable(False)

    def move(self, joint, angle, speed, accel=None, relative=False, wait=False):
        """Move one joint to `angle` deg (or by `angle` if relative) at `speed` deg/s."""
        angles = [0.0 if relative else None] * NUM_JOINTS
        angles[joint] = angle
        self.move_all(angles, speed, accel, relative, wait)

    def move_all(self, angles, speed, accel=None, relative=False, wait=False):
        """Synchronized move; all joints arrive together, none faster than `speed` deg/s.

        angles: list of 5 (None = keep joint where it is) or dict {joint: angle}.
        accel: deg/s^2, default = firmware motion.default_accel (steps/s^2).
        """
        if isinstance(angles, dict):
            angles = [angles.get(j) for j in range(NUM_JOINTS)]
        if len(angles) != NUM_JOINTS:
            raise ValueError("need 5 angles")
        if speed <= 0 or (accel is not None and accel <= 0):
            raise ValueError("speed and accel must be positive")

        st = self.status()
        if st.idle or self._target is None:
            self._target = st.steps
        cur = self._target
        target = []
        for j, a in enumerate(angles):
            if a is None:
                target.append(cur[j])
            else:
                steps = round(a * self.steps_per_deg[j])
                target.append(cur[j] + steps if relative else steps)

        d_steps = [abs(t - c) for t, c in zip(target, cur)]
        d_deg = max(d / k for d, k in zip(d_steps, self.steps_per_deg))
        if d_deg == 0:
            return
        # firmware speed/accel apply to the joint with the most steps; scale so the
        # fastest joint in degrees moves at `speed`
        ratio = max(d_steps) / d_deg
        line = "M " + " ".join(map(str, target)) + f" {speed * ratio:.3f}"
        if accel is not None:
            line += f" {accel * ratio:.3f}"
        self.command(line)
        self._target = target
        if wait:
            self.wait()

    def stop(self):
        """Decelerate to a stop and drop queued moves."""
        self.command("S")
        self._target = None

    def halt(self):
        """Emergency stop, no deceleration."""
        self.command("!")
        self._target = None

    def home(self, joint=None, wait=True):
        """Home one joint, or every joint with joints.homing != none (joint=None).
        The home reference becomes joints.home_offset_deg."""
        self.command("H" if joint is None else f"H {joint}")
        self._target = None
        if wait:
            self.wait()

    def segment(self, angles, dt):
        """Queue a timed segment: reach `angles` (deg, list of 5) in `dt` seconds at constant speed.
        Back-to-back segments blend without stopping when motion.blend is on."""
        steps = [round(a * k) for a, k in zip(angles, self.steps_per_deg)]
        self.command("T " + " ".join(map(str, steps)) + f" {dt * 1000:.1f}")
        self._target = steps

    def zero(self, angles=None):
        """Declare the current position as `angles` (default all 0 deg)."""
        if angles is None:
            self.command("Z")
        else:
            self.command("Z " + " ".join(str(round(a * k)) for a, k in zip(angles, self.steps_per_deg)))
        self._target = None

    def gripper(self, us):
        """Servo pulse width 500..2500 us, 0 = off."""
        self.command(f"G {int(us)}")
