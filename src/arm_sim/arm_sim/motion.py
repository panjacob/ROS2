"""Joint-level motion model shared by the ROS node and tests."""

import math
import xml.etree.ElementTree as ET


def joints_from_urdf(description):
    """Return joint name -> (lower, upper, max velocity), in ROS units."""
    joints = {}
    for joint in ET.fromstring(description).findall('joint'):
        if joint.get('type') != 'revolute':
            continue
        limit = joint.find('limit')
        if limit is None:
            raise ValueError(f"Missing limits for {joint.get('name')}")
        lower, upper, velocity = (float(limit.get(key)) for key in ('lower', 'upper', 'velocity'))
        if not all(map(math.isfinite, (lower, upper, velocity))) or lower >= upper or velocity <= 0:
            raise ValueError(f"Invalid limits for {joint.get('name')}")
        joints[joint.get('name')] = (lower, upper, velocity)
    if not joints:
        raise ValueError('URDF has no revolute joints')
    return joints


class JointMotion:
    def __init__(self, limits, jog_timeout=0.3):
        self.limits = limits
        self.position = {name: 0.0 for name in limits}
        self.target = self.position.copy()
        self.jog = {name: 0.0 for name in limits}
        self.jog_stamp = None
        self.jog_timeout = jog_timeout
        self.trajectory = None
        self.trajectory_started = None
        self.trajectory_result = None

    def start_trajectory(self, names, points, now):
        """Follow timed (seconds, positions) points by linear interpolation."""
        if len(names) != len(self.limits) or set(names) != set(self.limits) or not points:
            raise ValueError('Trajectory must contain every arm joint exactly once')
        previous_time = -1.0
        previous = self.position.copy()
        checked = []
        for when, positions in points:
            if (not math.isfinite(when) or when < 0 or when <= previous_time or
                    len(positions) != len(names)):
                raise ValueError('Trajectory points need increasing times and all joint positions')
            target = dict(zip(names, positions))
            for name, value in target.items():
                low, high, _speed = self.limits[name]
                if not math.isfinite(value) or value < low - 1e-6 or value > high + 1e-6:
                    raise ValueError(f'Trajectory position for {name} is outside URDF limits')
            if when == 0 and any(abs(target[name] - previous[name]) > 0.02 for name in names):
                raise ValueError('Trajectory start differs from current state')
            checked.append((when, target))
            previous_time = when
        self.jog = {name: 0.0 for name in self.limits}
        self.jog_stamp = None
        self.target = self.position.copy()
        self.trajectory = checked
        self.trajectory_started = now
        self.trajectory_result = 'active'

    def command(self, names, positions):
        if self.trajectory is not None:
            self.trajectory = None
            self.trajectory_result = 'interrupted'
        self.jog = {name: 0.0 for name in self.limits}
        self.jog_stamp = None
        for name, value in zip(names, positions):
            if name in self.limits and math.isfinite(value):
                low, high, _ = self.limits[name]
                self.target[name] = max(low, min(high, value))

    def set_jog(self, names, velocities, now):
        if self.trajectory is not None:
            self.trajectory = None
            self.trajectory_result = 'interrupted'
        self.target = self.position.copy()
        self.jog = {name: 0.0 for name in self.limits}
        for name, value in zip(names, velocities):
            if name in self.limits and math.isfinite(value):
                speed = self.limits[name][2]
                self.jog[name] = max(-speed, min(speed, value))
        self.jog_stamp = now

    def stop(self):
        if self.trajectory is not None:
            self.trajectory = None
            self.trajectory_result = 'interrupted'
        self.target = self.position.copy()
        self.jog = {name: 0.0 for name in self.limits}
        self.jog_stamp = None

    def step(self, dt, now):
        if self.trajectory is not None:
            previous = self.position.copy()
            elapsed = max(0.0, now - self.trajectory_started)
            before_time = 0.0
            before = self.target
            for when, after in self.trajectory:
                if elapsed <= when:
                    fraction = 1.0 if when == before_time else (
                        (elapsed - before_time) / (when - before_time))
                    self.position = {
                        name: before[name] + fraction * (after[name] - before[name])
                        for name in self.limits}
                    break
                before_time, before = when, after
            else:
                self.position = self.trajectory[-1][1].copy()
                self.target = self.position.copy()
                self.trajectory = None
                self.trajectory_result = 'completed'
            return any(abs(self.position[name] - previous[name]) > 1e-9
                       for name in self.limits)
        if self.jog_stamp is not None and now - self.jog_stamp > self.jog_timeout:
            self.stop()
        moving = False
        for name, (low, high, speed) in self.limits.items():
            current = self.position[name]
            velocity = self.jog[name]
            if velocity:
                next_pos = current + velocity * dt
            else:
                difference = self.target[name] - current
                next_pos = current + max(-speed * dt, min(speed * dt, difference))
            self.position[name] = max(low, min(high, next_pos))
            moving |= abs(self.position[name] - current) > 1e-9
        return moving
