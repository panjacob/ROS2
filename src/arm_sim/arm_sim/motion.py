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

    def command(self, names, positions):
        self.jog = {name: 0.0 for name in self.limits}
        self.jog_stamp = None
        for name, value in zip(names, positions):
            if name in self.limits and math.isfinite(value):
                low, high, _ = self.limits[name]
                self.target[name] = max(low, min(high, value))

    def set_jog(self, names, velocities, now):
        self.target = self.position.copy()
        self.jog = {name: 0.0 for name in self.limits}
        for name, value in zip(names, velocities):
            if name in self.limits and math.isfinite(value):
                speed = self.limits[name][2]
                self.jog[name] = max(-speed, min(speed, value))
        self.jog_stamp = now

    def stop(self):
        self.target = self.position.copy()
        self.jog = {name: 0.0 for name in self.limits}
        self.jog_stamp = None

    def step(self, dt, now):
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
