"""Bounded joint jogging, independent of ROS for deterministic motion tests."""

import math
import xml.etree.ElementTree as ET


class JointJog:
    def __init__(self, description, timeout=0.3, horizon=0.12, state_timeout=0.5):
        self.limits = {}
        for joint in ET.fromstring(description).findall('joint'):
            if joint.get('type') not in ('revolute', 'prismatic'):
                continue
            limit = joint.find('limit')
            lower, upper, speed = [float(limit.get(key)) for key in ('lower', 'upper', 'velocity')]
            if not all(map(math.isfinite, (lower, upper, speed))) or lower >= upper or speed <= 0:
                raise ValueError('Jog requires bounded joints with positive velocity limits')
            self.limits[joint.get('name')] = (lower, upper, min(speed, math.radians(120)))
        if not self.limits:
            raise ValueError('No bounded joints in robot_description')
        self.names = list(self.limits)
        self.timeout, self.horizon, self.state_timeout = timeout, horizon, state_timeout
        self.positions = None
        self.state_stamp = -math.inf
        self.command_stamp = -math.inf
        self.velocity = dict.fromkeys(self.names, 0.0)
        self.active = False
        self.blocked = False
        self.settle_until = -math.inf
        self.last_sim_time = None

    def update(self, names, positions, now):
        values = dict(zip(names, positions))
        if not all(name in values and math.isfinite(values[name]) for name in self.names):
            return
        self.positions = [values[name] for name in self.names]
        self.state_stamp = now

    def fresh(self, now):
        return self.positions is not None and now - self.state_stamp < self.state_timeout

    def command(self, names, velocities, now):
        if not names or len(names) != len(velocities) or len(names) != len(set(names)):
            raise ValueError('Jog needs matching, unique joint names and velocities')
        if any(name not in self.limits for name in names) or not all(map(math.isfinite, velocities)):
            raise ValueError('Jog contains unknown joints or non-finite velocities')
        moving = any(velocities)
        if not moving:
            self.blocked = False
        if self.blocked or (moving and not self.fresh(now)):
            return False
        requested = dict(zip(names, velocities))
        self.velocity = {name: max(-limit[2], min(limit[2], requested.get(name, 0.0)))
                         for name, limit in self.limits.items()}
        self.command_stamp = now
        if moving:
            self.active = True
        return True

    def stop(self, now, disarm=False):
        was_active = self.active
        self.active = False
        self.velocity = dict.fromkeys(self.names, 0.0)
        self.blocked = disarm
        if was_active:
            self.settle_until = now + self.horizon
            if self.fresh(now):
                return list(self.positions)
        return None

    def busy(self, now):
        return self.active or now < self.settle_until

    def tick(self, now, simulation_time):
        previous = self.last_sim_time
        self.last_sim_time = simulation_time
        if not self.active:
            return None
        if (now - self.command_stamp >= self.timeout or not any(self.velocity.values()) or
                not self.fresh(now) or (previous is not None and simulation_time < previous)):
            hold = self.stop(now)
            return None if hold is None else (hold, True)
        # Never accumulate movement against wall time while Gazebo is paused.
        if previous is None or simulation_time <= previous:
            return None
        target = [max(self.limits[name][0], min(self.limits[name][1],
                      position + self.velocity[name] * self.horizon))
                  for name, position in zip(self.names, self.positions)]
        return target, False
