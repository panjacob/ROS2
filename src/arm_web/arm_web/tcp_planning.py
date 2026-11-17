"""Position-only IK for a registered point on tool0, independent of ROS.

MoveIt's KDL position-only solver controls the tool0 origin. Its pose sampler
subtracts an offset using a sampled orientation which KDL is free to change.
Instead solve p(q) + R(q) @ offset directly, then give the verified joint goals
to MoveIt for collision checking and trajectory planning. No orientation is
constrained, and a TCP's RPY rotates its axes, not its origin.
"""

import math
import time
import xml.etree.ElementTree as ET

import numpy as np


POSITION_TOLERANCE_M = 0.001
IK_TOLERANCE_M = 0.00005


def selected_tcp(snapshot, name):
    if not isinstance(name, str) or name != snapshot['selected_tcp']:
        raise ValueError('TCP selection changed; select the tool and plan again.')
    for tcp in snapshot['tcps']:
        if tcp['name'] == name:
            return {**tcp, 'xyz_m': list(tcp['xyz_m']), 'rpy_deg': list(tcp['rpy_deg'])}
    raise ValueError('Selected TCP is unavailable.')


def validate_tcp_goal(body, snapshot):
    tcp = selected_tcp(snapshot, body.get('tcp_name'))
    xyz = []
    for axis in 'xyz':
        value = body.get(f'{axis}_mm')
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise ValueError(f'{axis.upper()} must be a finite number in mm.')
        try:
            value = float(value) / 1000.0
        except (ValueError, OverflowError) as exc:
            raise ValueError(f'{axis.upper()} must be a finite number in mm.') from exc
        if not math.isfinite(value):
            raise ValueError(f'{axis.upper()} must be a finite number in mm.')
        xyz.append(value)
    return tcp, xyz


def quaternion_matrix(x, y, z, w):
    q = np.asarray([x, y, z, w], dtype=float)
    norm = np.linalg.norm(q)
    if not np.all(np.isfinite(q)) or norm < 1e-12:
        raise ValueError('Invalid frame rotation.')
    x, y, z, w = q / norm
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)],
    ])


def transformed_point(translation, rotation, point):
    return np.asarray(translation) + quaternion_matrix(*rotation) @ np.asarray(point)


def _rotation(axis, angle):
    x, y, z = axis
    skew = np.array([[0., -z, y], [z, 0., -x], [-y, x, 0.]])
    return np.eye(3) + math.sin(angle)*skew + (1-math.cos(angle))*(skew @ skew)


def _origin(joint):
    result = np.eye(4)
    origin = joint.find('origin')
    if origin is not None:
        result[:3, 3] = [float(v) for v in origin.get('xyz', '0 0 0').split()]
        roll, pitch, yaw = [float(v) for v in origin.get('rpy', '0 0 0').split()]
        result[:3, :3] = (_rotation([0, 0, 1], yaw) @
                          _rotation([0, 1, 0], pitch) @ _rotation([1, 0, 0], roll))
    return result


class PointKinematics:
    """URDF serial-chain FK and bounded damped least-squares point IK.

    Only fixed, bounded revolute and prismatic joints are supported. Unsupported
    models fail closed; no silent assumptions about mimic or floating joints.
    Limits, axes, fixed rotations and translations come from the loaded URDF.
    """

    def __init__(self, urdf, limits, tip='tool0'):
        robot = ET.fromstring(urdf)
        by_child = {j.find('child').get('link'): j for j in robot.findall('joint')}
        chain = []
        link = tip
        visited = set()
        while link in by_child:
            if link in visited:
                raise ValueError('URDF joint chain contains a cycle.')
            visited.add(link)
            joint = by_child[link]
            chain.append(joint)
            link = joint.find('parent').get('link')
        self.root_frame = link
        self.names = []
        self.chain = []
        for joint in reversed(chain):
            kind, name = joint.get('type'), joint.get('name')
            if kind not in ('fixed', 'revolute', 'prismatic') or joint.find('mimic') is not None:
                raise ValueError(f'Unsupported point IK joint: {name} ({kind}).')
            axis_el = joint.find('axis')
            axis = np.asarray([float(v) for v in (
                axis_el.get('xyz', '1 0 0') if axis_el is not None else '1 0 0').split()])
            if np.linalg.norm(axis) < 1e-12:
                raise ValueError(f'Invalid joint axis: {name}.')
            axis = axis / np.linalg.norm(axis)
            index = None
            if kind != 'fixed':
                if name not in limits:
                    raise ValueError(f'Point IK joint has no limits: {name}.')
                index = len(self.names)
                self.names.append(name)
            self.chain.append((_origin(joint), kind, axis, index))
        if not self.names or set(self.names) != set(limits):
            raise ValueError('Point IK requires the complete arm chain ending at tool0.')
        self.lower = np.array([limits[name][0] for name in self.names])
        self.upper = np.array([limits[name][1] for name in self.names])
        if not np.all(np.isfinite([self.lower, self.upper])) or np.any(self.lower >= self.upper):
            raise ValueError('Point IK requires finite joint limits.')
        self.reach_bound = sum(np.linalg.norm(origin[:3, 3]) for origin, _, _, _ in self.chain)
        self.reach_bound += sum(max(abs(self.lower[i]), abs(self.upper[i]))
                                for _, kind, _, i in self.chain if kind == 'prismatic')

    def point_and_jacobian(self, positions, offset):
        transform = np.eye(4)
        axes, origins, kinds = [], [], []
        for origin, kind, axis, index in self.chain:
            transform = transform @ origin
            if index is None:
                continue
            axes.append(transform[:3, :3] @ axis)
            origins.append(transform[:3, 3].copy())
            kinds.append(kind)
            motion = np.eye(4)
            if kind == 'revolute':
                motion[:3, :3] = _rotation(axis, positions[index])
            else:
                motion[:3, 3] = axis * positions[index]
            transform = transform @ motion
        point = transform[:3, 3] + transform[:3, :3] @ np.asarray(offset)
        jacobian = np.column_stack([
            np.cross(axis, point-origin) if kind == 'revolute' else axis
            for axis, origin, kind in zip(axes, origins, kinds)])
        return point, jacobian

    def point(self, joints, offset):
        positions = np.array([joints[name] for name in self.names], dtype=float)
        if not np.all(np.isfinite(positions)):
            raise ValueError('Joint positions must be finite.')
        return self.point_and_jacobian(positions, offset)[0]

    def solve(self, target, offset, current, *, timeout=2.5, max_solutions=24, cancelled=None):
        target, offset = np.asarray(target, dtype=float), np.asarray(offset, dtype=float)
        if not np.all(np.isfinite([target, offset])):
            raise ValueError('TCP target and offset must be finite.')
        if np.linalg.norm(target) > self.reach_bound + np.linalg.norm(offset) + IK_TOLERANCE_M:
            return []
        current_q = np.array([current[name] for name in self.names], dtype=float)
        if not np.all(np.isfinite(current_q)):
            raise ValueError('Current joint positions must be finite.')
        rng = np.random.default_rng(0)
        seeds = [np.clip(current_q, self.lower, self.upper), (self.lower+self.upper)/2]
        seeds += [rng.uniform(self.lower, self.upper) for _ in range(62)]
        solutions = []
        deadline = time.monotonic() + timeout
        for seed in seeds:
            q = seed.copy()
            for _ in range(100):
                if time.monotonic() > deadline or (cancelled and cancelled()):
                    return self._goals(solutions, current_q)
                point, jacobian = self.point_and_jacobian(q, offset)
                error = target-point
                distance = np.linalg.norm(error)
                if distance <= IK_TOLERANCE_M:
                    if all(np.linalg.norm(q-old) > 0.08 for old in solutions):
                        solutions.append(q.copy())
                    break
                step = jacobian.T @ np.linalg.solve(
                    jacobian @ jacobian.T + np.eye(3)*1e-6, error)
                step *= min(1., 0.35/max(np.linalg.norm(step), 1e-12))
                improved = False
                for scale in (1., 0.5, 0.25, 0.125):
                    candidate = np.clip(q + scale*step, self.lower, self.upper)
                    if np.linalg.norm(self.point_and_jacobian(candidate, offset)[0]-target) < distance-1e-10:
                        q, improved = candidate, True
                        break
                if not improved:
                    break
            if len(solutions) >= max_solutions:
                break
        return self._goals(solutions, current_q)

    def _goals(self, solutions, current):
        solutions.sort(key=lambda q: np.linalg.norm(q-current))
        return [dict(zip(self.names, map(float, q))) for q in solutions]
