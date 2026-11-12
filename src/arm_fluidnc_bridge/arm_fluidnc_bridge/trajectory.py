"""FollowJointTrajectory validation and G-code planning (no ROS imports, easy to unit test).

ROS units (radians, seconds) come in; G-code in FluidNC units (degrees, deg/min) goes out.
The bridge follows positions and time_from_start only: each point becomes one absolute G1
segment whose feed makes the segment last its time difference. Velocities/accelerations are
checked for size and finiteness but are not tracked (FluidNC plans its own acceleration).
"""

import bisect
import math

from arm_fluidnc_bridge.fluidnc_protocol import build_move, compute_feed

# Codes are the names of control_msgs/FollowJointTrajectory.Result constants.
INVALID_GOAL = 'INVALID_GOAL'
INVALID_JOINTS = 'INVALID_JOINTS'

# Segments shorter than the G-code resolution (0.001 deg) become a dwell instead of a G1.
MIN_MOVE_DEG = 1e-3
VELOCITY_MARGIN = 1.05


class TrajectoryError(ValueError):

    def __init__(self, code, message):
        super().__init__(message)
        self.code = code


class TrajectoryPlan:
    """Validated trajectory: reference samples in configured joint order plus G-code lines."""

    def __init__(self, joints, times, positions, lines):
        self.joints = joints
        self.times = times          # seconds, strictly increasing, times[0] == 0.0
        self.positions = positions  # rad, configured order, positions[0] = start reference
        self.lines = lines          # G-code, one per segment (G1 or G4 dwell)

    @property
    def duration(self):
        return self.times[-1]

    @property
    def final(self):
        return self.positions[-1]

    def sample(self, elapsed):
        """Desired (positions, velocities) at `elapsed` seconds, linear between points."""
        if elapsed <= 0.0 or len(self.times) == 1:
            return list(self.positions[0]), [0.0] * len(self.joints)
        if elapsed >= self.times[-1]:
            return list(self.positions[-1]), [0.0] * len(self.joints)
        i = bisect.bisect_right(self.times, elapsed)
        t0, t1 = self.times[i - 1], self.times[i]
        p0, p1 = self.positions[i - 1], self.positions[i]
        f = (elapsed - t0) / (t1 - t0)
        return ([a + f * (b - a) for a, b in zip(p0, p1)],
                [(b - a) / (t1 - t0) for a, b in zip(p0, p1)])


def _finite(values):
    return all(isinstance(v, (int, float)) and math.isfinite(v) for v in values)


def map_joint_order(joint_names, joints):
    """Index of each configured joint in the goal's joint_names (goal order may differ)."""
    names = list(joint_names)
    if len(set(names)) != len(names):
        raise TrajectoryError(INVALID_JOINTS, 'Trajectory joint_names contain duplicates')
    if set(names) != set(joints):
        missing = sorted(set(joints) - set(names))
        extra = sorted(set(names) - set(joints))
        raise TrajectoryError(
            INVALID_JOINTS,
            f'Trajectory must name exactly {list(joints)} (missing {missing}, unknown {extra})')
    return [names.index(name) for name in joints]


def plan_trajectory(joint_names, points, joints, axes, limits, current_rad,
                    start_tolerance, max_feed):
    """Validate a joint trajectory and turn it into FluidNC G-code.

    joint_names: goal joint order. points: list of dicts with 'time' [s], 'positions' [rad],
    optional 'velocities', 'accelerations', 'effort'. joints/axes: configured joint order and
    FluidNC axis letters. limits: name -> (lower, upper, max_velocity) [rad, rad/s].
    current_rad: measured position in configured order. max_feed: deg/min.

    Requested speeds above the URDF velocity limit or above max_feed are rejected, never
    clamped, so the executed timing never silently deviates from time_from_start.
    """
    order = map_joint_order(joint_names, joints)
    n = len(joints)
    if not points:
        raise TrajectoryError(INVALID_GOAL, 'Trajectory has no points')
    if len(current_rad) != n or not _finite(current_rad):
        raise TrajectoryError(INVALID_GOAL, 'Current position is unknown')
    for name in joints:
        if name not in limits:
            raise TrajectoryError(INVALID_JOINTS, f'No URDF limits for {name}')

    times = [0.0]
    positions = [list(current_rad)]
    lines = []
    for k, point in enumerate(points):
        when = point['time']
        if not isinstance(when, (int, float)) or not math.isfinite(when) or when < 0.0:
            raise TrajectoryError(INVALID_GOAL, f'Point {k}: time_from_start must be finite and >= 0')
        raw = list(point['positions'])
        if len(raw) != n:
            raise TrajectoryError(INVALID_GOAL, f'Point {k}: expected {n} positions, got {len(raw)}')
        for field in ('velocities', 'accelerations'):
            values = list(point.get(field) or [])
            if values and (len(values) != n or not _finite(values)):
                raise TrajectoryError(
                    INVALID_GOAL, f'Point {k}: {field} must be empty or {n} finite values')
        if list(point.get('effort') or []):
            raise TrajectoryError(INVALID_GOAL, f'Point {k}: effort is not supported')
        if not _finite(raw):
            raise TrajectoryError(INVALID_GOAL, f'Point {k}: positions must be finite')
        target = [raw[i] for i in order]
        for name, value in zip(joints, target):
            lower, upper, _vel = limits[name]
            if value < lower - 1e-6 or value > upper + 1e-6:
                raise TrajectoryError(
                    INVALID_GOAL,
                    f'Point {k}: {name}={value:.4f} rad is outside URDF limits '
                    f'[{lower:.4f}, {upper:.4f}]')

        if when == 0.0:
            if k != 0:
                raise TrajectoryError(INVALID_GOAL, 'time_from_start must be strictly increasing')
            for name, value, actual in zip(joints, target, current_rad):
                if abs(value - actual) > start_tolerance:
                    raise TrajectoryError(
                        INVALID_GOAL,
                        f'Trajectory start differs from current state: {name} is '
                        f'{actual:.4f} rad, trajectory starts at {value:.4f} rad '
                        f'(tolerance {start_tolerance:.4f})')
            positions[0] = target
            continue
        if when <= times[-1]:
            raise TrajectoryError(INVALID_GOAL, 'time_from_start must be strictly increasing')

        dt = when - times[-1]
        previous = positions[-1]
        for name, a, b in zip(joints, previous, target):
            speed = abs(b - a) / dt
            max_vel = limits[name][2]
            if speed > max_vel * VELOCITY_MARGIN + 1e-9:
                raise TrajectoryError(
                    INVALID_GOAL,
                    f'Point {k}: {name} needs {speed:.3f} rad/s, URDF limit is {max_vel:.3f} rad/s')
        start_deg = [math.degrees(v) for v in previous]
        target_deg = [math.degrees(v) for v in target]
        dist = math.sqrt(sum((b - a) ** 2 for a, b in zip(start_deg, target_deg)))
        if dist < MIN_MOVE_DEG:
            lines.append(f'G4 P{dt:.3f}')
        else:
            feed = dist / dt * 60.0
            if feed > max_feed * (1.0 + 1e-9):
                raise TrajectoryError(
                    INVALID_GOAL,
                    f'Point {k}: needs feed {feed:.0f} deg/min, above max_feed {max_feed:.0f}')
            lines.append(build_move(axes, target_deg, compute_feed(start_deg, target_deg, dt, max_feed)))
        times.append(when)
        positions.append(target)
    return TrajectoryPlan(list(joints), times, positions, lines)


def goal_tolerances(tolerances, joints, default):
    """Final position tolerance per configured joint [rad] from control_msgs/JointTolerance.

    position > 0 overrides the default, 0 keeps it. Velocity/acceleration tolerances >= 0 are
    satisfied by requiring FluidNC Idle (at rest). Negative values ("no check") are rejected.
    """
    result = {name: default for name in joints}
    for tol in tolerances:
        name = tol['name']
        if name not in result:
            raise TrajectoryError(INVALID_JOINTS, f'Goal tolerance for unknown joint "{name}"')
        values = (tol['position'], tol['velocity'], tol['acceleration'])
        if not _finite(values) or any(v < 0.0 for v in values):
            raise TrajectoryError(
                INVALID_GOAL, f'Goal tolerance for {name} must be finite and >= 0')
        if tol['position'] > 0.0:
            result[name] = tol['position']
    return [result[name] for name in joints]


def allowed_duration(duration, goal_time_tolerance, scaling, margin):
    """Seconds allowed until the final point must be reached and settled.

    goal_time_tolerance > 0 is honoured as-is (duration + tolerance); 0 uses the default
    duration * scaling + margin.
    """
    if not math.isfinite(goal_time_tolerance) or goal_time_tolerance < 0.0:
        raise TrajectoryError(INVALID_GOAL, 'goal_time_tolerance must be finite and >= 0')
    if goal_time_tolerance > 0.0:
        return duration + goal_time_tolerance
    return duration * scaling + margin


def within_tolerance(actual, target, tolerances):
    """Return the name/error of the first joint outside its tolerance, or None."""
    for i, (a, b, tol) in enumerate(zip(actual, target, tolerances)):
        if abs(a - b) > tol:
            return i, a - b
    return None
