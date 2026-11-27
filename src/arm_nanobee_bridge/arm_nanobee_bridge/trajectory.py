"""FollowJointTrajectory validation and NanoBee segment planning (no ROS imports).

ROS units (radians, seconds) come in; NanoBee timed segments ("T p0..p4 dt_ms", absolute
motor steps) go out. Every trajectory point becomes one segment that the firmware runs at
constant speed and blends with the next one (motion.blend), so the arm follows the
positions and time_from_start of the goal. Velocities/accelerations are checked for size and
finiteness but are not tracked separately.

If the last segment still moves fast at its end (e.g. a single-point goal), a short
deceleration ramp at the joints' max_accel is appended so the arm stops on the final point
instead of overshooting it; the trajectory then ends at most max_speed / (2 * max_accel) late.
"""

import bisect
import math

from arm_nanobee_bridge.nanobee_protocol import MAX_SEGMENT_MS, SPEED_MARGIN

# Codes are the names of control_msgs/FollowJointTrajectory.Result constants.
INVALID_GOAL = 'INVALID_GOAL'
INVALID_JOINTS = 'INVALID_JOINTS'

VELOCITY_MARGIN = 1.05      # tolerance on URDF velocity limits (rounding in time parameterization)
RAMP_PIECE_MIN_S = 0.005    # shortest piece of the final deceleration ramp
RAMP_PIECES_MAX = 8


class TrajectoryError(ValueError):

    def __init__(self, code, message):
        super().__init__(message)
        self.code = code


class TrajectoryPlan:
    """Validated trajectory: reference samples in configured joint order plus T segments."""

    def __init__(self, joints, times, positions, segments, tail):
        self.joints = joints
        self.times = times          # seconds, strictly increasing, times[0] == 0.0
        self.positions = positions  # rad, configured order, positions[0] = start reference
        self.segments = segments    # [(steps list in firmware joint order, dt_ms)]
        self.tail = tail            # s added by the final deceleration ramp

    @property
    def duration(self):
        return self.times[-1]

    @property
    def execution_time(self):
        return self.times[-1] + self.tail

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


def check_segment(joints, settings, referenced, start, target, dt_ms, label):
    """Raise TrajectoryError if the firmware would reject this T segment (steps, ms)."""
    if not 0.0 < dt_ms <= MAX_SEGMENT_MS:
        raise TrajectoryError(
            INVALID_GOAL, f'{label}: segment of {dt_ms / 1000:.3f} s; NanoBee accepts '
            f'(0, {MAX_SEGMENT_MS / 1000:.0f}] s between points')
    for j, (name, a, b) in enumerate(zip(joints, start, target)):
        speed = abs(b - a) * 1000.0 / dt_ms
        limit = settings.max_speed[j] * SPEED_MARGIN + 1.0
        if speed > limit:
            raise TrajectoryError(
                INVALID_GOAL,
                f'{label}: {name} needs {speed / settings.steps_per_deg[j]:.1f} deg/s, '
                f'NanoBee joints.joint{j + 1}.max_speed is {settings.max_speed[j] / settings.steps_per_deg[j]:.1f}'
                ' deg/s')
        if settings.soft_limits and referenced[j]:
            lo, hi = settings.limit_steps(j)
            if not lo <= b <= hi:
                raise TrajectoryError(
                    INVALID_GOAL,
                    f'{label}: {name}={b / settings.steps_per_deg[j]:.2f} deg is outside the '
                    f'NanoBee soft limits [{settings.min_deg[j]:.1f}, {settings.max_deg[j]:.1f}]')


def stop_ramp(start, target, dt_s, settings):
    """Split the last segment so it ends at rest: constant part + deceleration at max_accel.

    Returns ([(steps, dt_ms)], extra_seconds). The constant part keeps the segment's speed;
    the ramp halves the speed of the remaining distance, so the end is reached extra seconds
    later. Without a fast end (all speeds below min_speed) the segment is returned unchanged.
    """
    vel = [(b - a) / dt_s for a, b in zip(start, target)]  # steps/s
    if all(abs(v) < settings.min_speed for v in vel):
        return [(list(target), dt_s * 1000.0)], 0.0
    ramp = max(abs(v) / a for v, a in zip(vel, settings.max_accel))
    ramp = min(ramp, 2.0 * dt_s)
    cruise = dt_s - ramp / 2.0
    if cruise < RAMP_PIECE_MIN_S:  # too short to keep: decelerate over the whole segment
        ramp, cruise = 2.0 * dt_s, 0.0
    segments = []
    t0 = cruise
    if cruise > 0.0:
        segments.append(([int(round(a + v * cruise)) for a, v in zip(start, vel)], cruise * 1000.0))
    pieces = max(1, min(RAMP_PIECES_MAX, int(ramp / RAMP_PIECE_MIN_S)))
    for k in range(1, pieces + 1):
        tau = ramp * k / pieces
        if k == pieces:
            steps = list(target)
        else:
            steps = [int(round(a + v * t0 + v * tau - v * tau * tau / (2.0 * ramp)))
                     for a, v in zip(start, vel)]
        segments.append((steps, ramp / pieces * 1000.0))
    return segments, ramp / 2.0


def plan_trajectory(joint_names, points, joints, limits, settings, current_steps, referenced,
                    start_tolerance):
    """Validate a joint trajectory and turn it into NanoBee T segments.

    joint_names: goal joint order. points: list of dicts with 'time' [s], 'positions' [rad],
    optional 'velocities', 'accelerations', 'effort'. joints: configured joint order (= firmware
    joint order). limits: name -> (lower, upper, max_velocity) [rad, rad/s] from the URDF.
    settings: BoardSettings. current_steps/referenced: from the latest status report.

    Requested speeds above the URDF velocity limit or the firmware max_speed are rejected, never
    clamped, so the executed timing never silently deviates from time_from_start.
    """
    order = map_joint_order(joint_names, joints)
    n = len(joints)
    if not points:
        raise TrajectoryError(INVALID_GOAL, 'Trajectory has no points')
    if len(current_steps) < n:
        raise TrajectoryError(INVALID_GOAL, 'Current position is unknown')
    for name in joints:
        if name not in limits:
            raise TrajectoryError(INVALID_JOINTS, f'No URDF limits for {name}')
    current_rad = [settings.to_rad(j, current_steps[j]) for j in range(n)]

    times = [0.0]
    positions = [current_rad]
    steps = [list(current_steps[:n])]
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
        for name, a, b in zip(joints, positions[-1], target):
            speed = abs(b - a) / dt
            max_vel = limits[name][2]
            if speed > max_vel * VELOCITY_MARGIN + 1e-9:
                raise TrajectoryError(
                    INVALID_GOAL,
                    f'Point {k}: {name} needs {speed:.3f} rad/s, URDF limit is {max_vel:.3f} rad/s')
        target_steps = [settings.to_steps(j, v) for j, v in enumerate(target)]
        check_segment(joints, settings, referenced, steps[-1], target_steps, dt * 1000.0,
                      f'Point {k}')
        times.append(when)
        positions.append(target)
        steps.append(target_steps)

    segments = [(steps[i], (times[i] - times[i - 1]) * 1000.0) for i in range(1, len(steps))]
    tail = 0.0
    if segments and settings.blend:
        last_dt = (times[-1] - times[-2])
        ramp, tail = stop_ramp(steps[-2], steps[-1], last_dt, settings)
        previous = steps[-2]
        for target_steps, dt_ms in ramp:
            check_segment(joints, settings, referenced, previous, target_steps, dt_ms,
                          'Final deceleration')
            previous = target_steps
        segments = segments[:-1] + ramp
    return TrajectoryPlan(list(joints), times, positions, segments, tail)


def goal_tolerances(tolerances, joints, default):
    """Final position tolerance per configured joint [rad] from control_msgs/JointTolerance.

    position > 0 overrides the default, 0 keeps it. Velocity/acceleration tolerances >= 0 are
    satisfied by requiring the board to be Idle (at rest). Negative values ("no check") are
    rejected.
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
    """Return the index/error of the first joint outside its tolerance, or None."""
    for i, (a, b, tol) in enumerate(zip(actual, target, tolerances)):
        if abs(a - b) > tol:
            return i, a - b
    return None
