"""Joint goal validation shared by the planning API and its tests."""

import math
import xml.etree.ElementTree as ET


def joint_limits_from_urdf(urdf):
    root = ET.fromstring(urdf)
    limits = {}
    for joint in root.findall('joint'):
        if joint.get('type') not in ('revolute', 'prismatic'):
            continue
        limit = joint.find('limit')
        if limit is None:
            continue
        limits[joint.get('name')] = (
            float(limit.get('lower')), float(limit.get('upper')))
    return root.get('name'), limits


def validate_joint_goal(values, limits):
    if not isinstance(values, dict):
        raise ValueError('Expected a map of joint angles')
    missing = sorted(set(limits) - set(values))
    extra = sorted(set(values) - set(limits))
    if missing:
        raise ValueError('Missing joints: ' + ', '.join(missing))
    if extra:
        raise ValueError('Unknown joints: ' + ', '.join(extra))
    target = {}
    for name, (lower, upper) in limits.items():
        try:
            degrees = float(values[name])
        except (ValueError, TypeError) as exc:
            raise ValueError(f'{name}: angle must be a number') from exc
        if not math.isfinite(degrees):
            raise ValueError(f'{name}: angle must be finite')
        radians = math.radians(degrees)
        if radians < lower - 1e-9 or radians > upper + 1e-9:
            raise ValueError(
                f'{name}: {degrees:g}° is outside the URDF limits '
                f'[{math.degrees(lower):.1f}°, {math.degrees(upper):.1f}°]')
        target[name] = radians
    return target


def check_plan_start(current, planned, tolerance_rad=0.02):
    """Reject a preview if the robot has moved since it was planned."""
    for name, position in planned.items():
        actual = current.get(name)
        if actual is None or not math.isfinite(actual):
            raise ValueError(f'No current position for {name}')
        if abs(actual - position) > tolerance_rad:
            raise ValueError(
                f'{name} moved since planning; create a new plan before executing')
