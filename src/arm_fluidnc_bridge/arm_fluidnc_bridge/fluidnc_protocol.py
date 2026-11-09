"""Pure helpers for talking to FluidNC (no ROS imports, easy to unit test)."""

import math
import re

_STATUS_RE = re.compile(r'^<([^|>]+)\|(.*)>$')


def parse_status(line):
    """Parse a FluidNC status report into a dict.

    Example: '<Run|MPos:1.0,2.0,0.0|FS:600,0|Pn:XYZ>'
    Returns a dict or None if the line is not a status report:
        {
            'state':    str,              # e.g. 'Idle', 'Run', 'Hold:0'
            'positions': [float, ...] | None,  # MPos, falls back to WPos
            'pos_type':  'MPos' | 'WPos' | None,
            'wco':       [float, ...] | None,  # work coordinate offset (sent only now and then)
            'planner_free': int | None,   # free planner blocks from Bf
            'feed':      float | None,    # feed rate [units/min]
            'spindle':   float | None,    # spindle speed
            'pins':      str,             # triggered pins, '' when none
        }
    """
    match = _STATUS_RE.match(line.strip())
    if not match:
        return None
    fields = {}
    for part in match.group(2).split('|'):
        key, sep, value = part.partition(':')
        if sep:
            fields[key] = value

    result = {
        'state': match.group(1),
        'positions': None,
        'pos_type': None,
        'wco': _floats(fields.get('WCO')),
        'planner_free': None,
        'feed': None,
        'spindle': None,
        'pins': fields.get('Pn', ''),
    }

    for key in ('MPos', 'WPos'):
        if key in fields:
            result['positions'] = _floats(fields[key])
            result['pos_type'] = key if result['positions'] is not None else None
            break

    bf = fields.get('Bf')
    if bf is not None:
        try:
            result['planner_free'] = int(bf.split(',')[0])
        except ValueError:
            pass

    fs = fields.get('FS')
    if fs is not None:
        parts = fs.split(',')
        try:
            result['feed'] = float(parts[0])
            if len(parts) > 1:
                result['spindle'] = float(parts[1])
        except (ValueError, IndexError):
            pass
    else:
        f = fields.get('F')
        if f is not None:
            try:
                result['feed'] = float(f)
            except ValueError:
                pass

    return result


def compute_feed(start, target, dt_s, max_feed):
    """Feed rate [units/min] so a coordinated move of all axes takes dt_s seconds.

    FluidNC uses the vector length of all axes for the feed rate,
    so the result is clamped to (0, max_feed].
    """
    dist = math.sqrt(sum((t - s) ** 2 for s, t in zip(start, target)))
    if dt_s <= 0.0 or dist == 0.0:
        return max_feed
    return max(1.0, min(max_feed, dist / dt_s * 60.0))


def build_move(axis_letters, values, feed):
    """Build an absolute G1 move, e.g. 'G1 X10.000 Y-5.000 F600.0'."""
    words = ' '.join(f'{a}{v:.3f}' for a, v in zip(axis_letters, values))
    return f'G1 {words} F{feed:.1f}'


def build_jog(axis_letters, deltas, feed):
    """Build a relative jog, e.g. '$J=G91 X0.500 F600.0'. Zero deltas are left out.

    Returns None when every delta is zero.
    """
    words = ' '.join(f'{a}{d:.4f}' for a, d in zip(axis_letters, deltas) if round(d, 4) != 0.0)
    if not words:
        return None
    return f'$J=G91 {words} F{feed:.1f}'


def jog_segment(velocities, seconds, max_feed):
    """Turn joint velocities [units/s] into (deltas, feed [units/min]) for one jog segment.

    The feed is the vector speed over all axes (how FluidNC interprets F), clamped to max_feed;
    when clamped, the deltas are scaled down so the segment still lasts `seconds`.
    """
    deltas = [v * seconds for v in velocities]
    dist = math.sqrt(sum(d * d for d in deltas))
    if dist == 0.0 or seconds <= 0.0:
        return [0.0] * len(velocities), 0.0
    feed = dist / seconds * 60.0
    if feed > max_feed:
        deltas = [d * max_feed / feed for d in deltas]
        feed = max_feed
    return deltas, feed


def _floats(raw):
    if raw is None:
        return None
    try:
        return [float(v) for v in raw.split(',')]
    except ValueError:
        return None
