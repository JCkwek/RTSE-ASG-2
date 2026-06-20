"""Bottom-anchored sliding-window road trace (pure, dependency-free).

The road is reliable right in front of the car (bottom of frame) but ambiguous
far away (curves on turns, ends early uphill, runs long downhill). So instead of
assuming one global road shape, we ANCHOR at the bottom and follow the curbs
upward row by row, measuring the road where it actually is.

The output is a per-row profile ``[(y, center, width), ...]`` ordered bottom
(largest y) -> top (smallest y). Lane classification interpolates center/width
from this profile at each token's row, so it bends with turns and stops with
hill crests -- no fixed horizon, no straight-line assumption.

This module is pure (operates on plain lists of curb x-positions per row); the
OpenCV mask scanning that produces those lists lives in sample_drive.
"""

from lane_geometry import LANE_MIN, LANE_MAX, LANES_PER_ROAD


def find_anchor(curb_xs, frame_width, min_gap, car_cx=160.0):
    """The road gap the CAR sits in -> (left, right), or None if too narrow.

    The car is always on the road at frame center, so the road is the gap
    straddling car_cx -- not merely the widest gap, which on an off-center road
    is a side margin. Returns None if that gap is narrower than min_gap (e.g.
    near the horizon), so the trace stops instead of latching onto a margin.
    """
    xs = [0] + sorted(curb_xs) + [frame_width - 1]
    gaps = list(zip(xs, xs[1:]))
    road = None
    for a, b in gaps:
        if a <= car_cx <= b:
            road = (a, b)
            break
    if road is None:  # car landed on a curb pixel -> nearest gap by center
        road = min(gaps, key=lambda g: abs((g[0] + g[1]) / 2.0 - car_cx))
    if road[1] - road[0] < min_gap:
        return None
    return road


def trace_road(rows, scan_order, frame_width, min_gap, window, collapse_frac=0.4, car_cx=160.0):
    """Follow the curbs upward from the bottom anchor.

    rows:       {y: sorted iterable of curb x-positions in that row}.
    scan_order: list of y values, bottom (largest) -> top (smallest).
    window:     max px a curb may move per scanned row (keeps the trace on the
                same curb instead of jumping to a sign/blob far away).

    Returns the profile [(y, center, width), ...] bottom -> top.
    """
    anchor = None
    start = 0
    for i, y in enumerate(scan_order):
        a = find_anchor(rows.get(y, ()), frame_width, min_gap, car_cx)
        if a:
            anchor, start = a, i
            break
    if anchor is None:
        return []

    xl, xr = anchor
    profile = [(scan_order[start], (xl + xr) / 2.0, float(xr - xl))]
    for y in scan_order[start + 1:]:
        curbs = rows.get(y, ())
        nl = _nearest(curbs, xl, window)
        nr = _nearest(curbs, xr, window)
        if nl is None and nr is None:
            break  # lost both curbs -> road ended (crest / horizon)
        if nl is not None:
            xl = nl
        if nr is not None:
            xr = nr
        width = xr - xl
        if width < min_gap * collapse_frac:
            break  # gap collapsed -> near the vanishing point
        profile.append((y, (xl + xr) / 2.0, float(width)))
    return profile


def interp(profile, y):
    """(center, width) at full-frame row y, or None if outside the traced road."""
    if not profile:
        return None
    if y >= profile[0][0]:
        return profile[0][1], profile[0][2]      # at/below the car -> bottom anchor
    if y <= profile[-1][0]:
        return None                               # above the trace -> beyond reliable road
    for (y0, c0, w0), (y1, c1, w1) in zip(profile, profile[1:]):
        if y1 <= y <= y0:                          # y1 < y0 (going up)
            if y0 == y1:
                return c0, w0
            t = (y0 - y) / (y0 - y1)
            return c0 + t * (c1 - c0), w0 + t * (w1 - w0)
    return None


def occupied_lanes_from_profile(profile, cx, y, obj_w):
    """Sorted relative lane offsets for an object at (cx, full-frame y), or []."""
    cw = interp(profile, y)
    if cw is None:
        return []
    center, width = cw
    lane_width = width / LANES_PER_ROAD
    if lane_width <= 0:
        return []
    rel = int(round((cx - center) / lane_width))
    lanes = {_clamp(rel)}
    if obj_w > lane_width * 1.5:
        lanes.add(_clamp(rel - 1))
        lanes.add(_clamp(rel + 1))
    return sorted(lanes)


def car_lane_from_profile(profile, car_cx=160.0):
    """The car's own relative lane from the bottom anchor, or None.

    The car sits at frame center; its lane is its offset from the measured road
    center at the bottom (most reliable row). Used to correct dead-reckoning drift.
    """
    if not profile:
        return None
    _, center, width = profile[0]
    lane_width = width / LANES_PER_ROAD
    if lane_width <= 0:
        return None
    return _clamp(int(round((car_cx - center) / lane_width)))


def _nearest(xs, target, window):
    """Nearest x to target within +/-window, or None."""
    best = None
    best_d = window
    for x in xs:
        d = abs(x - target)
        if d <= best_d:
            best_d = d
            best = x
    return best


def _clamp(lane):
    return max(LANE_MIN, min(LANE_MAX, lane))
