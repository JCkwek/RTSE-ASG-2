"""Pure lane-geometry helpers (object -> occupied relative lanes).

Dependency-free so it can be unit-tested without OpenCV / NumPy.

Classifies a detected object into relative lane offsets using the MEASURED road
bounds when available -- so decisions use the same road the on-screen grid draws,
instead of a hardcoded center that drifts out of agreement with the real road and
collapses off-center lanes onto the center lane. Falls back to a fixed perspective
model only when the curbs were not detected this frame.
"""

from chaser_logic import LANE_MIN, LANE_MAX

HORIZON_Y = 80               # full-frame image row the road converges to
FALLBACK_LANE_SLOPE = 0.44   # lane width per unit depth when no curbs measured
FALLBACK_ROAD_CENTER = 160.0  # frame center -- used only without measured bounds


def occupied_lanes(cx, center_y, obj_w, road_bounds=None, roi_start_y=100):
    """Return the sorted relative lane offsets an object occupies.

    cx:          object center x in the 320-wide frame.
    center_y:    object center y in ROI-local coords (row 0 == roi_start_y).
    obj_w:       object width (px); wide objects span three lanes.
    road_bounds: (x_left, x_right, lane_w, scan_y) measured by curb detection,
                 or None. When present, the REAL road center and a
                 perspective-scaled REAL lane width are used.
    roi_start_y: row offset of the ROI inside the full frame.
    """
    actual_y = center_y + roi_start_y
    dist_to_horizon = actual_y - HORIZON_Y
    if dist_to_horizon <= 0:
        return []

    center, lane_width = _lane_model(dist_to_horizon, road_bounds, roi_start_y)
    if lane_width <= 0:
        return []

    rel_lane = int(round((cx - center) / lane_width))
    lanes = {_clamp(rel_lane)}
    if obj_w > lane_width * 1.5:
        # A blob wide enough to straddle neighbours occupies them too.
        lanes.add(_clamp(rel_lane - 1))
        lanes.add(_clamp(rel_lane + 1))
    return sorted(lanes)


def _lane_model(dist_to_horizon, road_bounds, roi_start_y):
    """(road_center_x, lane_width_px) at the object's depth."""
    if road_bounds is None:
        return FALLBACK_ROAD_CENTER, dist_to_horizon * FALLBACK_LANE_SLOPE

    x_left, x_right, lane_w_scan, scan_y = road_bounds
    center = (x_left + x_right) / 2.0
    dist_scan = (scan_y + roi_start_y) - HORIZON_Y
    if dist_scan <= 0:
        return center, dist_to_horizon * FALLBACK_LANE_SLOPE
    # Perspective-scale the measured lane width from the scan row to this depth:
    # the road (and each lane) narrows linearly toward the horizon.
    return center, lane_w_scan * (dist_to_horizon / dist_scan)


def _clamp(lane):
    return max(LANE_MIN, min(LANE_MAX, lane))
