"""Pure lane-geometry helpers (object -> occupied relative lanes).

Dependency-free so it can be unit-tested without OpenCV / NumPy.

A RoadModel describes the road in image space as two linear functions of the
full-frame image row ``y``::

    center(y) = cx_slope * y + cx_intercept      # lane-grid center (handles curve/offset)
    width(y)  = width_slope * (y - horizon_y)    # road width; 0 at the horizon row

``horizon_y`` is the vanishing point's row. Fitting it per frame from the
detected curbs lets the model FLOAT with the camera pitch on slopes, instead of
assuming a fixed horizon that drifts out of position (跑位) on inclines.

Model selection (caller chains these): a multi-row ``fit_road_model`` (slope-aware)
-> ``model_from_bounds`` (single measured row, real center) -> ``default_road_model``.
"""

from collections import namedtuple

from chaser_logic import LANE_MIN, LANE_MAX

RoadModel = namedtuple("RoadModel", ["cx_slope", "cx_intercept", "width_slope", "horizon_y"])

LANES_PER_ROAD = 5
HORIZON_Y = 80                 # fallback horizon row (flat-road assumption)
FALLBACK_LANE_SLOPE = 0.44     # lane width per unit depth, fixed model
FALLBACK_ROAD_CENTER = 160.0   # frame center, fixed model
CAR_CX = 160.0                 # the camera is car-centered: the car sits at frame x=160,
                               # so object lanes are measured RELATIVE TO THE CAR here

# A fit needs enough rows and vertical spread to extrapolate the horizon reliably.
MIN_FIT_SAMPLES = 3
MIN_FIT_Y_SPAN = 30


def default_road_model():
    """Flat-road fallback: center at frame center, fixed-slope perspective."""
    return RoadModel(0.0, FALLBACK_ROAD_CENTER, FALLBACK_LANE_SLOPE * LANES_PER_ROAD, HORIZON_Y)


def model_from_bounds(road_bounds, roi_start_y=100):
    """Degenerate model from a single measured curb row: real center, fixed horizon.

    Used when a full multi-row fit is not available but one row was measured --
    keeps the measured center (but cannot float the horizon).
    """
    if road_bounds is None:
        return None
    x_left, x_right, lane_w, scan_y = road_bounds
    center = (x_left + x_right) / 2.0
    y_scan = scan_y + roi_start_y
    if y_scan <= HORIZON_Y:
        return RoadModel(0.0, center, FALLBACK_LANE_SLOPE * LANES_PER_ROAD, HORIZON_Y)
    width = lane_w * LANES_PER_ROAD
    return RoadModel(0.0, center, width / (y_scan - HORIZON_Y), HORIZON_Y)


def fit_road_model(samples, min_samples=MIN_FIT_SAMPLES, min_span=MIN_FIT_Y_SPAN):
    """Least-squares RoadModel from ``[(y, center_x, road_width), ...]`` curb rows.

    Returns None when there are too few rows, too little vertical spread, or the
    fit is degenerate (non-increasing width / implausible horizon) -- the caller
    then falls back to a single-row or fixed model.
    """
    if len(samples) < min_samples:
        return None
    ys = [s[0] for s in samples]
    if max(ys) - min(ys) < min_span:
        return None

    cxs = [s[1] for s in samples]
    widths = [s[2] for s in samples]

    width_slope, width_intercept = _linfit(ys, widths)
    if width_slope is None or width_slope <= 0:
        return None  # width must grow toward the car; otherwise the fit is noise
    horizon_y = -width_intercept / width_slope
    # The horizon must sit ABOVE the nearest measured row (smaller y) and be sane.
    if not (-200 < horizon_y < min(ys)):
        return None

    cx_slope, cx_intercept = _linfit(ys, cxs)
    if cx_slope is None:
        cx_slope, cx_intercept = 0.0, sum(cxs) / len(cxs)
    return RoadModel(cx_slope, cx_intercept, width_slope, horizon_y)


def occupied_lanes(cx, center_y, obj_w, road_model=None, roi_start_y=100):
    """Sorted relative lane offsets an object occupies, per the RoadModel.

    cx:         object center x in the 320-wide frame.
    center_y:   object center y in ROI-local coords (row 0 == roi_start_y).
    obj_w:      object width (px); wide objects span three lanes.
    road_model: a RoadModel, or None to use the flat-road fallback.
    """
    model = road_model if road_model is not None else default_road_model()
    y = center_y + roi_start_y

    lane_width = (model.width_slope * (y - model.horizon_y)) / LANES_PER_ROAD
    if lane_width <= 0:
        return []  # at/above the horizon there is no road plane to classify on

    # Classify RELATIVE TO THE CAR (frame center), not the road center: the camera
    # is car-centered and the decision logic treats lane 0 as "ahead of the car".
    # Using the road center mis-placed objects when the car was off-center and drove
    # the bot into the police car. Width still comes from the measured road model.
    rel_lane = int(round((cx - CAR_CX) / lane_width))
    lanes = {_clamp(rel_lane)}
    if obj_w > lane_width * 1.5:
        lanes.add(_clamp(rel_lane - 1))
        lanes.add(_clamp(rel_lane + 1))
    return sorted(lanes)


def _linfit(xs, ys):
    """Ordinary least-squares (slope, intercept) for y = slope*x + intercept."""
    n = len(xs)
    if n < 2:
        return None, None
    sx, sy = sum(xs), sum(ys)
    sxx = sum(x * x for x in xs)
    sxy = sum(x * y for x, y in zip(xs, ys))
    denom = n * sxx - sx * sx
    if denom == 0:
        return None, None
    slope = (n * sxy - sx * sy) / denom
    intercept = (sy - slope * sx) / n
    return slope, intercept


def _clamp(lane):
    return max(LANE_MIN, min(LANE_MAX, lane))
