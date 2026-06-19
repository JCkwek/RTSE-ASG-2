"""Pure, dependency-free decision helpers for chaser evasion.

Kept import-free so it can be unit-tested without OpenCV/NumPy or the
Windows game runtime.
"""

LANE_MIN = -2
LANE_MAX = 2


def choose_chaser_evade(current_lane, evade_dir, blocked):
    """Continuous sweep/weave chooser.

    current_lane: int in [LANE_MIN, LANE_MAX] (net_lane_position).
    evade_dir:    int, -1 or +1, the current sweep direction.
    blocked:      set of RELATIVE lane offsets that are unsafe (police/danger).

    Returns (steer, new_evade_dir, debug_text):
      steer:         float in {-1.0, 0.0, 1.0}
      new_evade_dir: int -1/+1 (reversed if we had to turn around)
      debug_text:    str
    """
    for d in (evade_dir, -evade_dir):
        target = current_lane + d
        if target < LANE_MIN or target > LANE_MAX:
            continue  # would run off the track on this side
        if d in blocked:
            continue  # adjacent lane this way is unsafe
        text = "<< SWEEP CHASER LEFT" if d < 0 else "SWEEP CHASER RIGHT >>"
        return float(d), d, text
    return 0.0, evade_dir, "CHASER TRAPPED! FLOOR IT"


# Side thresholds in the 320-wide rear frame (car is centered ~160).
_SIDE_LEFT_X = 130
_SIDE_RIGHT_X = 190


def chaser_box_metrics(x, y, w, h):
    """Convert a rear-ROI bounding rect into (final_box, proximity, side).

    Mirrors the original draw-box math: ROI is offset by (+40, +130) from the
    320x240 frame; box size scales with how low (close) the car sits.
    """
    car_x = x + 40
    car_y = y + 130
    scale = max(0.1, min(1.0, (car_y - 120) / 120.0))
    box_w = int(140 * scale)
    box_h = int(60 * scale)
    center_x = car_x + w / 2.0
    center_y = car_y + h / 2.0
    final_x = int(center_x - box_w / 2)
    final_y = int(center_y - box_h / 2)
    proximity = scale
    if center_x < _SIDE_LEFT_X:
        side = -1
    elif center_x > _SIDE_RIGHT_X:
        side = 1
    else:
        side = 0
    return (final_x, final_y, box_w, box_h), proximity, side
