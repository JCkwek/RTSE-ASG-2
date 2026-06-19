"""Pure, dependency-free lateral-decision helpers (chaser evasion + green harvest).

Kept import-free so it can be unit-tested without OpenCV/NumPy or the
Windows game runtime.
"""

LANE_MIN = -2
LANE_MAX = 2


def choose_chaser_evade(current_lane, evade_dir, hard_blocked, soft_blocked=None):
    """Continuous sweep/weave chooser with two-tier lane blocking.

    current_lane: int in [LANE_MIN, LANE_MAX] (net_lane_position).
    evade_dir:    int, -1 or +1, the current sweep direction.
    hard_blocked: RELATIVE offsets that must NEVER be entered (police -> game over).
    soft_blocked: RELATIVE offsets to avoid UNLESS no clean lane exists
                  (danger/red tokens -- a clip is better than a chaser hit).

    Returns (steer, new_evade_dir, debug_text):
      steer:         float in {-1.0, 0.0, 1.0}
      new_evade_dir: int -1/+1 (reversed if we had to turn around)
      debug_text:    str

    Pass 1 prefers a fully clear lane; pass 2 accepts a soft-blocked lane as a
    last resort. A hard-blocked lane is never entered.
    """
    if soft_blocked is None:
        soft_blocked = set()
    for avoid, suffix in ((hard_blocked | soft_blocked, ""), (hard_blocked, " (RISK)")):
        for d in (evade_dir, -evade_dir):
            target = current_lane + d
            if target < LANE_MIN or target > LANE_MAX:
                continue  # would run off the track on this side
            if d in avoid:
                continue
            base = "<< SWEEP CHASER LEFT" if d < 0 else "SWEEP CHASER RIGHT >>"
            return float(d), d, base + suffix
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


def choose_green_seek(current_lane, green_lanes, blocked):
    """Pick a lateral move to harvest green across the 5-lane track.

    current_lane: int in [LANE_MIN, LANE_MAX].
    green_lanes:  set of RELATIVE offsets where green tokens were seen.
    blocked:      RELATIVE offsets unsafe to enter (danger | police).

    Returns (steer, label):
      steer: 0.0 hold / -1.0 left / 1.0 right, or None if no good green move.
      label: debug string ("" when steer is None).

    Holds when green is straight ahead; otherwise moves one lane toward the
    NEAREST reachable green side (current code only considered +-1).
    """
    if not green_lanes:
        return None, ""
    if 0 in green_lanes and 0 not in blocked:
        return 0.0, "SEEK GREEN AHEAD"

    left = [abs(l) for l in green_lanes if l < 0]
    right = [l for l in green_lanes if l > 0]
    can_left = (-1 not in blocked) and (current_lane > LANE_MIN)
    can_right = (1 not in blocked) and (current_lane < LANE_MAX)

    options = []  # (distance_to_nearest_green, steer, label)
    if left and can_left:
        options.append((min(left), -1.0, "<< SEEK GREEN LEFT"))
    if right and can_right:
        options.append((min(right), 1.0, "SEEK GREEN RIGHT >>"))
    if not options:
        return None, ""
    options.sort(key=lambda o: o[0])
    _, steer, label = options[0]
    return steer, label
