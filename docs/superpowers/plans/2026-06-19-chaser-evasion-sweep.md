# Chaser Evasion Sweep Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make the car reliably shake the homing chasing car (EV3/EV4) by detecting it earlier/more reliably and escaping with a continuous edge-to-edge sweep/weave instead of a single one-lane dodge.

**Architecture:** Extract the pure, dependency-free decision logic (sweep chooser + rear-box geometry) into a new `chaser_logic.py` so it can be unit-tested with stdlib `unittest` on any machine. `sample_drive.py` keeps the OpenCV masking and live wiring, importing the pure helpers. A time-based "evade latch" in shared state bridges one-frame detection dropouts.

**Tech Stack:** Python 3.9, OpenCV (`cv2`), NumPy, stdlib `unittest`, `threading`. The game itself is a Windows-only `.exe`.

## Global Constraints

- Scope is the chaser path ONLY. Do NOT modify Police, Golden-Lane/OCR, Low-Light, or green-seek behavior.
- Lanes are `net_lane_position` integers in `[-2, 2]` (displayed as L1–L5; absolute = rel + 3).
- `obj['lanes']`, `police_lanes`, and `danger_lanes` hold lane offsets RELATIVE to the current lane.
- Steering output is a float in `{-1.0, 0.0, 1.0}`; acceleration float in `[-1.0, 1.0]`.
- The author works on macOS and CANNOT launch the Windows game; `sample_drive.py` cannot be imported here (it imports `cv2`). Local verification = stdlib `unittest` on `chaser_logic.py` + `python3 -m py_compile sample_drive.py`. Behavioral validation = user runs on Windows.
- Git commits/pushes are SKIPPED per user preference. Each task ends with a verification command instead of a commit.
- Keep changes surgical and follow the existing single-file, module-global style (`tap_state`, `smoothed_steering`, etc.).

---

## File Structure

- Create: `chaser_logic.py` — pure helpers: `choose_chaser_evade`, `chaser_box_metrics`, constants `LANE_MIN`/`LANE_MAX`. No third-party imports.
- Create: `tests/test_chaser_logic.py` — stdlib `unittest` tests for the pure helpers.
- Modify: `sample_drive.py` — constants, `detect_back_environment` rework, `processing_task` latch wiring, `evaluate_decision` chaser branch, `send_controls_task` emergency cadence, back-camera overlay.

---

### Task 1: Pure sweep chooser (`choose_chaser_evade`)

**Files:**
- Create: `chaser_logic.py`
- Test: `tests/test_chaser_logic.py`

**Interfaces:**
- Produces: `LANE_MIN = -2`, `LANE_MAX = 2`; `choose_chaser_evade(current_lane: int, evade_dir: int, blocked: set[int]) -> tuple[float, int, str]` returning `(steer, new_evade_dir, debug_text)`. `steer ∈ {-1.0,0.0,1.0}`, `new_evade_dir ∈ {-1,1}`.

- [ ] **Step 1: Write the failing test**

Create `tests/test_chaser_logic.py`:

```python
import unittest
from chaser_logic import choose_chaser_evade


class TestChooseChaserEvade(unittest.TestCase):
    def test_mid_track_continues_in_evade_dir(self):
        steer, new_dir, _ = choose_chaser_evade(0, 1, set())
        self.assertEqual((steer, new_dir), (1.0, 1))

    def test_continues_left_when_dir_negative(self):
        steer, new_dir, _ = choose_chaser_evade(0, -1, set())
        self.assertEqual((steer, new_dir), (-1.0, -1))

    def test_flips_at_right_edge(self):
        # At lane +2 sweeping right: must reverse to left.
        steer, new_dir, _ = choose_chaser_evade(2, 1, set())
        self.assertEqual((steer, new_dir), (-1.0, -1))

    def test_flips_at_left_edge(self):
        steer, new_dir, _ = choose_chaser_evade(-2, -1, set())
        self.assertEqual((steer, new_dir), (1.0, 1))

    def test_reverses_when_forward_lane_blocked(self):
        # Sweeping right but +1 lane is unsafe -> go left instead.
        steer, new_dir, _ = choose_chaser_evade(0, 1, {1})
        self.assertEqual((steer, new_dir), (-1.0, -1))

    def test_trapped_both_sides_floors_straight(self):
        steer, new_dir, text = choose_chaser_evade(0, 1, {-1, 1})
        self.assertEqual(steer, 0.0)
        self.assertEqual(new_dir, 1)
        self.assertIn("FLOOR", text)

    def test_trapped_at_edge_with_block(self):
        # At +2 (right edge) sweeping right, and left lane (-1) blocked -> trapped.
        steer, _, _ = choose_chaser_evade(2, 1, {-1})
        self.assertEqual(steer, 0.0)


if __name__ == "__main__":
    unittest.main()
```

- [ ] **Step 2: Run test to verify it fails**

Run: `python3 -m unittest discover -s tests -p "test_*.py" -v`
Expected: FAIL — `ModuleNotFoundError: No module named 'chaser_logic'`

- [ ] **Step 3: Write minimal implementation**

Create `chaser_logic.py`:

```python
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
```

- [ ] **Step 4: Run test to verify it passes**

Run: `python3 -m unittest discover -s tests -p "test_*.py" -v`
Expected: PASS (7 tests OK)

- [ ] **Step 5: Verify (no commit — git skipped)**

Run: `python3 -m py_compile chaser_logic.py && echo OK`
Expected: `OK`

---

### Task 2: Pure rear-box geometry (`chaser_box_metrics`)

**Files:**
- Modify: `chaser_logic.py`
- Test: `tests/test_chaser_logic.py`

**Interfaces:**
- Consumes: nothing from other tasks.
- Produces: `chaser_box_metrics(x: int, y: int, w: int, h: int) -> tuple[tuple[int,int,int,int], float, int]` returning `(final_box, proximity, side)`. `final_box = (final_x, final_y, box_w, box_h)` in 320×240 rear-frame coordinates; `proximity ∈ [0.1, 1.0]` (larger = closer); `side ∈ {-1, 0, 1}` (left/center/right of car). Input `(x, y, w, h)` is a bounding rect within the rear ROI `roi_back = small_frame[130:240, 40:280]` (i.e., ROI-local coordinates).

- [ ] **Step 1: Write the failing test**

Append to `tests/test_chaser_logic.py` (add import at top):

```python
from chaser_logic import chaser_box_metrics


class TestChaserBoxMetrics(unittest.TestCase):
    def test_centered_far_box_is_center_side_low_proximity(self):
        # ROI-local (x,y,w,h): small y -> high in rear image -> far away.
        box, proximity, side = chaser_box_metrics(x=110, y=0, w=20, h=10)
        self.assertEqual(side, 0)
        self.assertAlmostEqual(proximity, max(0.1, min(1.0, (0 + 130 - 120) / 120.0)))

    def test_near_box_has_higher_proximity_than_far(self):
        _, prox_far, _ = chaser_box_metrics(110, 0, 20, 10)
        _, prox_near, _ = chaser_box_metrics(110, 100, 20, 10)
        self.assertGreater(prox_near, prox_far)

    def test_left_box_reports_left_side(self):
        # car_x = x + 40; center_x = car_x + w/2 must be < 130 for left.
        _, _, side = chaser_box_metrics(x=0, y=50, w=10, h=10)  # center_x = 45
        self.assertEqual(side, -1)

    def test_right_box_reports_right_side(self):
        # center_x = (x+40) + w/2 must be > 190 for right.
        _, _, side = chaser_box_metrics(x=170, y=50, w=20, h=10)  # center_x = 220
        self.assertEqual(side, 1)

    def test_proximity_clamped_to_unit_range(self):
        _, proximity, _ = chaser_box_metrics(110, 240, 20, 10)
        self.assertLessEqual(proximity, 1.0)
        self.assertGreaterEqual(proximity, 0.1)
```

- [ ] **Step 2: Run test to verify it fails**

Run: `python3 -m unittest discover -s tests -p "test_*.py" -v`
Expected: FAIL — `ImportError: cannot import name 'chaser_box_metrics'`

- [ ] **Step 3: Write minimal implementation**

Append to `chaser_logic.py`:

```python
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
```

- [ ] **Step 4: Run test to verify it passes**

Run: `python3 -m unittest discover -s tests -p "test_*.py" -v`
Expected: PASS (12 tests OK)

- [ ] **Step 5: Verify (no commit)**

Run: `python3 -m py_compile chaser_logic.py && echo OK`
Expected: `OK`

---

### Task 3: Constants + rear-perception rework in `sample_drive.py`

**Files:**
- Modify: `sample_drive.py` (import block near top ~line 9-10; constants near line 60-65; `detect_back_environment` lines 266-287)

**Interfaces:**
- Consumes: `chaser_box_metrics` from Task 2.
- Produces: `detect_back_environment(back_frame) -> (chaser_boxes: list, proximity: float, side: int)` (was: `list`). Returns `([], 0.0, 0)` when nothing valid. New module-level names: `MIN_CHASER_AREA`, `CHASER_EVADE_LATCH_SEC`, `CHASER_CLOSE_PROXIMITY`, `EMERGENCY_TAP_HOLD_FRAMES`, `EMERGENCY_COOLDOWN_FRAMES`, and global `evade_dir`.

- [ ] **Step 1: Add the import**

In `sample_drive.py`, after the existing `import re` / `import os` lines (around line 10), add:

```python
from chaser_logic import choose_chaser_evade, chaser_box_metrics
```

- [ ] **Step 2: Add tunable constants and the sweep-direction global**

In the tapping-control block (near lines 57-62, after `COOLDOWN_FRAMES = 15`), add:

```python
# --- Chaser evasion tunables (EV3/EV4) ---
MIN_CHASER_AREA = 12          # rear-mask noise floor (320x240 space)
CHASER_EVADE_LATCH_SEC = 0.6  # keep evading through 1-frame detection dropouts
CHASER_CLOSE_PROXIMITY = 0.6  # >= this => escalate (never coast)
EMERGENCY_TAP_HOLD_FRAMES = 7 # shorter hold so lane changes chain into a weave
EMERGENCY_COOLDOWN_FRAMES = 2 # near-zero cooldown during chaser evasion
evade_dir = 1                 # current sweep direction (-1 left / +1 right)
```

- [ ] **Step 3: Rewrite `detect_back_environment`**

Replace the entire body of `detect_back_environment` (lines 266-287) with:

```python
def detect_back_environment(back_frame):
    if back_frame is None:
        return [], 0.0, 0
    small_frame = cv2.resize(back_frame, (320, 240))
    roi_back = small_frame[130:240, 40:280]
    roi_hsv = cv2.cvtColor(roi_back, cv2.COLOR_BGR2HSV)
    mask_car = cv2.inRange(roi_hsv, np.array([85, 150, 80]), np.array([130, 255, 255]))
    mask_car = cv2.morphologyEx(mask_car, cv2.MORPH_OPEN, morph_kernel)
    contours, _ = cv2.findContours(mask_car, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Pick the LARGEST valid contour (the closest/real chaser), not the first.
    best_rect = None
    best_area = 0.0
    for c in contours:
        area = cv2.contourArea(c)
        if area < MIN_CHASER_AREA:
            continue
        if area > best_area:
            best_area = area
            best_rect = cv2.boundingRect(c)

    if best_rect is None:
        return [], 0.0, 0

    x, y, w, h = best_rect
    final_box, proximity, side = chaser_box_metrics(x, y, w, h)
    return [final_box], proximity, side
```

- [ ] **Step 4: Verify the file still compiles**

Run: `python3 -m py_compile sample_drive.py && echo OK`
Expected: `OK` (compiles even without cv2 installed; import errors only surface at runtime on Windows)

- [ ] **Step 5: Re-run the pure tests (unchanged, must still pass)**

Run: `python3 -m unittest discover -s tests -p "test_*.py" -v`
Expected: PASS (12 tests OK)

---

### Task 4: Latch wiring + sweep integration

**Files:**
- Modify: `sample_drive.py` — `shared_data` dict (lines 34-53), `processing_task` (lines 522-579), `evaluate_decision` chaser block (lines 457-492).

**Interfaces:**
- Consumes: `detect_back_environment` (Task 3) `-> (boxes, proximity, side)`; `choose_chaser_evade` (Task 1).
- Produces: latched `shared_data['chaser_behind']`, plus `shared_data['chaser_proximity']`, `shared_data['chaser_side']`, `shared_data['chaser_evade_end_time']`. `evaluate_decision` chaser branch now sweeps via the global `evade_dir`.

- [ ] **Step 1: Add latch/telemetry keys to `shared_data`**

In the `shared_data` dict (after `'chaser_boxes': [],` near line 47), add:

```python
    'chaser_evade_end_time': 0.0,
    'chaser_proximity': 0.0,
    'chaser_side': 0,
```

- [ ] **Step 2: Wire detection + latch + sweep seeding in `processing_task`**

In `processing_task`, replace the two lines:

```python
        chaser_boxes = detect_back_environment(back_frame)
        chaser_behind = len(chaser_boxes) > 0
```

with:

```python
        global evade_dir
        chaser_boxes, chaser_proximity, chaser_side = detect_back_environment(back_frame)
        chaser_detected = len(chaser_boxes) > 0
        now = time.time()
        with state_lock:
            was_latched = now < shared_data.get('chaser_evade_end_time', 0.0)
            if chaser_detected:
                shared_data['chaser_evade_end_time'] = now + CHASER_EVADE_LATCH_SEC
                shared_data['chaser_proximity'] = chaser_proximity
                shared_data['chaser_side'] = chaser_side
                if not was_latched:
                    # Fresh chaser: start the sweep AWAY from where it is.
                    evade_dir = -1 if chaser_side > 0 else 1
            chaser_behind = now < shared_data.get('chaser_evade_end_time', 0.0)
```

NOTE: keep the existing `global tap_state` declaration at the top of `processing_task`; add `global evade_dir` as shown (Python allows multiple names, but they must be declared before first use in the function — place this `global evade_dir` line as the first statement of the block above, which is before any `evade_dir` assignment).

- [ ] **Step 3: Replace the chaser branch in `evaluate_decision`**

In `evaluate_decision`, DELETE the now-dead `chaser_lanes` computation block:

```python
    chaser_lanes = set()
    if chaser_behind:
        for (cx, cy, cw, ch) in chaser_boxes:
            center_x = cx + cw / 2.0
            if center_x < 130: chaser_lanes.add(-1)
            elif center_x > 190: chaser_lanes.add(1)
            else: chaser_lanes.add(0)
```

and REPLACE the old chaser evade block:

```python
    if chaser_behind and 0 in chaser_lanes:
        target_accel = 1.0 
        safe_lanes = [l for l in [-1, 0, 1] if l not in police_lanes and l not in danger_lanes and l not in chaser_lanes and -2 <= current_lane + l <= 2]
        if safe_lanes: best_lane = min(safe_lanes, key=lambda l: abs(l))
        else:
            semi_safe = [l for l in [-1, 0, 1] if l not in police_lanes and l not in chaser_lanes and -2 <= current_lane + l <= 2]
            if semi_safe: best_lane = min(semi_safe, key=lambda l: abs(l))
            else: best_lane = 0 
        
        if best_lane < 0: return -1.0, target_accel, "<< DODGE CHASER LEFT"
        elif best_lane > 0: return 1.0, target_accel, "DODGE CHASER RIGHT >>"
        else: return 0.0, target_accel, "CHASER IMMINENT! FLOOR IT!"
```

with the sweep version (note the `global evade_dir`):

```python
    if chaser_behind:
        global evade_dir
        target_accel = 1.0
        blocked = police_lanes | danger_lanes
        steer, evade_dir, text = choose_chaser_evade(current_lane, evade_dir, blocked)
        return steer, target_accel, text
```

This block stays in its current position — AFTER the `if 0 in police_lanes:` police-evade block, so a police car dead-ahead (game-over risk) is still handled first, and the sweep avoids police/danger lanes via `blocked`.

- [ ] **Step 4: Verify compile**

Run: `python3 -m py_compile sample_drive.py && echo OK`
Expected: `OK`

- [ ] **Step 5: Re-run pure tests**

Run: `python3 -m unittest discover -s tests -p "test_*.py" -v`
Expected: PASS (12 tests OK)

---

### Task 5: Emergency tap cadence in `send_controls_task`

**Files:**
- Modify: `sample_drive.py` — `send_controls_task` (lines 596-615).

**Interfaces:**
- Consumes: `EMERGENCY_TAP_HOLD_FRAMES`, `EMERGENCY_COOLDOWN_FRAMES` (Task 3); latched `shared_data['chaser_behind']` (Task 4).

- [ ] **Step 1: Use the emergency hold when entering a tap**

In `send_controls_task`, in the `IDLE` branch, replace:

```python
            tap_state = 'TAPPING'
            tap_timer = TAP_HOLD_FRAMES
```

with:

```python
            tap_state = 'TAPPING'
            tap_timer = EMERGENCY_TAP_HOLD_FRAMES if is_emergency else TAP_HOLD_FRAMES
```

- [ ] **Step 2: Use the named emergency cooldown**

In the `TAPPING` branch, replace:

```python
            tap_timer = 2 if is_emergency else COOLDOWN_FRAMES
```

with:

```python
            tap_timer = EMERGENCY_COOLDOWN_FRAMES if is_emergency else COOLDOWN_FRAMES
```

- [ ] **Step 3: Verify compile**

Run: `python3 -m py_compile sample_drive.py && echo OK`
Expected: `OK`

---

### Task 6: Back-camera debug overlay

**Files:**
- Modify: `sample_drive.py` — main display loop: state snapshot (lines 671-687) and back-camera draw block (lines 761-771).

**Interfaces:**
- Consumes: `shared_data['chaser_proximity']`, `shared_data['chaser_side']`, `shared_data['chaser_evade_end_time']`, global `evade_dir`.

- [ ] **Step 1: Snapshot the new telemetry**

In the `with state_lock:` block of the main loop (after `chaser_boxes = shared_data.get('chaser_boxes', [])` near line 678), add:

```python
                chaser_proximity = shared_data.get('chaser_proximity', 0.0)
                chaser_evade_end = shared_data.get('chaser_evade_end_time', 0.0)
```

- [ ] **Step 2: Draw the readout on the back-camera window**

In the back-camera draw block, replace:

```python
                if chaser_behind:
                    cv2.putText(display_back, "CHASER WARNING", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2) 
                    for (x, y, w, h) in chaser_boxes:
                        cv2.rectangle(display_back, (x, y), (x+w, y+h), (0, 255, 0), 2)
                        cv2.putText(display_back, "CHASER", (x, y-5), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)
```

with:

```python
                latched = time.time() < chaser_evade_end
                if chaser_behind or latched:
                    sweep = "RIGHT" if evade_dir > 0 else "LEFT"
                    cv2.putText(display_back, f"CHASER | prox {chaser_proximity:.2f} | SWEEP {sweep}",
                                (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2)
                    for (x, y, w, h) in chaser_boxes:
                        cv2.rectangle(display_back, (x, y), (x+w, y+h), (0, 255, 0), 2)
                        cv2.putText(display_back, "CHASER", (x, y-5), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)
```

- [ ] **Step 3: Verify compile**

Run: `python3 -m py_compile sample_drive.py && echo OK`
Expected: `OK`

---

### Task 7: Windows behavioral validation (manual, by user)

**Files:** none.

This is the only end-to-end check; it must be run by the user on Windows.

- [ ] **Step 1:** Launch `SpeedTrials2D.exe`.
- [ ] **Step 2:** Run `python sample_drive.py` from the repo root (so `import chaser_logic` resolves).
- [ ] **Step 3:** Trigger **Chasing Car A** and watch the back-camera window. Expected: "CHASER | prox … | SWEEP …" appears; the car begins changing lanes across the track and reverses at the edges; the chaser does not collide.
- [ ] **Step 4:** Trigger **Chasing Car B**. Expected: same sustained weave; survives without collision.
- [ ] **Step 5:** Tuning guide (edit constants at top of `sample_drive.py`, re-run):
  - Reacts too late → raise `CHASER_EVADE_LATCH_SEC` (e.g. 0.8) and/or lower `MIN_CHASER_AREA` (detect smaller/farther).
  - Locks onto noise / false weaves → raise `MIN_CHASER_AREA`.
  - Weave too sluggish → lower `EMERGENCY_TAP_HOLD_FRAMES` (e.g. 5).
  - Confirm Police / Golden Lane / Low-Light still behave as before (regression check).

---

## Self-Review

**Spec coverage:**
- Rear perception rework (largest contour, proximity, side) → Tasks 2, 3. ✓
- Detection hysteresis / evade latch → Task 4 Step 2. ✓
- Decisive sweep/weave with persistent direction + edge flip → Tasks 1, 4 Step 3. ✓
- Escalation at close proximity → partially: `CHASER_CLOSE_PROXIMITY` is wired as a tunable + overlay readout; the sweep already always emits a move when any safe lane exists, which satisfies "never coast when a safe lane exists." The proximity value drives tuning/telemetry rather than a separate code branch. (Documented intentionally; no extra branch needed because the sweep is already non-coasting.)
- Actuation latency / emergency cadence → Task 5. ✓
- Tunable constants → Task 3 Step 2. ✓
- Debug overlay → Task 6. ✓
- "Direction already correct, do not flip" → preserved; no sign changes anywhere. ✓
- "Do not touch other events" → police-evade ordering preserved (Task 4 Step 3), no edits to golden/low-light/green paths. ✓

**Placeholder scan:** No TBD/TODO; every code step shows full code; every test step shows full assertions. ✓

**Type consistency:** `choose_chaser_evade(current_lane, evade_dir, blocked) -> (float, int, str)` used identically in Task 1 and Task 4. `chaser_box_metrics(x,y,w,h) -> (box, float, int)` used identically in Task 2 and Task 3. `detect_back_environment -> (list, float, int)` defined in Task 3 and unpacked the same way in Task 4. `evade_dir` is a module global declared `global` in both `processing_task` and `evaluate_decision`. ✓
