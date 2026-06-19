# Chaser Evasion Overhaul (EV3 / EV4) — Design

**Date:** 2026-06-19
**Owner:** Leow (Yan-Hong branch)
**Scope:** Only the chasing-car detection + dodge path in `sample_drive.py`. Police, Golden Lane (OCR), Low-Light, and green-seek logic are out of scope and must remain unchanged.

## Problem

Against the two chasing-car events the car currently:

- Reacts **too late** and **does not move far enough to escape**.
- Steering **direction is already correct** (no left/right inversion).
- The chaser **homes into the car's current lane**, so a single one-lane dodge is re-acquired and the car gets re-caught — especially near a track edge.

## Goal

Reliably avoid collision across both chaser events (EV3, EV4) by:

1. Detecting the chaser **earlier and more reliably**.
2. Escaping with a **continuous sweep/weave** that exploits the homing chaser's steering lag, instead of a single one-lane step.

Non-goals: lane-display accuracy (Kwek), net-green-60 token seeking (Kew), any non-chaser event.

## Current behavior (reference)

- `detect_back_environment(back_frame)` — masks a blue/teal range on the rear ROI `small_frame[130:240, 40:280]`, takes the **first** contour passing a trivial filter (`area > 0 or w >= 1 or h >= 1`) and `break`s. Returns a list with 0 or 1 box.
- `processing_task` — sets `chaser_behind = len(chaser_boxes) > 0`.
- `evaluate_decision` — builds `chaser_lanes` from box center using fixed pixels (`<130` left, `>190` right, else center). Evasion fires **only** when `0 in chaser_lanes`; picks the **smallest** safe move (`min(..., key=abs(l))`), biasing back toward center.
- `send_controls_task` — tap state machine; when `chaser_behind` it uses `ALPHA = 1.0` (no smoothing) and cooldown `2` frames; otherwise `COOLDOWN_FRAMES = 15`. `TAP_HOLD_FRAMES = 12`.

## Approach (Approach A: reactive weave + earlier trigger + hysteresis)

### 1. Rear perception — rework `detect_back_environment`

- Replace first-contour-`break` with **largest valid contour** selection.
- Add a real area floor `MIN_CHASER_AREA` (replaces `area > 0`).
- Compute a **proximity** scalar in `[0,1]` reusing the existing `scale = clamp((car_y - 120)/120, 0.1, 1.0)` as the "how close" proxy (lower in frame / larger = closer).
- Derive **side** (`-1` left / `0` center / `1` right) from the selected box center using the existing 130/190 thresholds.
- Return `(chaser_boxes, proximity, side)`. Empty/None-safe: returns `([], 0.0, 0)` when nothing valid.

### 2. Detection hysteresis — evade latch

- New `shared_data['chaser_evade_end_time']`.
- On any valid detection, set `chaser_evade_end_time = now + CHASER_EVADE_LATCH_SEC`.
- Decision and actuation read the **latched** `chaser_behind = now < chaser_evade_end_time`, so a one-frame mask dropout does not abort an in-progress escape.
- Also persist latest `chaser_proximity` and `chaser_side` in `shared_data` for the decision layer and overlay.

### 3. Decisive sweep/weave — chaser branch of `evaluate_decision`

- Trigger evasion whenever the **latched** chaser is behind (not only when dead-center), since it homes into the car's lane.
- Maintain a persistent **`evade_dir`** (module-level, like the tap state): keep moving in that direction across lanes; when the next lane in `evade_dir` is unsafe (police/danger) or an edge (`net_lane_position` would exceed `[-2, 2]`), **flip `evade_dir`**. This produces an edge-to-edge sweep that a lagging homing chaser cannot track.
- Safety filter unchanged in spirit: candidate target lane must not be a police lane, a danger lane, or off-track. If both sweep direction and its flip are blocked, output neutral steer with floored accel ("trapped → floor it straight").
- Escalation: when `proximity >= CHASER_CLOSE_PROXIMITY`, never emit neutral steer if any safe lane exists (force a move); below that, normal sweep cadence.
- Accel stays floored (`1.0`) throughout chaser evasion (existing behavior).

### 4. Actuation latency — `send_controls_task`

- `is_emergency` continues to gate fast actuation, now driven by the **latched** chaser state.
- Introduce `EMERGENCY_TAP_HOLD_FRAMES` (shorter than `TAP_HOLD_FRAMES`) and `EMERGENCY_COOLDOWN_FRAMES` (= current `2`) so consecutive lane changes **chain into a weave** rather than completing one slow tap.

### 5. Tunable constants (top of file)

- `MIN_CHASER_AREA` — rear-mask noise floor.
- `CHASER_EVADE_LATCH_SEC` — hysteresis window (~0.6 s).
- `CHASER_CLOSE_PROXIMITY` — escalation threshold (~0.6).
- `EMERGENCY_TAP_HOLD_FRAMES`, `EMERGENCY_COOLDOWN_FRAMES` — emergency tap cadence.

### 6. Debug overlay

- On the back-camera window, draw a readout: chaser proximity, latched evade state, and current `evade_dir`, so detection range and weave can be observed and the constants tuned during Windows test runs.

## Data flow

```
back cam -> read_back_camera_task -> shared_data['latest_back_frame']
  -> processing_task:
       (boxes, proximity, side) = detect_back_environment(...)
       update chaser_evade_end_time / chaser_proximity / chaser_side
       chaser_behind (latched) -> evaluate_decision (sweep evade)
  -> send_controls_task: latched chaser_behind -> fast emergency taps
```

## Error handling

- `detect_back_environment` stays exception-safe and returns the empty triple on no detection; no exceptions propagate into the RT loop.
- All lane indices clamped to `[-2, 2]`.
- Latch read with `.get(..., default)` so missing keys never raise.

## Testing / validation

- Code-only authoring (the game is a Windows `.exe`; cannot be launched on the author's macOS).
- The diff is confined to: `detect_back_environment`, the chaser branch of `evaluate_decision`, the latch block in `processing_task`, emergency timing in `send_controls_task`, the new constants, and the back-camera overlay. No other event path is touched.
- User runs `SpeedTrials2D.exe` + `python sample_drive.py` on Windows, triggers **both** chaser events, and reports whether the homing car is shaken. Constants are tuned iteratively from the overlay readout.

## Success criteria

- Chaser is detected earlier and tracking does not lock onto rear-mask noise.
- The car sustains an edge-to-edge weave while a chaser is latched behind it, and survives both chaser events without collision in repeated runs.
- Police / Golden Lane / Low-Light / green-seek behavior is unchanged.
