"""Pure decision cascade for the SpeedTrials2D driver.

Extracted from sample_drive.py so the priority cascade can be unit-tested
without OpenCV / NumPy / the Windows runtime (same rationale as chaser_logic).

`evaluate_decision` returns (steer, accel, label, new_evade_dir); the caller
owns the evade_dir state and all shared_data plumbing.
"""

from chaser_logic import choose_chaser_evade, choose_green_seek, LANE_MIN, LANE_MAX

# Defaults mirror the tuned constants in sample_drive.py. They are accepted as
# parameters so the live tunables stay the single source of truth there, while
# tests can call this function with plain literals.
DEFAULT_CHASER_CLOSE_PROXIMITY = 0.6
DEFAULT_MAX_RED_TOKEN_AREA = 130
DEFAULT_GOLDEN_COMMIT_SEC = 2.5


def evaluate_decision(detected_objects, current_lane, low_light_mode, chaser_behind,
                      seek_red_mode, golden_time_left, golden_target, evade_dir,
                      chaser_proximity=0.0,
                      chaser_close_proximity=DEFAULT_CHASER_CLOSE_PROXIMITY,
                      max_red_token_area=DEFAULT_MAX_RED_TOKEN_AREA,
                      golden_commit_sec=DEFAULT_GOLDEN_COMMIT_SEC):
    """Pick (steer, accel, label, new_evade_dir) from the perception state.

    Steering follows a fixed P0>P1>P2 priority cascade; acceleration is a
    separate longitudinal channel (see below). `evade_dir` is threaded in/out
    so the chaser sweep's edge-reversal state lives with the caller.
    """
    new_evade_dir = evade_dir

    police_lanes, danger_lanes, red_lanes, green_lanes = set(), set(), set(), set()
    for obj in detected_objects:
        for lane in obj['lanes']:
            if current_lane + lane < LANE_MIN or current_lane + lane > LANE_MAX:
                continue
            if obj['type'] == 'DANGER':
                if obj.get('subtype') == 'POLICE':
                    police_lanes.add(lane)
                elif obj.get('subtype') == 'RED':
                    # A collectible red is small; a large red blob is the police
                    # CAR body -> never seek it, always avoid.
                    is_token = obj.get('area', 0) <= max_red_token_area
                    if seek_red_mode and is_token:
                        red_lanes.add(lane)
                    else:
                        danger_lanes.add(lane)
                else:
                    danger_lanes.add(lane)
            elif obj['type'] == 'GREEN':
                green_lanes.add(lane)

    # --- Longitudinal channel (accel), decoupled from steering (contract section 6) ---
    # Darkness owns the TOP longitudinal priority. EV1 only recovers the light when
    # we send accel=-1.0, and failing EV1 forfeits the Tactical win outright; a chaser
    # collision merely costs 50% speed (a distance penalty). So the darkness brake must
    # NOT be overridden by the chaser the way it used to be -- we brake AND run the
    # chaser sweep steer at the same time, passing EV1 and EV3 together. Easing off near
    # the police car (0.75) still applies, since rear-ending it is an instant game over.
    if low_light_mode:
        target_accel = -1.0          # EV1: full brake to recover the light
    elif police_lanes:
        target_accel = 0.75          # ease off near the police car (avoid game-over rear-end)
    else:
        target_accel = 1.0           # default / chaser evade: floor it

    golden_mode = (golden_time_left > 0) and (1 <= golden_target <= 5)
    rel_golden = (golden_target - (current_lane + 3)) if golden_mode else 0
    chaser_imminent = chaser_behind and chaser_proximity >= chaser_close_proximity

    # --- P0: never hit the police car (instant game over) ---
    if 0 in police_lanes:
        if -1 not in police_lanes and -1 not in danger_lanes and current_lane > LANE_MIN:
            return -1.0, target_accel, "<< EVADE POLICE LEFT", new_evade_dir
        elif 1 not in police_lanes and 1 not in danger_lanes and current_lane < LANE_MAX:
            return 1.0, target_accel, "EVADE POLICE RIGHT >>", new_evade_dir
        elif -1 not in police_lanes and current_lane > LANE_MIN:
            return -1.0, target_accel, "<< EVADE POLICE LEFT (RISK)", new_evade_dir
        elif current_lane < LANE_MAX:
            return 1.0, target_accel, "EVADE POLICE RIGHT (RISK) >>", new_evade_dir
        else:
            return 0.0, 0.5, "TRAPPED BY POLICE", new_evade_dir

    # --- P1: golden LATE-COMMIT (final approach -- EV5 only scores the lane at expiry) ---
    if golden_mode and golden_time_left <= golden_commit_sec and rel_golden not in police_lanes:
        if rel_golden < 0:
            return -1.0, target_accel, f"EV5 COMMIT L{golden_target} <<", new_evade_dir
        elif rel_golden > 0:
            return 1.0, target_accel, f"EV5 COMMIT L{golden_target} >>", new_evade_dir
        else:
            return 0.0, target_accel, f"HOLDING LANE {golden_target}", new_evade_dir

    # --- P1: collect red (EV2), unless the chaser is about to hit us ---
    if seek_red_mode and red_lanes and not chaser_imminent:
        if 0 in red_lanes and 0 not in police_lanes:
            return 0.0, target_accel, "SEEKING RED AHEAD", new_evade_dir
        elif -1 in red_lanes and -1 not in police_lanes and current_lane > LANE_MIN:
            return -1.0, target_accel, "<< SEEKING RED LEFT", new_evade_dir
        elif 1 in red_lanes and 1 not in police_lanes and current_lane < LANE_MAX:
            return 1.0, target_accel, "SEEKING RED RIGHT >>", new_evade_dir

    # --- P1: chaser evade (sweep). accel set above (brake if darkness, else floor) ---
    if chaser_behind:
        # Police lanes are hard-blocked (game over); danger/red lanes are a last
        # resort so the car escapes rather than freezing when boxed in.
        steer, new_evade_dir, text = choose_chaser_evade(current_lane, evade_dir, police_lanes, danger_lanes)
        return steer, target_accel, text, new_evade_dir

    # --- P1: golden EARLY-SEEK (loose window; late-commit above handles the deadline) ---
    if golden_mode and rel_golden not in police_lanes:
        if rel_golden < 0:
            return -1.0, target_accel, f"EV5: SEEKING L{golden_target} <<", new_evade_dir
        elif rel_golden > 0:
            return 1.0, target_accel, f"EV5: SEEKING L{golden_target} >>", new_evade_dir
        else:
            return 0.0, target_accel, f"HOLDING AT LANE {golden_target}", new_evade_dir

    # --- Darkness with nothing else active: brake straight (perception unreliable) ---
    if low_light_mode:
        return 0.0, target_accel, "LOW LIGHT: BRAKING STRAIGHT", new_evade_dir

    # --- P2: evade danger (red only -- yellow is neutral), preferring a green-ward escape ---
    if 0 in danger_lanes:
        safe_left = (-1 not in danger_lanes) and (-1 not in police_lanes) and (current_lane > LANE_MIN)
        safe_right = (1 not in danger_lanes) and (1 not in police_lanes) and (current_lane < LANE_MAX)
        if safe_left and safe_right:
            if 1 in green_lanes:
                return 1.0, target_accel, "EVADE RIGHT (TO GREEN) >>", new_evade_dir
            elif -1 in green_lanes:
                return -1.0, target_accel, "<< EVADE LEFT (TO GREEN)", new_evade_dir
            else:
                return -1.0, target_accel, "<< EVADE LEFT (DEFAULT)", new_evade_dir
        elif safe_left:
            return -1.0, target_accel, "<< EVADE LEFT", new_evade_dir
        elif safe_right:
            return 1.0, target_accel, "EVADE RIGHT >>", new_evade_dir
        else:
            return 0.0, 0.6, "TRAPPED BY DANGER", new_evade_dir

    # --- P2: harvest green across all 5 lanes (toward nearest reachable green) ---
    green_steer, green_label = choose_green_seek(current_lane, green_lanes, police_lanes | danger_lanes)
    if green_steer is not None:
        return green_steer, target_accel, green_label, new_evade_dir

    # --- P2: auto-center ---
    if current_lane < 0 and 1 not in police_lanes and 1 not in danger_lanes:
        return 1.0, target_accel, "AUTO CENTER >>", new_evade_dir
    elif current_lane > 0 and -1 not in police_lanes and -1 not in danger_lanes:
        return -1.0, target_accel, "<< AUTO CENTER", new_evade_dir

    return 0.0, target_accel, "CRUISING", new_evade_dir
