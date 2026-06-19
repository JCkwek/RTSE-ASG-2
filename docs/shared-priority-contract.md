# Shared Decision Priority Contract

**Status:** Draft for team review (Leow). **Touches all modules — adopt as a team.**
**Date:** 2026-06-19

## Why this exists

The Tactical win is a *global* objective: **net ≥ +60 green AND pass every event**, with
**Tactical win > distance at 180s**. Every module (lane estimate, police, golden, chaser, green)
competes for the same two scarce resources — **the car's lane** and **time**. They are already
"connected" in one place: the priority cascade in `evaluate_decision`. This contract makes that
ordering deliberate and gives each module a fixed interface so the four parts compose instead of
fighting.

The controller is **blind to score, clock, and pass/fail** (camera frames only). So the policy is a
**fixed priority order**, not a dynamic "am I winning?" strategy. The order is derived directly from
the win rules.

## 1. Single source of truth: the lane estimate

`net_lane_position` (int, −2…+2; absolute lane = rel + 3 → L1…L5) is the **shared substrate**.
Everything downstream — golden targeting, the chaser sweep's edge reversal, green/red lane
assignment — is only as correct as this value.

- **Owner:** Kwek (lane detection).
- **Consumers:** everyone, read-only via `shared_data['net_lane_position']`.
- **Rule:** no module computes its own lane estimate. Fix accuracy here once; everyone benefits.

## 2. Priority tiers (derived from the win rules)

| Tier | Meaning | Rule of thumb |
|------|---------|---------------|
| **P0 — Survive** | Never do something that ends the game | Never enter a **police** lane; never hit the police car |
| **P1 — Pass the active event(s)** | Tactical needs *every* event passed | Darkness brake, chaser evade, collect red, hold golden |
| **P2 — Harvest** | Maximize net green when no event needs the lane | Seek green, avoid red (net = green − red) |

**Arbitration rule:** the highest tier with an *active desire* wins. P0 is a hard constraint layered
on **all** tiers — a P1/P2 action may never steer into a police lane.

## 3. Per-behavior contract

Each behavior is "active" only on its trigger, and emits a **desired relative steer** (−1/0/+1),
an **acceleration**, and a **tier**. `accel` and `steer` are independent signals (see §6).

| Behavior | Owner | Tier | Active when | Desired steer | Accel | Notes |
|----------|-------|------|-------------|---------------|-------|-------|
| Police-ahead evade | police | **P0** | `0 in police_lanes` | away from police, never into police/danger | 0.75 | Game-over avoidance — overrides P1/P2 steer |
| Darkness brake | (low-light) | **P1** | `low_light_mode` | (see §6: brake; weave only if chaser) | **−1.0** | EV1 = full brake |
| Chaser evade | **Leow** | **P1** | `chaser_behind` (latched) | `choose_chaser_evade` sweep | 1.0 | police = hard-block, danger = soft-block |
| Collect red | police | **P1** | `seek_red_mode and red_lanes` | toward red, not into police | 1.0 | EV2; yields to chaser only when `chaser_proximity ≥ CHASER_CLOSE_PROXIMITY` |
| Golden hold | golden | **P1** | `golden_mode` | toward `golden_target` (see §5 timing) | 1.0 | EV5: only the lane *at expiry* matters |
| Danger (red/yellow) evade | green/Kew | **P2** | `0 in danger_lanes` | to a safe lane, prefer green-ward | ~0.6 | avoid losing green to a red hit |
| Seek green | Kew | **P2** | `green_lanes` | toward nearest green | 1.0 | drives net +60 |
| Auto-center | shared | **P2** | else | toward lane 0 | 1.0 | default |

**Blocked-lane convention (already implemented for the chaser):**
- **hard-blocked** = police lanes → never enter (P0).
- **soft-blocked** = danger/red lanes → enter only as a last resort to satisfy a higher tier.

## 4. Overlap resolution (the event schedule forces these)

Within each 60s cycle the windows overlap: **0–30s** can have Darkness + Chaser A + Golden;
**30–50s** can have Police + Chaser B + Golden. The contract must resolve each pair the same way
every time:

| Overlap | Conflict | Resolution |
|---------|----------|------------|
| Police-ahead + anything | game over vs event | **P0 wins** — evade police first, always |
| Darkness + Chaser | brake (EV1) vs evade (EV3) | **Do both:** `accel = −1.0` *and* run the chaser sweep steer (§6). Today low-light just yields to the chaser → EV1 fails. |
| Police(collect red) + Chaser | grab red (EV2) vs evade (EV3) | Collect red **unless** `chaser_proximity ≥ CHASER_CLOSE_PROXIMITY`, then weave. *(implemented)* |
| Golden + Chaser | hold lane (EV5) vs evade (EV3) | Weave until **golden timer ≤ ~1.0s left**, then commit to the golden lane. EV5 only needs the lane at expiry. |
| Golden + Police-ahead | hold lane vs game over | Police P0 wins; golden must skip any steer that enters a police lane (`rel_golden in police_lanes` guard — already present). |

## 5. Golden timing rule (new, enables Golden+Chaser coexistence)

Golden exposes its remaining time so it can yield:
- `golden_time_left = golden_lane_end_time − now`
- If `golden_time_left > GOLDEN_COMMIT_SEC` and a higher-or-equal P1 (chaser) is active →
  **let the chaser sweep run**.
- If `golden_time_left ≤ GOLDEN_COMMIT_SEC` → **commit to the golden lane** regardless (final approach).

`GOLDEN_COMMIT_SEC = 2.5` (implemented). It is **not** ~1.0s because the OCR latch
(`now + 5.5s`) overshoots the real 5s EV5 timer by ~the OCR read lag (~1s). A 2.5s commit
window makes the car hold the golden lane across the *real* expiry instant despite that lag.
Tune against EV5 hit rate: raise it if EV5 is still missed (commit earlier), lower it if the
chaser catches the car during golden overlaps.

This converts golden from "always hold" to "hold at the moment that scores," freeing the earlier
seconds for evasion and green harvest.

## 6. Decouple acceleration from steering

Today `low_light` returns `(0.0, −1.0, …)` and `and not chaser_behind` makes it surrender to the
chaser entirely. Instead, treat **accel and steer as separate channels**:

- `accel` is owned by the most urgent *longitudinal* requirement: **−1.0 if Darkness**, 0.75 near
  police, else 1.0.
- `steer` is owned by the highest-tier *lateral* requirement (the cascade in §7).

So Darkness + Chaser becomes `accel = −1.0` **and** `steer = sweep` — passing EV1 and EV3 together.

## 7. Concrete cascade order for `evaluate_decision`

Map the tiers onto the existing function (compute `accel` first, then pick `steer`):

```
1. Compute police_lanes / danger_lanes / red_lanes / green_lanes  (exists)
2. accel = -1.0 if low_light else (0.75 if police_lanes else 1.0)  (decoupled — NEW)
3. P0  if 0 in police_lanes:            -> evade police (never into police/danger)
4. P1  if golden_mode and time_left<=GOLDEN_COMMIT_SEC: -> commit golden lane
5. P1  if chaser_behind:                -> choose_chaser_evade(police=hard, danger=soft)
6. P1  if golden_mode:                  -> seek golden (early window, may be preempted above)
7. P1  if seek_red_mode and red_lanes and not chaser_imminent: -> collect red
8. P2  if 0 in danger_lanes:            -> evade to safe lane, prefer green-ward
9. P2  if green_lanes:                  -> seek green
10. P2 else:                            -> auto-center
   (return (steer, accel, label) — accel from step 2 unless a step overrides longitudinally)
```

Differences from today: step 2 decouples accel (enables Darkness+Chaser); golden splits into a
late-commit (step 4, above chaser) and an early-seek (step 6, below chaser); everything else keeps
its current behavior. P0 stays a hard constraint on every steering choice.

## 8. P2 harvest note (net +60)

Net green = **green − red**, so dodging a red is worth as much as taking a green. Keep "evade toward
green" in the danger step (already there), and seek green every idle frame. The weave necessarily
costs harvest time during chaser events — that's inherent, not a bug.

## 9. Open decisions for the team

1. `GOLDEN_COMMIT_SEC` (implemented 2.5s) — validate against measured OCR read lag and EV5 hit rate.
2. Is braking-while-steering effective in-game during Darkness+Chaser, or does the game ignore
   steer at `accel = −1.0`? (Needs a Windows test.)
3. Confirm Chaser B's exact window vs Police (both ~30–50s) to validate the proximity guard.
4. Who owns the merged `evaluate_decision` after this reorder (single integrator vs. pairing)?
