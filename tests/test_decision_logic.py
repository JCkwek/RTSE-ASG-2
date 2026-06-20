import unittest

from decision_logic import evaluate_decision


# --- object builders (mirror the dicts detect_environment emits) ---
def green(*lanes, dist=50):
    return {'type': 'GREEN', 'subtype': 'GREEN', 'lanes': list(lanes), 'dist': dist}


def red(*lanes, area=50, dist=50):
    return {'type': 'DANGER', 'subtype': 'RED', 'lanes': list(lanes), 'area': area, 'dist': dist}


def police(*lanes, dist=50):
    return {'type': 'DANGER', 'subtype': 'POLICE', 'lanes': list(lanes), 'dist': dist}


def decide(objects=None, current_lane=0, low_light=False, chaser=False,
           seek_red=False, golden_time_left=0.0, golden_target=0, evade_dir=1,
           chaser_proximity=0.0):
    return evaluate_decision(
        objects or [], current_lane, low_light, chaser, seek_red,
        golden_time_left, golden_target, evade_dir, chaser_proximity,
    )


class TestAccelDecoupling(unittest.TestCase):
    """CRITICAL fix: darkness must brake (accel=-1.0) even while a chaser is behind,
    while still steering the chaser sweep -- so EV1 and EV3 pass together."""

    def test_darkness_alone_brakes_straight(self):
        steer, accel, label, _ = decide(low_light=True)
        self.assertEqual(accel, -1.0)
        self.assertEqual(steer, 0.0)
        self.assertIn("LOW LIGHT", label)

    def test_darkness_with_chaser_brakes_and_sweeps(self):
        # The regression: old code forced accel=1.0 under a chaser and never sent
        # -1.0, so the light never recovered. Now accel=-1.0 AND a sweep steer.
        steer, accel, label, _ = decide(low_light=True, chaser=True)
        self.assertEqual(accel, -1.0)
        self.assertNotEqual(steer, 0.0)        # a chaser sweep, not a straight brake
        self.assertIn("CHASER", label)

    def test_chaser_alone_floors_it(self):
        steer, accel, _, _ = decide(chaser=True)
        self.assertEqual(accel, 1.0)
        self.assertNotEqual(steer, 0.0)

    def test_idle_cruise_floors_it(self):
        _, accel, label, _ = decide()
        self.assertEqual(accel, 1.0)
        self.assertEqual(label, "CRUISING")

    def test_side_police_eases_off(self):
        _, accel, _, _ = decide(objects=[police(1)])
        self.assertEqual(accel, 0.75)

    def test_police_and_chaser_keeps_police_easeoff(self):
        # No darkness: police ease-off (0.75) now applies even under a chaser,
        # because rear-ending the police car is game over (worse than a chaser hit).
        _, accel, _, _ = decide(objects=[police(1)], chaser=True)
        self.assertEqual(accel, 0.75)


class TestPoliceP0(unittest.TestCase):
    def test_police_ahead_evades_left(self):
        steer, accel, label, _ = decide(objects=[police(0)], current_lane=0)
        self.assertEqual(steer, -1.0)
        self.assertEqual(accel, 0.75)
        self.assertIn("POLICE", label)

    def test_police_ahead_overrides_golden_hold(self):
        # Golden lane is straight ahead (rel 0) but so is the police car ->
        # P0 must win and steer out, never hold into the police lane.
        steer, _, label, _ = decide(objects=[police(0)], current_lane=0,
                                    golden_time_left=1.0, golden_target=3)
        self.assertNotEqual(steer, 0.0)
        self.assertIn("POLICE", label)


class TestGoldenChaserOverlap(unittest.TestCase):
    def test_late_commit_overrides_chaser(self):
        # Within commit window: commit to golden even with a chaser behind.
        steer, _, label, _ = decide(chaser=True, golden_time_left=2.0,
                                    golden_target=5, current_lane=0)
        self.assertEqual(steer, 1.0)           # rel_golden = +2 -> right
        self.assertIn("COMMIT", label)

    def test_early_window_yields_to_chaser(self):
        # Outside commit window: the chaser sweep runs, golden waits.
        _, _, label, _ = decide(chaser=True, golden_time_left=4.0,
                                golden_target=5, current_lane=0)
        self.assertIn("CHASER", label)


class TestSeekRedChaser(unittest.TestCase):
    def test_collects_red_when_chaser_not_imminent(self):
        steer, _, label, _ = decide(objects=[red(-1)], seek_red=True, current_lane=0,
                                    chaser=True, chaser_proximity=0.2)
        self.assertEqual(steer, -1.0)
        self.assertIn("RED", label)

    def test_yields_red_to_imminent_chaser(self):
        _, _, label, _ = decide(objects=[red(-1)], seek_red=True, current_lane=0,
                                chaser=True, chaser_proximity=0.9)
        self.assertIn("CHASER", label)

    def test_large_red_is_never_collected(self):
        # An oversized red (police car body) must be danger, not a seek target.
        steer, _, label, _ = decide(objects=[red(0, area=500)], seek_red=True,
                                    current_lane=0)
        self.assertNotIn("SEEKING RED", label)


class TestHarvestAndEvade(unittest.TestCase):
    def test_seek_green_when_idle(self):
        steer, accel, label, _ = decide(objects=[green(1)], current_lane=0)
        self.assertEqual(steer, 1.0)
        self.assertEqual(accel, 1.0)
        self.assertIn("GREEN", label)

    def test_evade_danger_toward_green(self):
        # Red ahead (not seek mode -> danger), green to the right -> evade right.
        steer, _, label, _ = decide(objects=[red(0), green(1)], current_lane=0)
        self.assertEqual(steer, 1.0)
        self.assertIn("GREEN", label)

    def test_evade_dir_reverses_at_edge_through_chaser(self):
        # evade_dir threading: at the right edge sweeping right -> reversed to left.
        steer, _, _, new_dir = decide(chaser=True, current_lane=2, evade_dir=1)
        self.assertEqual(steer, -1.0)
        self.assertEqual(new_dir, -1)


class TestRedDodge(unittest.TestCase):
    """Reds are the net-green drag -> ease the throttle while dodging one."""

    def test_eases_throttle_dodging_red(self):
        steer, accel, _, _ = decide(objects=[red(0)], current_lane=0)
        self.assertEqual(accel, 0.8)
        self.assertNotEqual(steer, 0.0)

    def test_red_dodge_keeps_police_easeoff(self):
        # Police to the side keeps the 0.75 ease (min with 0.8) while dodging the red.
        _, accel, _, _ = decide(objects=[police(1), red(0)], current_lane=0)
        self.assertEqual(accel, 0.75)

    def test_trapped_by_red_unchanged(self):
        steer, accel, label, _ = decide(objects=[red(-1), red(0), red(1)], current_lane=0)
        self.assertEqual((steer, accel), (0.0, 0.6))
        self.assertIn("TRAPPED", label)

    def test_idle_green_seek_still_floors(self):
        # No red ahead -> no ease; harvesting still runs at full throttle.
        _, accel, _, _ = decide(objects=[green(1)], current_lane=0)
        self.assertEqual(accel, 1.0)


if __name__ == "__main__":
    unittest.main()
