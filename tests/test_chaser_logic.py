import unittest
from chaser_logic import choose_chaser_evade, chaser_box_metrics


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


if __name__ == "__main__":
    unittest.main()
