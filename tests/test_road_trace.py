import unittest

from road_trace import (
    find_anchor, trace_road, interp, occupied_lanes_from_profile, car_lane_from_profile,
)


def make_rows(ys, center_fn, halfwidth_fn, extra=None):
    """Synthetic curb rows: two curb pixels per row at center +/- halfwidth."""
    rows = {}
    for y in ys:
        c, hw = center_fn(y), halfwidth_fn(y)
        xs = [round(c - hw), round(c + hw)]
        if extra and y in extra:
            xs += extra[y]
        rows[y] = sorted(xs)
    return rows


FRAME_W = 320
YS = [180, 170, 160, 150, 140, 130, 120]


class TestAnchor(unittest.TestCase):
    def test_widest_gap(self):
        self.assertEqual(find_anchor([96, 224], FRAME_W, 40), (96, 224))

    def test_none_when_too_narrow(self):
        self.assertIsNone(find_anchor([150, 170], FRAME_W, 40))


class TestTraceShapes(unittest.TestCase):
    def test_straight_flat_road(self):
        rows = make_rows(YS, lambda y: 160, lambda y: 0.8 * (y - 100))
        prof = trace_road(rows, YS, FRAME_W, min_gap=40, window=25)
        self.assertEqual(len(prof), 7)
        self.assertTrue(all(abs(c - 160) < 1e-6 for _, c, _ in prof))
        widths = [w for _, _, w in prof]
        self.assertEqual(widths, sorted(widths, reverse=True))  # narrows upward

    def test_turn_curves_the_center(self):
        # Center slides right going up (a right bend) -> trace must follow it.
        rows = make_rows(YS, lambda y: 160 + 0.5 * (180 - y), lambda y: 0.8 * (y - 100))
        prof = trace_road(rows, YS, FRAME_W, min_gap=40, window=25)
        centers = [c for _, c, _ in prof]
        self.assertTrue(all(b > a for a, b in zip(centers, centers[1:])))  # bends right

    def test_uphill_trace_is_short(self):
        # Road narrows fast and crests early -> few rows traced.
        rows = make_rows(YS, lambda y: 160, lambda y: 2.0 * (y - 150))
        prof = trace_road(rows, YS, FRAME_W, min_gap=40, window=25)
        self.assertLessEqual(len(prof), 3)

    def test_downhill_trace_is_long(self):
        # Gentle narrowing -> road runs far -> many rows traced.
        ys = list(range(180, 80, -10))
        rows = make_rows(ys, lambda y: 160, lambda y: 0.4 * (y - 60))
        prof = trace_road(rows, ys, FRAME_W, min_gap=40, window=25)
        self.assertGreaterEqual(len(prof), 8)

    def test_window_ignores_far_blob(self):
        # A stray red-sign pixel near the center must not capture the edge trace.
        rows = make_rows(YS, lambda y: 160, lambda y: 0.8 * (y - 100),
                         extra={150: [160], 140: [160]})
        prof = trace_road(rows, YS, FRAME_W, min_gap=40, window=25)
        self.assertEqual(len(prof), 7)
        self.assertTrue(all(abs(c - 160) < 1e-6 for _, c, _ in prof))

    def test_no_anchor_returns_empty(self):
        rows = {y: [150, 170] for y in YS}  # never a wide enough gap
        self.assertEqual(trace_road(rows, YS, FRAME_W, min_gap=40, window=25), [])


class TestInterpAndClassify(unittest.TestCase):
    def setUp(self):
        self.prof = trace_road(
            make_rows(YS, lambda y: 160, lambda y: 0.8 * (y - 100)),
            YS, FRAME_W, min_gap=40, window=25)

    def test_interp_between_rows(self):
        c, w = interp(self.prof, 175)
        self.assertAlmostEqual(c, 160.0)
        self.assertTrue(112 <= w <= 128)  # between the y170 and y180 widths

    def test_interp_above_trace_is_none(self):
        self.assertIsNone(interp(self.prof, 90))

    def test_interp_below_uses_anchor(self):
        c, w = interp(self.prof, 200)
        self.assertEqual((c, w), (160.0, 128.0))

    def test_center_token_is_lane_zero(self):
        self.assertEqual(occupied_lanes_from_profile(self.prof, 160, 170, 5), [0])

    def test_offset_token_other_lane(self):
        # y170 width 112 -> lane_width 22.4; +45 px ~ +2 lanes.
        self.assertEqual(occupied_lanes_from_profile(self.prof, 205, 170, 5), [2])

    def test_wide_blob_spreads(self):
        self.assertEqual(occupied_lanes_from_profile(self.prof, 160, 170, 200), [-1, 0, 1])

    def test_token_above_road_unclassified(self):
        self.assertEqual(occupied_lanes_from_profile(self.prof, 160, 90, 5), [])


class TestCarLane(unittest.TestCase):
    def test_centered_car_is_lane_zero(self):
        prof = trace_road(make_rows(YS, lambda y: 160, lambda y: 0.8 * (y - 100)),
                          YS, FRAME_W, min_gap=40, window=25)
        self.assertEqual(car_lane_from_profile(prof, 160), 0)

    def test_car_left_of_road_center(self):
        # Road center measured at 200 at the bottom; car at 160 sits left of it.
        prof = trace_road(make_rows(YS, lambda y: 200, lambda y: 0.8 * (y - 100)),
                          YS, FRAME_W, min_gap=40, window=25)
        self.assertEqual(car_lane_from_profile(prof, 160), -2)

    def test_empty_profile_returns_none(self):
        self.assertIsNone(car_lane_from_profile([]))


if __name__ == "__main__":
    unittest.main()
