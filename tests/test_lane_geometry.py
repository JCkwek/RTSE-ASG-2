import unittest

from lane_geometry import occupied_lanes


class TestFallbackModel(unittest.TestCase):
    """No measured curbs -> the old hardcoded model (center 160, slope 0.44)."""

    def test_center_token_is_lane_zero(self):
        # cx == 160 (frame center) -> relative lane 0 (the center lane).
        self.assertEqual(occupied_lanes(160, 80, 10), [0])

    def test_one_lane_right(self):
        # dist=100 -> lane_width=44; cx = 160 + 44 -> exactly +1.
        self.assertEqual(occupied_lanes(204, 80, 10), [1])

    def test_one_lane_left(self):
        self.assertEqual(occupied_lanes(116, 80, 10), [-1])

    def test_above_horizon_returns_empty(self):
        # actual_y <= HORIZON_Y (80) -> no road plane to classify against.
        self.assertEqual(occupied_lanes(160, -25, 10), [])

    def test_wide_blob_spans_three_lanes(self):
        # width 200 > lane_width(44)*1.5 -> occupies the neighbours too.
        self.assertEqual(occupied_lanes(160, 80, 200), [-1, 0, 1])


class TestMeasuredModel(unittest.TestCase):
    """With measured road_bounds, classify against the REAL center/width."""

    def test_uses_measured_center_not_frame_center(self):
        # Road measured shifted right: center at x=200. A token sitting at the
        # measured center is the CENTER lane (0)...
        bounds = (100, 300, 40, 85)  # x_left, x_right, lane_w, scan_y -> center 200
        self.assertEqual(occupied_lanes(200, 80, 10, road_bounds=bounds), [0])

    def test_same_token_misclassified_without_bounds(self):
        # ...but the old hardcoded model (center 160) calls that same token lane +1.
        # This is exactly the "off-center lane recognized as the wrong lane" bug.
        self.assertEqual(occupied_lanes(200, 80, 10, road_bounds=None), [1])

    def test_perspective_scales_lane_width_by_depth(self):
        # Same screen-x offset is a LOWER lane number when the object is near
        # (wide lanes) than when it is far (narrow lanes near the horizon).
        bounds = (60, 260, 40, 85)  # center 160, lane_w 40 at scan depth
        near = occupied_lanes(190, 85, 10, road_bounds=bounds)   # dist ~105
        far = occupied_lanes(190, 5, 10, road_bounds=bounds)     # dist ~25
        self.assertEqual(near, [1])
        self.assertEqual(far, [2])

    def test_clamps_to_track_extents(self):
        # A token far outside the measured road still clamps to +/-2.
        bounds = (60, 260, 40, 85)
        self.assertEqual(occupied_lanes(319, 85, 10, road_bounds=bounds), [2])


if __name__ == "__main__":
    unittest.main()
