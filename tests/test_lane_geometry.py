import unittest

from lane_geometry import occupied_lanes, fit_road_model, model_from_bounds


class TestFallbackModel(unittest.TestCase):
    """No model -> the flat-road fallback (center 160, slope 0.44)."""

    def test_center_token_is_lane_zero(self):
        self.assertEqual(occupied_lanes(160, 80, 10), [0])

    def test_one_lane_right(self):
        # dist=100 -> lane_width=44; cx = 160 + 44 -> exactly +1.
        self.assertEqual(occupied_lanes(204, 80, 10), [1])

    def test_one_lane_left(self):
        self.assertEqual(occupied_lanes(116, 80, 10), [-1])

    def test_above_horizon_returns_empty(self):
        self.assertEqual(occupied_lanes(160, -25, 10), [])

    def test_wide_blob_spans_three_lanes(self):
        self.assertEqual(occupied_lanes(160, 80, 200), [-1, 0, 1])


class TestModelFromBounds(unittest.TestCase):
    """Single measured row: use the REAL center (slope-unaware width)."""

    def test_uses_measured_center_not_frame_center(self):
        m = model_from_bounds((100, 300, 40, 85))  # center 200
        self.assertEqual(occupied_lanes(200, 80, 10, m), [0])

    def test_same_token_misclassified_without_model(self):
        # The old hardcoded center (160) calls that same token lane +1 -- the
        # "off-center lane recognized as the wrong lane" bug.
        self.assertEqual(occupied_lanes(200, 80, 10, None), [1])

    def test_none_bounds_returns_none(self):
        self.assertIsNone(model_from_bounds(None))


class TestFitRoadModel(unittest.TestCase):
    """Fit a floating horizon from multi-row curb samples (slope robustness)."""

    def _samples(self, horizon, width_slope=2.0, center=160.0, rows=(120, 140, 160, 180)):
        return [(y, center, width_slope * (y - horizon)) for y in rows]

    def test_recovers_flat_horizon(self):
        m = fit_road_model(self._samples(80))
        self.assertIsNotNone(m)
        self.assertAlmostEqual(m.horizon_y, 80, places=3)

    def test_recovers_shifted_horizon_on_slope(self):
        # Camera pitched: vanishing point moves to row 110 -> the fit must follow.
        m = fit_road_model(self._samples(110, rows=(130, 150, 170, 190)))
        self.assertAlmostEqual(m.horizon_y, 110, places=3)

    def test_rejects_too_few_samples(self):
        self.assertIsNone(fit_road_model(self._samples(80, rows=(180, 185))))

    def test_rejects_low_vertical_span(self):
        self.assertIsNone(fit_road_model(self._samples(80, rows=(180, 184, 188))))

    def test_rejects_non_increasing_width(self):
        # Width must grow toward the car; a shrinking trend is noise -> reject.
        bad = [(120, 160, 200), (140, 160, 150), (160, 160, 100)]
        self.assertIsNone(fit_road_model(bad))

    def test_floating_horizon_changes_classification(self):
        # Same token (cx=200, row 160), two horizons -> different lane width ->
        # different lane. This is the slope-induced misread the fit prevents.
        flat = fit_road_model(self._samples(80))
        slope = fit_road_model(self._samples(120, rows=(130, 150, 170, 190)))
        self.assertEqual(occupied_lanes(200, 60, 10, flat), [1])
        self.assertEqual(occupied_lanes(200, 60, 10, slope), [2])

    def test_fit_recovers_curved_center(self):
        # Center drifts right with depth -> cx_slope captured, classification follows.
        samples = [(y, 160 + 0.5 * (y - 120), 2.0 * (y - 80)) for y in (120, 140, 160, 180)]
        m = fit_road_model(samples)
        self.assertIsNotNone(m)
        self.assertGreater(m.cx_slope, 0)


if __name__ == "__main__":
    unittest.main()
