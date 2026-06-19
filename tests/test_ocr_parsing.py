import unittest

from ocr_parsing import parse_lane


class TestParseLane(unittest.TestCase):
    def test_clean(self):
        self.assertEqual(parse_lane("LANE 4"), 4)

    def test_no_space(self):
        self.assertEqual(parse_lane("LANE3"), 3)

    def test_with_prefix_word(self):
        self.assertEqual(parse_lane("GOLDEN LANE 5"), 5)

    def test_garbled_a_to_4_takes_lane_digit_not_the_4(self):
        # "LANE" misread as "L4NE": the 4 is the garbled A, 2 is the real lane.
        self.assertEqual(parse_lane("L4NE 2"), 2)

    def test_garbled_l_to_i(self):
        self.assertEqual(parse_lane("IANE 1"), 1)

    def test_garbled_l_to_1(self):
        self.assertEqual(parse_lane("1ANE5"), 5)

    def test_garbled_e_to_3(self):
        self.assertEqual(parse_lane("LAN3 4"), 4)

    def test_colon_separator(self):
        self.assertEqual(parse_lane("LANE: 2"), 2)

    def test_rejects_out_of_range(self):
        self.assertIsNone(parse_lane("LANE 7"))

    def test_rejects_no_lane_token(self):
        self.assertIsNone(parse_lane("SCORE 3"))

    def test_empty(self):
        self.assertIsNone(parse_lane(""))

    def test_none(self):
        self.assertIsNone(parse_lane(None))


if __name__ == "__main__":
    unittest.main()
