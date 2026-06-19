"""Pure parsing of the Golden Lane OCR text (EV5).

Dependency-free so it can be unit-tested without Tesseract / OpenCV.

Tesseract frequently garbles the short flash "LANE N": the L reads as I/1, the A
as 4/R, the N as M/H, the E as 3. parse_lane tolerates those substitutions and,
crucially, takes the digit that follows the LANE-like token -- so "L4NE 2" yields
2 (the lane), not 4 (the garbled A).
"""

import re

# L?->[LI1]  A?->[A4R]  N?->[NMH]  optional E?->[E3]  optional separator  digit 1-5
_LANE_RE = re.compile(r"[LI1][A4R][NMH][E3]?\s*[:\-]?\s*([1-5])")


def parse_lane(text):
    """Return the golden lane number (1-5) from noisy OCR text, or None."""
    if not text:
        return None
    m = _LANE_RE.search(text.upper())
    return int(m.group(1)) if m else None
