"""Detector-coverage timeline (docs/MATH.md §8; owner decision Q-7).

A non-detection is evidence only where the detector processed the ping (brief §17.9).
Perception publishes one ``Detection2DArray`` per seabed image, stamped with the image's newest
row; images are 256 rows at stride 128 (``BlueBoat-SSS/.../seabed_imager.py:34``), so once array
n has arrived every ping at or before t_{n-1} has been seen by all its images.
"""

from __future__ import annotations

import bisect
from typing import List


class CoverageTimeline:
    def __init__(self, max_gap_s: float) -> None:
        self.max_gap = float(max_gap_s)
        self._t: List[float] = []
        self._real: List[bool] = []

    def __len__(self) -> int:
        return len(self._t)

    def clear(self) -> None:
        self._t.clear()
        self._real.clear()

    def add_array(self, t: float, real: bool) -> None:
        i = bisect.bisect_right(self._t, t)
        self._t.insert(i, float(t))
        self._real.insert(i, bool(real))

    @property
    def release_bound(self) -> float:
        """Pings and detections at or before this time are final (the second-newest stamp)."""
        return self._t[-2] if len(self._t) >= 2 else float("-inf")

    @property
    def newest(self) -> float:
        return self._t[-1] if self._t else float("-inf")

    def covered(self, tau: float) -> bool:
        """tau in (t_{m-1}, t_m] with a gap <= max_gap and both arrays from a real detector."""
        m = bisect.bisect_left(self._t, tau)
        if m < 1 or m >= len(self._t):
            return False
        a, b = self._t[m - 1], self._t[m]
        if not (a < tau <= b) or b - a > self.max_gap:
            return False
        return self._real[m - 1] and self._real[m]

    def prune_before(self, t: float) -> None:
        """Drop stamps older than ``t`` but keep one bracket below it."""
        i = bisect.bisect_left(self._t, t) - 1
        if i > 0:
            del self._t[:i]
            del self._real[:i]
