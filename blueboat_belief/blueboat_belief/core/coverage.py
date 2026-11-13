"""Detector-coverage timelines (docs/MATH.md §8; owner decisions Q-7 and 2026-11-12).

A non-detection is evidence only where the detector processed the ping (brief §17.9). Two
sources say where that was:

- ``CoverageTimeline`` (§8.1, a heuristic): the stamps of ``/sss_ai/detections`` arrays.
  Perception publishes one array per seabed image, stamped with the image's newest row; images
  are 256 rows at stride 128 (``BlueBoat-SSS/.../seabed_imager.py:34``), so once array n has
  arrived every ping at or before t_{n-1} has been seen by all its images.
- ``RecordTimeline`` (§8.2, exact): the row span and detector status of each image, from
  Perception's per-image records (``/sss_ai/image_records``).
"""

from __future__ import annotations

import bisect
import math
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


class RecordTimeline:
    """Exact image spans from Perception's per-image records (docs/MATH.md §8.2).

    Records are kept sorted by ``t_max``. Each one carries its row span ``[t_min, t_max]`` and
    whether a real detector searched it. Images are assumed to overlap only their two
    neighbours (256 rows at stride 128); ``add`` checks that on every record and reports the
    first violation, after which the release bound falls back to the always-valid ``t_min``.
    """

    SCAN = 16                                   # records examined around a ping (>= any overlap)

    def __init__(self) -> None:
        self._tmin: List[float] = []
        self._tmax: List[float] = []
        self._searched: List[bool] = []
        self.overlap_ok = True

    def __len__(self) -> int:
        return len(self._tmax)

    def clear(self) -> None:
        self._tmin.clear()
        self._tmax.clear()
        self._searched.clear()
        self.overlap_ok = True

    def add(self, t_min: float, t_max: float, searched: bool) -> bool:
        """Insert one record; return True when this record is the first to break the
        neighbours-only overlap assumption (``t_min(m) < t_max(m-2)``)."""
        i = bisect.bisect_right(self._tmax, t_max)
        self._tmin.insert(i, float(t_min))
        self._tmax.insert(i, float(t_max))
        self._searched.insert(i, bool(searched))
        if self.overlap_ok and i >= 2 and self._tmin[i] < self._tmax[i - 2]:
            self.overlap_ok = False
            return True
        return False

    @property
    def release_bound(self) -> float:
        """Events strictly before this time are final (returned as an inclusive bound)."""
        n = len(self._tmax)
        if self.overlap_ok:
            if n < 2:
                return float("-inf")
            return math.nextafter(self._tmax[-2], -math.inf)
        if n < 1:
            return float("-inf")
        return math.nextafter(max(self._tmin), -math.inf)

    def covered(self, tau: float) -> bool:
        """Some searched record has ``t_min <= tau <= t_max``."""
        i = bisect.bisect_left(self._tmax, tau)
        for j in range(i, min(i + self.SCAN, len(self._tmax))):
            if self._searched[j] and self._tmin[j] <= tau <= self._tmax[j]:
                return True
        return False
