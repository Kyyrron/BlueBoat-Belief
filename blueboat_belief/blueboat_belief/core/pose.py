"""Pose buffer and frozen-pose detector (brief §7; docs/MATH.md §3, §1.5).

Times are explicit floats on the sonar-profile clock; nothing here reads a clock.
"""

from __future__ import annotations

import bisect
import math
from typing import List, Optional, Tuple

Pose = Tuple[float, float, float]   # x, y, yaw


def _lerp_angle(a: float, b: float, f: float) -> float:
    d = (b - a + math.pi) % (2.0 * math.pi) - math.pi
    return a + f * d


class PoseBuffer:
    """Time-sorted (x, y, yaw) samples; tolerant to small out-of-order insertion."""

    def __init__(self, tolerance_s: float, buffer_s: float) -> None:
        self.tol = float(tolerance_s)
        self.span = float(buffer_s)
        self._t: List[float] = []
        self._p: List[Pose] = []

    def __len__(self) -> int:
        return len(self._t)

    def clear(self) -> None:
        self._t.clear()
        self._p.clear()

    def add(self, t: float, x: float, y: float, yaw: float) -> None:
        i = bisect.bisect_right(self._t, t)
        self._t.insert(i, float(t))
        self._p.insert(i, (float(x), float(y), float(yaw)))
        newest = self._t[-1]
        cut = bisect.bisect_left(self._t, newest - self.span)
        if cut > 0:
            del self._t[:cut]
            del self._p[:cut]

    def at(self, t: float) -> Optional[Pose]:
        """Interpolated pose at ``t`` (both brackets within tolerance), else the nearest
        sample within tolerance, else None."""
        ts = self._t
        if not ts:
            return None
        i = bisect.bisect_left(ts, t)
        if 0 < i < len(ts):
            t0, t1 = ts[i - 1], ts[i]
            if t - t0 <= self.tol and t1 - t <= self.tol:
                if t1 == t0:
                    return self._p[i]
                f = (t - t0) / (t1 - t0)
                (x0, y0, a0), (x1, y1, a1) = self._p[i - 1], self._p[i]
                return (x0 + f * (x1 - x0), y0 + f * (y1 - y0), _lerp_angle(a0, a1, f))
        best = None
        for j in (i - 1, i):
            if 0 <= j < len(ts) and abs(ts[j] - t) <= self.tol:
                if best is None or abs(ts[j] - t) < abs(ts[best] - t):
                    best = j
        return None if best is None else self._p[best]


class FrozenPoseDetector:
    """>= ``count`` consecutive poses at exactly (0, 0): the live '/blueboat/odom reads zero'
    pathology (brief §2, §7). While active, such poses are rejected and diagnostics say ERROR."""

    def __init__(self, count: int) -> None:
        self.count = int(count)
        self.streak = 0
        self.ever_active = False

    def update(self, x: float, y: float) -> bool:
        """Feed one pose; True when this pose must be rejected as frozen."""
        if x == 0.0 and y == 0.0:
            self.streak += 1
        else:
            self.streak = 0
        if self.streak >= self.count:
            self.ever_active = True
            return True
        return False

    @property
    def active(self) -> bool:
        return self.streak >= self.count
