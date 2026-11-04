"""Turn back-scan gating (docs/MATH.md §4.6; owner decision 2026-11-04).

Evidence, primary text read:
- Paull et al. 2013 §II-C: when a side-scan sonar turns sharply, areas on the outside of the
  turn are missed and areas on the inside "can become completely distorted", which defeats ATR.
- Perception flags the same region per row as ``backscan_from_m = r*``, citing Cobra et al. 1992
  (it was not read here).

Beyond the instantaneous turn radius ``r* = v/|ω|``, the inner side is scanned backwards. A cell
there would otherwise collect looks from a sweeping heading, i.e. false multi-aspect credit.
Those samples are therefore excluded from looks, aspects and coverage, and counted. Detections
are unchanged: Perception's turn flag is a flag, never a rejection.

Deterministic. The estimate uses only the poses of processed pings, in processing order (time
order).
"""

from __future__ import annotations

import math
from collections import deque
from typing import Deque, Optional, Tuple

from .config import TurnCfg


def _wrap(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


class TurnEstimator:
    """Smoothed yaw rate and speed over ``window_s`` of ping time."""

    def __init__(self, cfg: TurnCfg) -> None:
        self.cfg = cfg
        self._hist: Deque[Tuple[float, float, float, float]] = deque()   # t, x, y, unwrapped yaw
        self._last_yaw: Optional[float] = None
        self._unwrapped = 0.0

    def clear(self) -> None:
        self._hist.clear()
        self._last_yaw = None
        self._unwrapped = 0.0

    def update(self, t: float, x: float, y: float, yaw: float) -> Tuple[float, float, float, int]:
        """Feed one ping pose; return (yaw_rate, speed, r_star, inner_side).

        - ``inner_side`` is +1 (port, a left turn, ω > 0), -1 (starboard) or 0 when not turning
          (``r_star`` = inf).
        - A ping older than the newest is ignored for the estimate (processor bursts); the
          current estimate is returned.
        - A gap longer than ``window_s`` restarts the estimate: no rate is inferred across a gap.
        """
        if self._hist and t < self._hist[-1][0]:
            return self._estimate()
        if self._hist and t - self._hist[-1][0] > self.cfg.window_s:
            self.clear()                      # a gap in the pings: no rate across it
        if self._last_yaw is None:
            self._unwrapped = yaw
        else:
            self._unwrapped += _wrap(yaw - self._last_yaw)
        self._last_yaw = yaw
        self._hist.append((t, x, y, self._unwrapped))
        while len(self._hist) > 2 and self._hist[1][0] <= t - self.cfg.window_s:
            self._hist.popleft()
        return self._estimate()

    def _estimate(self) -> Tuple[float, float, float, int]:
        if len(self._hist) < 2:
            return 0.0, 0.0, math.inf, 0
        t0, x0, y0, a0 = self._hist[0]
        t1, x1, y1, a1 = self._hist[-1]
        dt = t1 - t0
        if dt <= 0.0:
            return 0.0, 0.0, math.inf, 0
        omega = (a1 - a0) / dt
        speed = math.hypot(x1 - x0, y1 - y0) / dt
        if abs(omega) < self.cfg.min_yaw_rate_rad_s:
            return omega, speed, math.inf, 0
        r_star = speed / abs(omega) if speed >= self.cfg.min_speed_mps else 0.0
        return omega, speed, r_star, (1 if omega > 0 else -1)
