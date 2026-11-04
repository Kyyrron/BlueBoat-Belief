"""odom -> map translation (docs/MATH.md §3; owner decision D-3).

A translation-only median fit of GPS-fix <-> odom pairs (the MCS ``GeoFit`` model), latched once
valid; or an explicit override (the GPS of odom (0, 0)). Yaw is 0: the real-boat odom yaw is
absolute ENU (``robot_interface.py:627-639``; runtime check LOCAL_CHECKLIST L-1).
"""

from __future__ import annotations

import math
from collections import deque
from typing import Deque, Optional, Tuple

import numpy as np

from .config import AlignmentCfg
from .geo import latlon_to_local_en
from .pose import PoseBuffer

NO_FIX_STATUS = -1


class Aligner:
    def __init__(self, cfg: AlignmentCfg, datum_lat: float, datum_lon: float,
                 expect_identity: bool = False) -> None:
        self.cfg = cfg
        self.datum = (float(datum_lat), float(datum_lon))
        self.expect_identity = bool(expect_identity)
        self._pairs: Deque[Tuple[float, float]] = deque(maxlen=cfg.window)
        self.tx = 0.0
        self.ty = 0.0
        self.sigma = float("nan")
        self.valid = False
        self.latched = False
        self.source = "none"
        self.sim_check_failed = False
        if not math.isnan(cfg.origin_lat):
            e, n = latlon_to_local_en(cfg.origin_lat, cfg.origin_lon, *self.datum)
            self.tx, self.ty = e, n
            self.sigma = float(cfg.override_sigma_m)
            self.valid = True
            self.latched = True
            self.source = "override"

    @property
    def n_pairs(self) -> int:
        return len(self._pairs)

    @staticmethod
    def fix_is_valid(lat: float, lon: float, status: int) -> bool:
        """Only status == -1 means no fix (MCS publishes -2); lat == lon == 0 means no fix."""
        if status == NO_FIX_STATUS:
            return False
        if not (math.isfinite(lat) and math.isfinite(lon)):
            return False
        return not (lat == 0.0 and lon == 0.0)

    def add_fix(self, t: float, lat: float, lon: float, status: int,
                odom: PoseBuffer) -> Optional[str]:
        """Feed one fix; returns a rejection reason or None."""
        if not self.fix_is_valid(lat, lon, status):
            return "no_fix"
        if self.latched:
            return None
        pose = odom.at(t)
        if pose is None:
            return "no_pose_at_stamp"
        e, n = latlon_to_local_en(lat, lon, *self.datum)
        self._pairs.append((e - pose[0], n - pose[1]))
        self._refit()
        return None

    def _refit(self) -> None:
        d = np.asarray(self._pairs, dtype=np.float64)
        med = np.median(d, axis=0)
        spread = 1.4826 * np.median(np.abs(d - med), axis=0)
        sigma = float(math.sqrt(0.5 * float(spread[0] ** 2 + spread[1] ** 2)))
        self.tx, self.ty = float(med[0]), float(med[1])
        self.sigma = sigma
        ok = len(d) >= self.cfg.min_pairs and sigma <= self.cfg.max_spread_m
        if ok and not self.valid:
            self.valid = True
            self.source = "fit"
        if ok and self.cfg.latch:
            self.latched = True
        if self.valid and self.expect_identity:
            self.sim_check_failed = math.hypot(self.tx, self.ty) > self.cfg.sim_check_m

    def to_map(self, x, y):
        return x + self.tx, y + self.ty
