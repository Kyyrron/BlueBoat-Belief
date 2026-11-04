"""Negative evidence on the grid: looks, misses, coverage, aspect history (docs/MATH.md §4-§7).

Vectorised per ping side. All per-cell state is a flat numpy array of length W*H
(per-sector arrays W*H*S, index cell*S + s). Deterministic: no randomness, no clock.
"""

from __future__ import annotations

from typing import Optional, Tuple

import numpy as np

from .area import GridGeometry
from .config import BeliefConfig
from .geometry import look_bearing, sample_xy, sector_of
from .models import combine_miss, logit_scalar, pd_range, pfa_log_factor, prior_cell, rho_table, sigmoid

U16_MAX = np.iinfo(np.uint16).max
U32_MAX = np.iinfo(np.uint32).max


class Grid:
    def __init__(self, cfg: BeliefConfig, geom: GridGeometry, inside: np.ndarray) -> None:
        self.cfg = cfg
        self.geom = geom
        self.inside = np.asarray(inside, dtype=bool)
        n, S = geom.n_cells, cfg.aspect.sector_count
        self.S = S
        self.p0 = prior_cell(cfg.prior, geom.resolution)
        self.ell0 = logit_scalar(self.p0) if self.p0 > 0 else float("-inf")
        self.rho_tab = rho_table(S, cfg.aspect.origin_deg, cfg.rho.fold_opposite,
                                 cfg.rho.independence_angle_deg)
        self.pfa_term = pfa_log_factor(cfg.pd.pfa)
        # channels
        self.ell = np.full(n, self.ell0, dtype=np.float64)       # log-odds of p_background (raw)
        self.lnq = np.zeros(n, dtype=np.float64)                 # aggregate ln Q
        self.maxpd_s = np.zeros(n * S, dtype=np.float64)         # max covered PD per sector
        self.n_looks = np.zeros(n, dtype=np.uint16)
        self.n_pings = np.zeros(n, dtype=np.uint32)
        self.aspect_counts = np.zeros(n * S, dtype=np.uint8)
        self.depth = np.full(n, np.nan, dtype=np.float32)
        self.intensity = np.full(n, np.nan, dtype=np.float32)
        self.max_conf = np.zeros(n, dtype=np.float32)
        # open-look state
        self.look_sector = np.full(n, -1, dtype=np.int16)
        self.look_epoch = np.full(n, -1, dtype=np.int64)
        self.look_last = np.full(n, -np.inf, dtype=np.float64)
        self.look_rho = np.ones(n, dtype=np.float64)
        self.look_pd = np.zeros(n, dtype=np.float64)
        self.look_cov = np.zeros(n, dtype=bool)
        self.lnq_open = np.zeros(n, dtype=np.float64)
        self.ell_open = np.full(n, self.ell0, dtype=np.float64)
        self.searched = np.zeros(n, dtype=np.int64)             # bitmask of sectors with PD* > 0

    # ---------------------------------------------------------------------------------------
    def side_cells(self, x: float, y: float, yaw: float, yoff: np.ndarray, db: np.ndarray
                   ) -> Optional[Tuple[np.ndarray, np.ndarray, np.ndarray]]:
        """Cells hit by one ping side (map frame): (unique cells, PD per cell, mean dB per cell)."""
        if yoff.size == 0:
            return None
        px, py = sample_xy(x, y, yaw, yoff)
        idx = self.geom.cell_of(px, py)
        ok = idx >= 0
        ok[ok] = self.inside[idx[ok]]
        if not ok.any():
            return None
        idx = idx[ok]
        r = np.abs(yoff[ok]).astype(np.float64) - self.cfg.pd.lever_arm_m
        pd = pd_range(r, self.cfg.pd)
        uniq, inv = np.unique(idx, return_inverse=True)
        pd_c = np.zeros(uniq.size, dtype=np.float64)
        np.maximum.at(pd_c, inv, pd)
        cnt = np.bincount(inv, minlength=uniq.size)
        dsum = np.bincount(inv, weights=db[ok].astype(np.float64), minlength=uniq.size)
        return uniq, pd_c, dsum / cnt

    def observe_side(self, t: float, epoch: int, yaw: float, side: int, cells: np.ndarray,
                     pd_c: np.ndarray, db_c: np.ndarray, water_depth: float,
                     covered: bool) -> int:
        """MATH §5, §7 for one ping side. Returns the side's sector."""
        cfg, S = self.cfg, self.S
        k = sector_of(look_bearing(yaw, side), S, cfg.aspect.origin_deg)
        new = ((self.look_sector[cells] != k) | (self.look_epoch[cells] != epoch)
               | (t - self.look_last[cells] > cfg.look.timeout_s))
        if new.any():
            cn = cells[new]
            self.n_looks[cn] = np.minimum(self.n_looks[cn].astype(np.uint32) + 1, U16_MAX).astype(np.uint16)
            ai = cn * S + k
            self.aspect_counts[ai] = np.minimum(self.aspect_counts[ai].astype(np.uint16) + 1, 255).astype(np.uint8)
            self.look_sector[cn] = k
            self.look_epoch[cn] = epoch
            self.look_rho[cn] = self.rho_tab[k, self.searched[cn]]
            self.look_pd[cn] = 0.0
            self.look_cov[cn] = False
            self.lnq_open[cn] = self.lnq[cn]
            self.ell_open[cn] = self.ell[cn]
            self.look_last[cn] = t
        self.look_last[cells] = np.maximum(self.look_last[cells], t)
        if covered:
            old = self.look_pd[cells]
            best = np.maximum(old, pd_c)
            upd = (best > old) | (~self.look_cov[cells])
            if upd.any():
                cu, bu = cells[upd], best[upd]
                lnq = combine_miss(self.lnq_open[cu], self.look_rho[cu], bu, cfg.rho.ordering)
                self.lnq[cu] = lnq
                self.ell[cu] = self.ell_open[cu] + (lnq - self.lnq_open[cu]) + self.pfa_term
                self.look_pd[cu] = bu
                self.look_cov[cu] = True
                pos = cu[bu > 0.0]
                self.searched[pos] |= (1 << k)
            si = cells * S + k
            self.maxpd_s[si] = np.maximum(self.maxpd_s[si], pd_c)
        self.n_pings[cells] = np.minimum(self.n_pings[cells].astype(np.uint64) + 1, U32_MAX).astype(np.uint32)
        self.depth[cells] = np.float32(water_depth)
        self.intensity[cells] = db_c.astype(np.float32)
        return k

    # -------------------------------------------------------------------------- read-outs
    def p_background(self) -> np.ndarray:
        lo, hi = logit_scalar(self.cfg.clamp.p_min), logit_scalar(self.cfg.clamp.p_max)
        p = sigmoid(np.clip(self.ell, lo, hi)).astype(np.float32)
        p[~self.inside] = np.nan
        return p

    def coverage(self) -> np.ndarray:
        c = (-np.expm1(self.lnq)).astype(np.float32)
        c[~self.inside] = np.nan
        return c

    def n_aspects(self) -> np.ndarray:
        a = (self.aspect_counts.reshape(-1, self.S) > 0).sum(axis=1)
        return a.astype(np.uint8)

    def aspect_log_miss(self) -> np.ndarray:
        v = np.log1p(-self.maxpd_s).astype(np.float32)
        v.reshape(-1, self.S)[~self.inside, :] = np.nan
        return v

    def masked_f32(self, arr: np.ndarray) -> np.ndarray:
        out = np.asarray(arr, dtype=np.float32).copy()
        out[~self.inside] = np.nan
        return out
