"""Sensor and dependence models (docs/MATH.md §2, §4, §6, §7). All PLACEHOLDERS until fitted.

Nothing here is a measured performance claim: ``pd_range`` is the brief's placeholder window,
``rho_table`` is Paull et al. 2013's linear-in-angle *assumption*, and PFA is neglected by
default (the Bourgault miss update is the PFA = 0 special case).
"""

from __future__ import annotations

import math

import numpy as np

from .config import PdCfg, PriorCfg


def logit(p):
    p = np.asarray(p, dtype=np.float64)
    return np.log(p) - np.log1p(-p)


def sigmoid(l):
    l = np.asarray(l, dtype=np.float64)
    return np.where(l >= 0, 1.0 / (1.0 + np.exp(-np.abs(l))), np.exp(-np.abs(l)) / (1.0 + np.exp(-np.abs(l))))


def logit_scalar(p: float) -> float:
    return math.log(p) - math.log1p(-p)


def sigmoid_scalar(l: float) -> float:
    if l >= 0:
        return 1.0 / (1.0 + math.exp(-l))
    e = math.exp(l)
    return e / (1.0 + e)


def prior_cell(cfg: PriorCfg, resolution_m: float) -> float:
    """MATH §2: p0 = lambda (rare-event approximation) or 1 - exp(-lambda)."""
    lam = cfg.density_per_ha * resolution_m * resolution_m / 1.0e4
    return -math.expm1(-lam) if cfg.exact_poisson else lam


def pd_range(r, cfg: PdCfg) -> np.ndarray:
    """MATH §4: PLACEHOLDER trapezoid, plateau on [r_lo, r_hi], linear ramps of width w outside."""
    r = np.asarray(r, dtype=np.float64)
    w = cfg.ramp_m
    up = np.clip((r - (cfg.r_lo_m - w)) / w, 0.0, 1.0)
    down = np.clip(((cfg.r_hi_m + w) - r) / w, 0.0, 1.0)
    return cfg.plateau * up * down


def sector_centres_deg(n: int, origin_deg: float) -> np.ndarray:
    return origin_deg + (np.arange(n) + 0.5) * (360.0 / n)


def sector_angle_deg(n: int, origin_deg: float, fold_opposite: bool) -> np.ndarray:
    """(n, n) angle between sector centres: directed in [0, 180], or folded line angle in [0, 90]."""
    c = sector_centres_deg(n, origin_deg)
    d = np.abs(c[:, None] - c[None, :]) % 360.0
    d = np.minimum(d, 360.0 - d)
    if fold_opposite:
        d = np.minimum(d, 180.0 - d)
    return d


def rho_table(n: int, origin_deg: float, fold_opposite: bool, theta_deg: float) -> np.ndarray:
    """MATH §6: table[k, mask] = min over set bits k' of clip(Delta(k, k')/theta, 0, 1); mask 0 -> 1.

    ``mask`` is the bitmask of sectors already searched (bit k' = sector k')."""
    pair = np.clip(sector_angle_deg(n, origin_deg, fold_opposite) / theta_deg, 0.0, 1.0)
    table = np.ones((n, 1 << n), dtype=np.float64)
    for mask in range(1, 1 << n):
        low = mask & -mask
        k2 = low.bit_length() - 1
        table[:, mask] = np.minimum(table[:, mask ^ low], pair[:, k2])
    return table


def combine_miss(lnq_open, rho, pd, ordering: str) -> np.ndarray:
    """MATH §7: ln Q after a look with dependence rho and best covered PD, from ln Q at look open."""
    lnq_open = np.asarray(lnq_open, dtype=np.float64)
    rho = np.asarray(rho, dtype=np.float64)
    pd = np.asarray(pd, dtype=np.float64)
    if ordering == "sequential":
        return lnq_open + np.log1p(-rho * pd)
    if ordering == "paull_pairwise":
        c_acc = -np.expm1(lnq_open)
        c_hi = np.maximum(c_acc, pd)
        c_lo = np.minimum(c_acc, pd)
        return np.log1p(-c_hi) + np.log1p(-rho * c_lo)
    raise ValueError(f"unknown ordering {ordering!r}")


def pfa_log_factor(pfa: float) -> float:
    """-ln(1 - PFA): the false-alarm denominator of the non-detection likelihood ratio."""
    return -math.log1p(-pfa)
