"""Positive evidence: Bernoulli-inspired contacts (docs/MATH.md §9, §10).

Not a multi-Bernoulli / RFS filter: nearest-neighbour association inside a 2 m AND chi-square
gate, no clutter model (brief §17.7). PRIORITY: while ``detector.calibrated`` is false the
score ``q`` is raw and ``p_exists`` is a score-derived index, not a probability.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

from .config import BeliefConfig
from .models import logit_scalar, pfa_log_factor, rho_table, sigmoid_scalar

STATE_CANDIDATE = 0
STATE_CONFIRMED = 1
SECTOR_UNKNOWN = 255


@dataclass
class Look:
    sector: int
    epoch: int
    t_first: float
    t_last: float
    rho: float
    L_open: float
    creation: bool = False
    detected: bool = False
    covered: bool = False
    pd_best: float = 0.0
    q_best: float = -1.0
    z: Optional[np.ndarray] = None          # best detection position (map) in this look
    R_ind: Optional[np.ndarray] = None      # its independent covariance part
    class_idx: int = -1
    bearing_deg: float = float("nan")
    range_m: float = float("nan")
    detection_ids: List[str] = field(default_factory=list)
    open: bool = True


@dataclass
class Contact:
    uid: int
    created: float
    updated: float
    L: float
    first_confidence: float
    first_sector: int
    max_confidence: float
    alpha: np.ndarray
    looks: List[Look] = field(default_factory=list)
    x: float = 0.0
    y: float = 0.0
    P: np.ndarray = field(default_factory=lambda: np.eye(2))
    state: int = STATE_CANDIDATE
    looked_mask: int = 0

    @property
    def p_exists(self) -> float:
        return sigmoid_scalar(self.L)

    @property
    def current(self) -> Optional[Look]:
        return self.looks[-1] if self.looks and self.looks[-1].open else None

    def n_detected(self) -> int:
        return sum(1 for lk in self.looks if lk.detected)


def valid_covariance(R: np.ndarray) -> bool:
    if R.shape != (2, 2) or not np.all(np.isfinite(R)):
        return False
    scale = max(1.0, float(np.max(np.abs(R))))
    if abs(R[0, 1] - R[1, 0]) > 1e-9 * scale:
        return False
    if float(np.trace(R)) <= 0.0:
        return False
    return float(np.min(np.linalg.eigvalsh(R))) >= -1e-12 * scale


class ContactTable:
    def __init__(self, cfg: BeliefConfig) -> None:
        self.cfg = cfg
        S = cfg.aspect.sector_count
        self.rho_tab = rho_table(S, cfg.aspect.origin_deg, cfg.rho.fold_opposite,
                                 cfg.rho.independence_angle_deg)
        self.logit_pi = logit_scalar(cfg.detector.base_rate)
        self.lo = logit_scalar(cfg.clamp.p_min)
        self.hi = logit_scalar(cfg.clamp.p_max)
        self.pfa_term = pfa_log_factor(cfg.pd.pfa)
        self.contacts: Dict[int, Contact] = {}
        self.next_uid = 1
        self.retired = 0
        self.class_index = {n: i for i, n in enumerate(cfg.detector.class_names)}
        self.other_idx = self.class_index.get("other", -1)
        self.sigma_align = 0.0

    # ------------------------------------------------------------------ helpers
    def _clamp(self, L: float) -> float:
        return min(max(L, self.lo), self.hi)

    def _new_look(self, c: Contact, t: float, epoch: int, sector: int) -> Look:
        rho = float(self.rho_tab[sector, c.looked_mask])
        lk = Look(sector=sector, epoch=epoch, t_first=t, t_last=t, rho=rho, L_open=c.L)
        c.looks.append(lk)
        return lk

    def _look_for(self, c: Contact, t: float, epoch: int, sector: int) -> Look:
        cur = c.current
        if cur is not None and cur.sector == sector and cur.epoch == epoch \
                and t - cur.t_last <= self.cfg.look.timeout_s:
            return cur
        if cur is not None:
            self._close(c, cur)
        return self._new_look(c, t, epoch, sector)

    def _close(self, c: Contact, lk: Look) -> None:
        if not lk.open:
            return
        lk.open = False
        if not lk.detected and lk.covered and lk.pd_best > 0.0:
            c.L = self._clamp(c.L + math.log1p(-lk.rho * lk.pd_best) + self.pfa_term)
        if lk.detected or (lk.covered and lk.pd_best > 0.0):
            c.looked_mask |= (1 << lk.sector)

    def _refuse(self, c: Contact) -> None:
        """Recompute the fused position and covariance from one observation per detected look."""
        cfg = self.cfg.contact
        lam = np.zeros((2, 2))
        eta = np.zeros(2)
        for lk in c.looks:
            if lk.detected and lk.z is not None:
                Ri = np.linalg.inv(lk.R_ind)
                lam += Ri
                eta += Ri @ lk.z
        cov = np.linalg.inv(lam)
        mean = cov @ eta
        common = cfg.common_mode_sigma_m ** 2 + cfg.position_floor_sigma_m ** 2 + self.sigma_align ** 2
        c.x, c.y = float(mean[0]), float(mean[1])
        c.P = cov + common * np.eye(2)

    def independent_part(self, R: np.ndarray) -> np.ndarray:
        cfg = self.cfg.contact
        w, V = np.linalg.eigh(R - (cfg.common_mode_sigma_m ** 2) * np.eye(2))
        w = np.maximum(w, cfg.min_indep_sigma_m ** 2)
        return (V * w) @ V.T

    def _update_state(self, c: Contact) -> None:
        if c.state == STATE_CONFIRMED:
            return
        det = [lk for lk in c.looks if lk.detected]
        if self.cfg.contact.confirm_distinct_sectors:
            ok = len({lk.sector for lk in det}) >= self.cfg.contact.confirm_looks
        else:
            ok = len(det) >= self.cfg.contact.confirm_looks
        if ok:
            c.state = STATE_CONFIRMED

    # ------------------------------------------------------------------ association
    def associate(self, z: np.ndarray, R: np.ndarray) -> Optional[Contact]:
        best, best_d2 = None, math.inf
        gate = self.cfg.association.gate_m
        for uid in sorted(self.contacts):
            c = self.contacts[uid]
            dx, dy = z[0] - c.x, z[1] - c.y
            if math.hypot(dx, dy) > gate:
                continue
            S = R + c.P
            d = np.array([dx, dy])
            d2 = float(d @ np.linalg.solve(S, d))
            if d2 <= self.cfg.association.chi2 and d2 < best_d2:
                best, best_d2 = c, d2
        return best

    # ------------------------------------------------------------------ updates
    def add_detection(self, t: float, epoch: int, sector: int, q: float, z: np.ndarray,
                      R: np.ndarray, det_id: str, class_name: str, bearing_deg: float,
                      range_m: float) -> Tuple[Contact, bool]:
        """MATH §9.1, §9.3, §9.4, §10. Returns (contact, created)."""
        q = min(max(float(q), self.cfg.clamp.p_min), self.cfg.clamp.p_max)
        R_ind = self.independent_part(R)
        cls = self.class_index.get(class_name, self.other_idx)
        c = self.associate(z, R)
        created = c is None
        if created:
            c = Contact(uid=self.next_uid, created=t, updated=t, L=self.logit_pi,
                        first_confidence=float(q), first_sector=int(sector),
                        max_confidence=float(q),
                        alpha=np.full(len(self.cfg.detector.class_names),
                                      self.cfg.contact.dirichlet_alpha0, dtype=np.float64),
                        x=float(z[0]), y=float(z[1]), P=R.copy())
            self.next_uid += 1
            self.contacts[c.uid] = c
            lk = self._new_look(c, t, epoch, sector)
            lk.creation = True
            lk.rho = 1.0
            lk.L_open = self.logit_pi
        else:
            lk = self._look_for(c, t, epoch, sector)
        lk.t_last = max(lk.t_last, t)
        lk.detection_ids.append(det_id)
        if not lk.detected or q > lk.q_best:
            lk.detected = True
            if lk.class_idx >= 0:
                c.alpha[lk.class_idx] -= lk.q_best
            lk.q_best = q
            lk.z = np.asarray(z, dtype=np.float64).copy()
            lk.R_ind = R_ind
            lk.class_idx = cls
            lk.bearing_deg = float(bearing_deg)
            lk.range_m = float(range_m)
            if cls >= 0:
                c.alpha[cls] += q
            c.L = self._clamp(lk.L_open + lk.rho * (logit_scalar(q) - self.logit_pi))
        c.max_confidence = max(c.max_confidence, q)
        c.updated = max(c.updated, t)
        self._refuse(c)
        self._update_state(c)
        if created:
            self._cap()
        return c, created

    def observe(self, t: float, epoch: int, sector: int, cell_pd: Dict[int, float],
                contact_cells: Dict[int, int], covered: bool, boat_xy: Tuple[float, float],
                bearing_deg: float) -> None:
        """A released ping side whose swath contains a contact's cell (MATH §9.2)."""
        for uid in sorted(contact_cells):
            cell = contact_cells[uid]
            if cell not in cell_pd:
                continue
            c = self.contacts[uid]
            lk = self._look_for(c, t, epoch, sector)
            lk.t_last = max(lk.t_last, t)
            if covered:
                lk.covered = True
                lk.pd_best = max(lk.pd_best, cell_pd[cell])
            if math.isnan(lk.range_m):
                lk.range_m = float(math.hypot(c.x - boat_xy[0], c.y - boat_xy[1]))
                lk.bearing_deg = bearing_deg

    def close_due(self, t: float, epoch: int) -> None:
        """Close looks that timed out or belong to an older epoch (MATH §9.6); prune."""
        for uid in sorted(self.contacts):
            c = self.contacts[uid]
            cur = c.current
            if cur is not None and (cur.epoch != epoch or t - cur.t_last > self.cfg.look.timeout_s):
                self._close(c, cur)
        self.prune()

    def close_all(self) -> None:
        for uid in sorted(self.contacts):
            c = self.contacts[uid]
            if c.current is not None:
                self._close(c, c.current)
        self.prune()

    def prune(self) -> None:
        cfg = self.cfg.contact
        for uid in sorted(self.contacts):
            c = self.contacts[uid]
            if c.state == STATE_CANDIDATE and c.current is None and c.p_exists < cfg.prune_p:
                del self.contacts[uid]
                self.retired += 1
        self._cap()

    def _cap(self) -> None:
        cfg = self.cfg.contact
        if len(self.contacts) > cfg.max_contacts:
            order = sorted(self.contacts.values(), key=lambda c: (c.p_exists, -c.uid))
            for c in order[: len(self.contacts) - cfg.max_contacts]:
                del self.contacts[c.uid]
                self.retired += 1

    def set_sigma_align(self, sigma: float) -> None:
        self.sigma_align = 0.0 if not math.isfinite(sigma) else float(sigma)
        for uid in sorted(self.contacts):
            self._refuse(self.contacts[uid])

    # ------------------------------------------------------------------ read-out
    def to_dicts(self, class_names: Sequence[str]) -> List[dict]:
        out = []
        for uid in sorted(self.contacts):
            c = self.contacts[uid]
            a = c.alpha / c.alpha.sum()
            det = sorted({lk.sector for lk in c.looks if lk.detected})
            miss = sorted({lk.sector for lk in c.looks
                           if not lk.open and not lk.detected and lk.covered and lk.pd_best > 0})
            ids: List[str] = []
            for lk in c.looks:
                ids.extend(lk.detection_ids)
            out.append({
                "uid": np.uint32(c.uid),
                "x": float(c.x), "y": float(c.y),
                "covariance": np.array([c.P[0, 0], c.P[0, 1], c.P[1, 0], c.P[1, 1]], dtype=np.float64),
                "p_exists": np.float32(c.p_exists),
                "state": np.uint8(c.state),
                "class_probabilities": a.astype(np.float32),
                "sectors_detected": np.asarray(det, dtype=np.uint8),
                "sectors_missed": np.asarray(miss, dtype=np.uint8),
                "n_looks_detected": np.uint16(min(c.n_detected(), 65535)),
                "first_confidence": np.float32(c.first_confidence),
                "first_sector": np.uint8(c.first_sector),
                "max_confidence": np.float32(c.max_confidence),
                "shadow_known": False,
                "has_shadow": False,
                "created": float(c.created),
                "updated": float(c.updated),
                "look_bearings_deg": np.asarray([lk.bearing_deg for lk in c.looks], dtype=np.float32),
                "look_ranges_m": np.asarray([lk.range_m for lk in c.looks], dtype=np.float32),
                "look_detected": np.asarray([lk.detected for lk in c.looks], dtype=bool),
                "detection_ids": ids,
            })
        return out
