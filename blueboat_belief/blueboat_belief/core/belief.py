"""The Belief facade: ingestion, event ordering, rendering, disc query (docs/MATH.md).

Inputs are plain dataclasses (``ros/adapters.py`` builds them from ROS messages, duck-typed).
Every time is an explicit float on the sonar-profile clock. No exception escapes on bad sensor
data: each rejection is counted by reason (``REASONS``).
"""

from __future__ import annotations

import bisect
import math
from collections import deque
from dataclasses import dataclass, field
from typing import Deque, Dict, List, Optional, Tuple

import numpy as np

from .alignment import Aligner
from .area import FORMAT_WORLD, SurveyArea, grid_for_area
from .config import BeliefConfig
from .contacts import ContactTable, valid_covariance
from .coverage import CoverageTimeline
from .geometry import PORT, STARBOARD, look_bearing, sector_of, side_of_point
from .grid import Grid
from .pose import FrozenPoseDetector, PoseBuffer
from .turn import TurnEstimator

REASONS = (
    "nan_input", "frozen_pose", "altitude_unknown", "time_reset", "no_pose_at_stamp",
    "outside_area", "bad_covariance", "alignment_pending", "alignment_queue_overflow",
    "no_fix", "ignored_class", "unknown_class", "duplicate_detection", "stale_detection",
    "future_detection", "zero_stamp_array", "coverage_expired", "empty_ping",
)


@dataclass(frozen=True)
class PingObs:
    t: float                     # port stamp, else starboard stamp [s]
    x: float                     # odom frame [m]
    y: float
    yaw: float                   # rad ENU
    water_depth: float
    port_ping_number: int
    starboard_ping_number: int
    port_y: np.ndarray           # float32, signed ground offsets (> 0), near -> far
    port_db: np.ndarray
    starboard_y: np.ndarray      # (< 0)
    starboard_db: np.ndarray


@dataclass(frozen=True)
class Det:
    id: str
    t_row: float
    x: float                     # odom frame [m]
    y: float
    theta: float                 # boat heading at the row (GCS compass) [rad]
    class_name: str
    score: float
    cov: np.ndarray              # (2, 2) m^2, odom frame
    size_x: float = float("nan")
    size_y: float = float("nan")


@dataclass(frozen=True)
class DetArray:
    t: float                     # image newest-row time [s]; 0 = unknown
    detections: Tuple[Det, ...] = ()


@dataclass(frozen=True)
class OdomObs:
    t: float
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class FixObs:
    t: float
    lat: float
    lon: float
    status: int


@dataclass(order=True)
class _Pending:
    t: float
    kind: int                    # 0 = ping (first on ties), 1 = detection
    seq: int
    obj: object = field(compare=False)
    stamp: float = field(compare=False, default=0.0)


class Belief:
    def __init__(self, cfg: BeliefConfig, area: SurveyArea) -> None:
        self.cfg = cfg
        self.area = area
        self.geom, self.inside = grid_for_area(area, cfg.grid.resolution_m)
        self.grid = Grid(cfg, self.geom, self.inside)
        self.contacts = ContactTable(cfg)
        self.aligner = Aligner(cfg.alignment, area.lat0, area.lon0,
                               expect_identity=(area.source_format == FORMAT_WORLD))
        tol, span = cfg.pose.stamp_tolerance_s, cfg.pose.buffer_s
        self.odom_buf = PoseBuffer(tol, span)
        self.ping_buf = PoseBuffer(tol, span)
        self.frozen_ping = FrozenPoseDetector(cfg.pose.frozen_count)
        self.frozen_odom = FrozenPoseDetector(cfg.pose.frozen_count)
        self.timeline = CoverageTimeline(cfg.coverage.max_array_gap_s)
        self.turn = TurnEstimator(cfg.turn)
        self.counters: Dict[str, int] = {r: 0 for r in REASONS}
        self.stats: Dict[str, int] = {"pings": 0, "pings_covered": 0, "pings_geometry_only": 0,
                                      "arrays": 0, "detections": 0, "contacts_created": 0,
                                      "fixes": 0, "odom": 0, "backscan_pings": 0,
                                      "backscan_samples": 0}
        self.epoch = 0
        self.newest_ping_t = -math.inf
        self.last_event_t = 0.0
        self._pending: List[_Pending] = []
        self._seq = 0
        self._queue: Deque[Tuple[str, object]] = deque()
        self._seen_det: set = set()
        self._was_valid = self.aligner.valid
        if self._was_valid:
            self.contacts.set_sigma_align(self.aligner.sigma)

    # ============================================================ ingestion (public)
    def ingest_odom(self, o: OdomObs) -> None:
        if not all(math.isfinite(v) for v in (o.t, o.x, o.y, o.yaw)):
            self._count("nan_input")
            return
        if self.frozen_odom.update(o.x, o.y):
            self._count("frozen_pose")
            return
        self.stats["odom"] += 1
        self.odom_buf.add(o.t, o.x, o.y, o.yaw)

    def ingest_fix(self, f: FixObs) -> None:
        if not math.isfinite(f.t):
            self._count("nan_input")
            return
        self.stats["fixes"] += 1
        reason = self.aligner.add_fix(f.t, f.lat, f.lon, int(f.status), self.odom_buf)
        if reason:
            self._count(reason)
        self._check_alignment()

    def ingest_ping(self, p: PingObs) -> None:
        if not all(math.isfinite(v) for v in (p.t, p.x, p.y, p.yaw, p.water_depth)):
            self._count("nan_input")
            return
        if self.frozen_ping.update(p.x, p.y):
            self._count("frozen_pose")
            return
        if p.t < self.newest_ping_t - self.cfg.look.time_reset_s:
            self._new_epoch()
        self.newest_ping_t = max(self.newest_ping_t, p.t)
        self.ping_buf.add(p.t, p.x, p.y, p.yaw)
        if abs(p.water_depth - self.cfg.ping.unknown_water_depth_m) <= self.cfg.ping.unknown_tol_m:
            self._count("altitude_unknown")
            return
        if p.port_ping_number == 0 and p.starboard_ping_number == 0:
            self._count("empty_ping")
            return
        if not self.aligner.valid:
            self._enqueue("ping", p)
            return
        self._accept_ping(p)

    def ingest_detections(self, a: DetArray) -> None:
        if not math.isfinite(a.t) or a.t <= 0.0:
            self._count("zero_stamp_array")
            return
        if not self.aligner.valid:
            self._enqueue("dets", a)
            return
        self._accept_dets(a)

    def finish(self) -> None:
        """End of input: release everything pending (coverage judged as is), close every look."""
        self._release(math.inf)
        self.contacts.close_all()

    # ============================================================ internals
    def _count(self, reason: str, n: int = 1) -> None:
        self.counters[reason] += n

    def _enqueue(self, kind: str, obj: object) -> None:
        self._count("alignment_pending")
        if len(self._queue) >= self.cfg.alignment.queue_max:
            self._queue.popleft()
            self._count("alignment_queue_overflow")
        self._queue.append((kind, obj))

    def _check_alignment(self) -> None:
        if self.aligner.valid and not self._was_valid:
            self._was_valid = True
            self.contacts.set_sigma_align(self.aligner.sigma)
            while self._queue:
                kind, obj = self._queue.popleft()
                if kind == "ping":
                    self._accept_ping(obj)          # type: ignore[arg-type]
                else:
                    self._accept_dets(obj)          # type: ignore[arg-type]

    def _new_epoch(self) -> None:
        self._count("time_reset")
        self._release(math.inf)
        self.contacts.close_all()
        self.timeline.clear()
        self.turn.clear()
        self.ping_buf.clear()
        self._seen_det.clear()
        self.epoch += 1
        self.newest_ping_t = -math.inf

    def _push(self, t: float, kind: int, obj: object, stamp: float = 0.0) -> None:
        self._seq += 1
        bisect.insort(self._pending, _Pending(t=t, kind=kind, seq=self._seq, obj=obj, stamp=stamp))

    def _accept_ping(self, p: PingObs) -> None:
        if not self.cfg.coverage.require_detector:
            self._process_ping(p, covered=True)
            return
        self._push(p.t, 0, p)
        self._expire(p.t)

    def _accept_dets(self, a: DetArray) -> None:
        self.stats["arrays"] += 1
        ignore = set(self.cfg.detector.ignore_classes)
        real = True
        for d in a.detections:
            if d.class_name in ignore:
                real = False
        if self.cfg.coverage.require_detector:
            self.timeline.add_array(a.t, real)
        for d in sorted(a.detections, key=lambda d: (d.t_row, d.id)):
            if d.class_name in ignore:
                self._count("ignored_class")
                continue
            if not all(math.isfinite(v) for v in (d.t_row, d.x, d.y, d.score)):
                self._count("nan_input")
                continue
            if d.t_row > a.t + 1e-6:
                self._count("future_detection")
                continue
            if d.t_row < self.newest_ping_t - (self.cfg.coverage.wait_s + self.cfg.look.timeout_s):
                self._count("stale_detection")
                continue
            key = (d.id, round(d.t_row, 6))
            if key in self._seen_det:
                self._count("duplicate_detection")
                continue
            self._seen_det.add(key)
            self._push(d.t_row, 1, d, stamp=a.t)
        if self.cfg.coverage.require_detector:
            self._release(self.timeline.release_bound)
            self._expire(a.t)
        else:
            self._release(math.inf)

    def _release(self, bound: float) -> None:
        while self._pending and self._pending[0].t <= bound:
            ev = self._pending.pop(0)
            self._dispatch(ev, gated=True)

    def _expire(self, now: float) -> None:
        limit = now - self.cfg.coverage.wait_s
        while self._pending and self._pending[0].t < limit:
            ev = self._pending.pop(0)
            if ev.kind == 0:
                self._count("coverage_expired")
            self._dispatch(ev, gated=True)

    def _dispatch(self, ev: _Pending, gated: bool) -> None:
        if ev.kind == 0:
            covered = self.timeline.covered(ev.t) if gated else True
            self._process_ping(ev.obj, covered)     # type: ignore[arg-type]
        else:
            self._process_det(ev.obj, ev.stamp)     # type: ignore[arg-type]

    # ------------------------------------------------------------ evidence
    def _contact_cells(self) -> Dict[int, int]:
        out: Dict[int, int] = {}
        for uid in sorted(self.contacts.contacts):
            c = self.contacts.contacts[uid]
            cell = int(self.geom.cell_of(c.x, c.y))
            if cell >= 0:
                out[uid] = cell
        return out

    def _process_ping(self, p: PingObs, covered: bool) -> None:
        self.stats["pings"] += 1
        self.stats["pings_covered" if covered else "pings_geometry_only"] += 1
        xm, ym = self.aligner.to_map(p.x, p.y)
        ccells = self._contact_cells() if self.contacts.contacts else {}
        cset = set(ccells.values())
        _, _, r_star, inner = self.turn.update(p.t, p.x, p.y, p.yaw)
        gate_turn = self.cfg.turn.enabled and inner != 0
        dropped_here = 0
        for side, yoff, db, present in ((PORT, p.port_y, p.port_db, p.port_ping_number != 0),
                                        (STARBOARD, p.starboard_y, p.starboard_db, p.starboard_ping_number != 0)):
            if not present:
                continue
            yoff = np.asarray(yoff, dtype=np.float64)
            db = np.asarray(db, dtype=np.float64)
            if yoff.size != db.size:
                self._count("nan_input")
                continue
            good = np.isfinite(yoff) & np.isfinite(db)
            if gate_turn and side == inner:
                back = np.abs(yoff) > r_star          # MATH §4.6: inner side beyond r* = v/|w|
                dropped_here += int(np.count_nonzero(back & good))
                good &= ~back
            if not good.all():
                yoff, db = yoff[good], db[good]
            res = self.grid.side_cells(xm, ym, p.yaw, yoff, db)
            if res is None:
                continue
            cells, pd_c, db_c = res
            k = self.grid.observe_side(p.t, self.epoch, p.yaw, side, cells, pd_c, db_c,
                                       p.water_depth, covered)
            if ccells:
                hit = {int(c): float(v) for c, v in zip(cells, pd_c) if int(c) in cset}
                if hit:
                    b = look_bearing(p.yaw, side)
                    self.contacts.observe(p.t, self.epoch, k, hit, ccells, covered, (xm, ym),
                                          math.degrees(b) % 360.0)
        if dropped_here:
            self.stats["backscan_pings"] += 1
            self.stats["backscan_samples"] += dropped_here
        self.contacts.close_due(p.t, self.epoch)
        self.last_event_t = max(self.last_event_t, p.t)

    def _fallback_cov(self, yaw: float, bearing: float, rng: float) -> np.ndarray:
        u = self.cfg.uncertainty
        Jb = rng * np.array([-math.sin(bearing), math.cos(bearing)])
        along = np.array([math.cos(yaw), math.sin(yaw)])
        across = np.array([math.cos(bearing), math.sin(bearing)])
        Rot = np.column_stack([along, across])
        D = np.diag([u.sigma_along_m ** 2, (u.sigma_across_m + u.across_per_range * rng) ** 2])
        return (u.sigma_pose_m ** 2) * np.eye(2) + np.outer(Jb, Jb) * math.radians(u.sigma_yaw_deg) ** 2 \
            + Rot @ D @ Rot.T

    def _process_det(self, d: Det, stamp: float) -> None:
        pose = self.ping_buf.at(d.t_row) or self.odom_buf.at(d.t_row)
        if pose is None:
            self._count("no_pose_at_stamp")
            return
        bx, by, yaw = pose
        zx, zy = self.aligner.to_map(d.x, d.y)
        cell = int(self.geom.cell_of(zx, zy))
        if cell < 0 or not self.inside[cell]:
            self._count("outside_area")
            return
        side = side_of_point(bx, by, yaw, d.x, d.y)
        b = look_bearing(yaw, side)
        k = sector_of(b, self.grid.S, self.cfg.aspect.origin_deg)
        rng = math.hypot(d.x - bx, d.y - by)
        R = np.asarray(d.cov, dtype=np.float64)
        if not valid_covariance(R):
            self._count("bad_covariance")
            R = self._fallback_cov(yaw, b, rng)
        if d.class_name not in self.contacts.class_index and self.contacts.other_idx < 0:
            self._count("unknown_class")
        det_id = f"{int(round(stamp * 1e9))}:{d.id}"
        c, created = self.contacts.add_detection(d.t_row, self.epoch, k, d.score,
                                                 np.array([zx, zy]), R, det_id, d.class_name,
                                                 math.degrees(b) % 360.0, rng)
        self.stats["detections"] += 1
        if created:
            self.stats["contacts_created"] += 1
        self.grid.max_conf[cell] = max(float(self.grid.max_conf[cell]), float(d.score))
        self.last_event_t = max(self.last_event_t, d.t_row)

    # ============================================================ read-outs
    def contact_mass_cells(self, c, cells_x: np.ndarray, cells_y: np.ndarray) -> np.ndarray:
        P = c.P
        inv = np.linalg.inv(P)
        dx, dy = cells_x - c.x, cells_y - c.y
        q = inv[0, 0] * dx * dx + 2 * inv[0, 1] * dx * dy + inv[1, 1] * dy * dy
        return np.exp(-0.5 * q) / (2.0 * math.pi * math.sqrt(np.linalg.det(P)))

    def render_p_target(self) -> np.ndarray:
        """MATH §11."""
        pbg = self.grid.p_background().astype(np.float64)
        keep = 1.0 - pbg
        g = self.geom
        for uid in sorted(self.contacts.contacts):
            c = self.contacts.contacts[uid]
            s = 4.0 * math.sqrt(max(np.linalg.eigvalsh(c.P)))
            c0 = max(0, int(math.floor((c.x - s - g.origin_x) / g.resolution)))
            c1 = min(g.width - 1, int(math.floor((c.x + s - g.origin_x) / g.resolution)))
            r0 = max(0, int(math.floor((c.y - s - g.origin_y) / g.resolution)))
            r1 = min(g.height - 1, int(math.floor((c.y + s - g.origin_y) / g.resolution)))
            if c0 > c1 or r0 > r1:
                continue
            cols = np.arange(c0, c1 + 1)
            rows = np.arange(r0, r1 + 1)
            cc, rr = np.meshgrid(cols, rows)
            idx = (rr * g.width + cc).ravel()
            cx = g.origin_x + (cc.ravel() + 0.5) * g.resolution
            cy = g.origin_y + (rr.ravel() + 0.5) * g.resolution
            w = self.contact_mass_cells(c, cx, cy) * g.resolution ** 2
            keep[idx] *= 1.0 - c.p_exists * np.minimum(w, 1.0)
        out = (1.0 - keep).astype(np.float32)
        out[~self.inside] = np.nan
        return out

    def contact_disc_mass(self, c, x0: float, y0: float, radius: float, step: float = 0.05) -> float:
        """MATH §12: midpoint quadrature of the contact Gaussian over the disc."""
        s = 5.0 * math.sqrt(max(np.linalg.eigvalsh(c.P)))
        xa, xb = max(x0 - radius, c.x - s), min(x0 + radius, c.x + s)
        ya, yb = max(y0 - radius, c.y - s), min(y0 + radius, c.y + s)
        if xa >= xb or ya >= yb:
            return 0.0
        nx = max(1, int(math.ceil((xb - xa) / step)))
        ny = max(1, int(math.ceil((yb - ya) / step)))
        hx, hy = (xb - xa) / nx, (yb - ya) / ny
        gx = xa + (np.arange(nx) + 0.5) * hx
        gy = ya + (np.arange(ny) + 0.5) * hy
        X, Y = np.meshgrid(gx, gy)
        X, Y = X.ravel(), Y.ravel()
        m = (X - x0) ** 2 + (Y - y0) ** 2 <= radius * radius
        if not m.any():
            return 0.0
        return float(min(1.0, self.contact_mass_cells(c, X[m], Y[m]).sum() * hx * hy))

    def query_disc(self, x0: float, y0: float, radius: float) -> dict:
        """MATH §12; backs ``QueryBelief``."""
        names = list(self.cfg.detector.class_names)
        if not (math.isfinite(x0) and math.isfinite(y0) and math.isfinite(radius)) or radius <= 0:
            return {"ok": False, "error": "x, y and radius_m must be finite, radius_m > 0"}
        cx, cy = self.geom.centres()
        sel = ((cx - x0) ** 2 + (cy - y0) ** 2 <= radius * radius) & self.inside
        pbg = self.grid.p_background().astype(np.float64)[sel]
        cov = self.grid.coverage().astype(np.float64)[sel]
        nasp = self.grid.n_aspects()[sel]
        nlk = self.grid.n_looks[sel]
        log_keep = float(np.log1p(-pbg).sum())
        expected = float(pbg.sum())
        cls = np.zeros(len(names))
        uids = []
        for uid in sorted(self.contacts.contacts):
            c = self.contacts.contacts[uid]
            m = self.contact_disc_mass(c, x0, y0, radius)
            if m <= 0.0:
                continue
            pm = c.p_exists * m
            log_keep += math.log1p(-min(pm, 1.0 - 1e-15))
            expected += pm
            cls += pm * (c.alpha / c.alpha.sum())
            if (c.x - x0) ** 2 + (c.y - y0) ** 2 <= radius * radius:
                uids.append(uid)
        tot = cls.sum()
        return {
            "ok": True, "error": "",
            "p_any": float(-math.expm1(log_keep)),
            "expected_count": expected,
            "coverage_mean": float(cov.mean()) if cov.size else float("nan"),
            "coverage_min": float(cov.min()) if cov.size else float("nan"),
            "observed_cells": int((nlk > 0).sum()),
            "total_cells": int(sel.sum()),
            "min_aspects": int(nasp.min()) if nasp.size else 0,
            "max_aspects": int(nasp.max()) if nasp.size else 0,
            "class_names": names,
            "class_distribution": (cls / tot if tot > 0 else cls).tolist(),
            "contact_uids": uids,
        }

    def area_cumulative_pd(self) -> float:
        """Bourgault 2006 Eq. 7 for one target placed uniformly in the survey area:
        P = 1 - sum_i pi_i Q_i with pi uniform over in-area cells = mean in-area coverage."""
        cov = self.grid.coverage()[self.inside]
        return float(cov.astype(np.float64).mean()) if cov.size else float("nan")

    def summary(self) -> Dict[str, float]:
        """Run-level numbers for /diagnostics and the snapshot manifest (not contract fields)."""
        cs = self.contacts.contacts.values()
        return {"area_cumulative_pd": self.area_cumulative_pd(),
                "contacts": len(self.contacts.contacts),
                "contacts_confirmed": sum(1 for c in cs if c.state == 1),
                "contacts_retired": self.contacts.retired,
                "alignment_valid": bool(self.aligner.valid)}

    def health(self) -> Tuple[str, str]:
        """(level, message) for /diagnostics: ERROR frozen pose; WARN alignment / sim check."""
        if self.frozen_ping.active or self.frozen_odom.active:
            return "ERROR", "pose frozen at (0, 0): /blueboat/odom reads zero"
        if not self.aligner.valid:
            return "WARN", f"odom->map not aligned yet ({self.aligner.n_pairs} fix pairs)"
        if self.aligner.sim_check_failed:
            return "WARN", "odom (0,0) is not the world anchor (sim cross-check)"
        if self.counters["alignment_queue_overflow"]:
            return "WARN", "observations dropped before alignment (queue overflow)"
        return "OK", "running"
