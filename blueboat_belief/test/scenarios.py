"""Dev-only synthetic scenarios for the smoke test (owner decision Q-4).

In-code scenarios only: seeded numpy generators, never a ``ground_truth/*`` or ``/sim/*`` topic.
They imitate the *shape* of the real inputs:
- the processor's ping geometry (``y = ±(0.285 + sqrt(s² − h²))``, 1200 bins over 20 m);
- Perception's one array per image (stride ~ half an image);
- Perception's covariance with σ_nav = 1 m.

They are not a model of detector performance.
"""

from __future__ import annotations

import math
from typing import List, Optional, Sequence, Tuple

import numpy as np

from blueboat_belief.core.belief import Det, DetArray, FixObs, OdomObs, PingObs
from blueboat_belief.core.geo import local_en_to_latlon

LAT0, LON0 = 33.8900, 130.8800          # arbitrary datum (Kitakyushu bay)
LEVER = 0.285


def area_dict(xmin: float, ymin: float, xmax: float, ymax: float, lat0: float = LAT0,
              lon0: float = LON0, fmt: str = "blueboat_survey_area/1", **extra) -> dict:
    def gps(x, y):
        lat, lon = local_en_to_latlon(x, y, lat0, lon0)
        return [lat, lon]
    d = {"format": fmt, "geo_anchor": {"lat0": lat0, "lon0": lon0, "theta_deg": 0.0},
         "limits": {"local": {"x_min": xmin, "y_min": ymin, "x_max": xmax, "y_max": ymax},
                    "corners_gps": [gps(xmin, ymin), gps(xmax, ymin), gps(xmax, ymax), gps(xmin, ymax)]}}
    d["name" if fmt.endswith("survey_area/1") else "world_name"] = "synthetic"
    d.update(extra)
    return d


def side_offsets(h: float, side: int, n: int = 1200, length_m: float = 20.0) -> np.ndarray:
    """The processor's ground offsets for one side (``sss_helper.project_side``)."""
    slant = np.arange(n) / (n - 1) * length_m
    keep = slant > h
    g = np.sqrt(np.maximum(slant[keep] ** 2 - h * h, 0.0))
    return (side * (LEVER + g)).astype(np.float32)


def straight_pass(x0: float, y0: float, yaw: float, speed: float, duration: float,
                  t0: float = 100.0, rate: float = 20.0, h: float = 2.0,
                  pn0: int = 1, db: float = -20.0) -> List[PingObs]:
    py = side_offsets(h, +1)
    sy = side_offsets(h, -1)
    pdb = np.full(py.size, db, dtype=np.float32)
    sdb = np.full(sy.size, db, dtype=np.float32)
    out = []
    n = int(round(duration * rate))
    for i in range(n + 1):
        t = t0 + i / rate
        d = speed * (t - t0)
        out.append(PingObs(t=t, x=x0 + d * math.cos(yaw), y=y0 + d * math.sin(yaw), yaw=yaw,
                           water_depth=h + 0.22, port_ping_number=pn0 + i,
                           starboard_ping_number=pn0 + i, port_y=py, port_db=pdb,
                           starboard_y=sy, starboard_db=sdb))
    return out


def odom_for(pings: Sequence[PingObs]) -> List[OdomObs]:
    return [OdomObs(t=p.t, x=p.x, y=p.y, yaw=p.yaw) for p in pings]


def fixes_for(pings: Sequence[PingObs], offset: Tuple[float, float] = (0.0, 0.0),
              every: int = 4, noise_m: float = 0.0, seed: int = 1,
              lat0: float = LAT0, lon0: float = LON0, status: int = 0) -> List[FixObs]:
    """GPS fixes of the odom poses; ``offset`` = the map position of odom (0, 0)."""
    rng = np.random.default_rng(seed)
    out = []
    for p in pings[::every]:
        e = p.x + offset[0] + (rng.normal(0, noise_m) if noise_m else 0.0)
        n = p.y + offset[1] + (rng.normal(0, noise_m) if noise_m else 0.0)
        lat, lon = local_en_to_latlon(e, n, lat0, lon0)
        out.append(FixObs(t=p.t, lat=lat, lon=lon, status=status))
    return out


def arrays_for(pings: Sequence[PingObs], targets: Sequence[Tuple[float, float]],
               period_s: float = 2.0, sigma_m: float = 0.3, score: float = 0.8,
               cls: str = "block", seed: int = 7, visible=None, r_min: float = 3.0,
               r_max: float = 19.0, cov_sigma_nav: float = 1.0, image0: int = 0,
               offset: Tuple[float, float] = (0.0, 0.0)) -> List[DetArray]:
    """One array per image; image k covers (stamp - 2*period, stamp] (50 % overlap).

    Targets are in the **map** frame; ``offset`` maps odom to map (positions on the wire are
    odom-frame, as Perception's). A target is detected in every image whose window contains
    its broadside time when ``visible(i, ping)`` is true (default: always)."""
    rng = np.random.default_rng(seed)
    t_start, t_end = pings[0].t, pings[-1].t
    times = np.array([p.t for p in pings])
    out: List[DetArray] = []
    k = image0
    stamp = t_start + period_s
    while stamp <= t_end + 1e-9:
        lo = stamp - 2.0 * period_s
        dets = []
        win = [p for p in pings if lo < p.t <= stamp]
        for j, (tx, ty) in enumerate(targets):
            ox, oy = tx - offset[0], ty - offset[1]
            # broadside within this image: the window's ping closest to abeam (|along| < 0.5 m)
            p_best, best = None, 0.5
            for p in win:
                along = (ox - p.x) * math.cos(p.yaw) + (oy - p.y) * math.sin(p.yaw)
                if abs(along) < best:
                    best, p_best = abs(along), p
            if p_best is None:
                continue
            rng_m = math.hypot(ox - p_best.x, oy - p_best.y) - LEVER
            if not (r_min <= rng_m <= r_max):
                continue
            if visible is not None and not visible(j, p_best):
                continue
            n_x, n_y = rng.normal(0.0, sigma_m, 2)
            v = cov_sigma_nav ** 2 + sigma_m ** 2
            dets.append(Det(id=str(2000000 + 16 * k + len(dets)), t_row=p_best.t,
                            x=ox + n_x, y=oy + n_y, theta=p_best.yaw, class_name=cls,
                            score=score, cov=np.array([[v, 0.0], [0.0, v]])))
        out.append(DetArray(t=float(times[np.searchsorted(times, stamp, side="right") - 1]),
                            detections=tuple(dets)))
        k += 1
        stamp += period_s
    return out


def events(pings, odom=(), fixes=(), arrays=(), array_latency_s: float = 1.0) -> list:
    """Arrival-ordered events: (arrival time, order, kind, obj); arrays arrive after their stamp."""
    ev = []
    for o in odom:
        ev.append((o.t, 0, "odom", o))
    for f in fixes:
        ev.append((f.t, 1, "fix", f))
    for p in pings:
        ev.append((p.t + 0.01, 2, "ping", p))
    for a in arrays:
        ev.append((a.t + array_latency_s, 3, "dets", a))
    ev.sort(key=lambda e: (e[0], e[1]))
    return ev


def feed(belief, evs) -> None:
    for _, _, kind, obj in evs:
        if kind == "odom":
            belief.ingest_odom(obj)
        elif kind == "fix":
            belief.ingest_fix(obj)
        elif kind == "ping":
            belief.ingest_ping(obj)
        else:
            belief.ingest_detections(obj)


def lawn(x0: float, y0: float, legs: int, spacing: float, length: float, speed: float = 1.0,
         t0: float = 100.0, gap_s: float = 5.0, yaw0: float = 0.0, pn0: int = 1) -> List[PingObs]:
    """Boustrophedon: ``legs`` parallel legs along ``yaw0``, ``spacing`` apart (to the left)."""
    out: List[PingObs] = []
    t = t0
    pn = pn0
    for i in range(legs):
        yaw = yaw0 if i % 2 == 0 else yaw0 + math.pi
        ox = x0 - i * spacing * math.sin(yaw0) + (0 if i % 2 == 0 else length * math.cos(yaw0))
        oy = y0 + i * spacing * math.cos(yaw0) + (0 if i % 2 == 0 else length * math.sin(yaw0))
        leg = straight_pass(ox, oy, yaw, speed, length / speed, t0=t, pn0=pn)
        out.extend(leg)
        t = leg[-1].t + gap_s
        pn = leg[-1].port_ping_number + 1
    return out
