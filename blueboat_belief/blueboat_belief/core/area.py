"""Survey area and grid geometry (docs/SURVEY_AREA_FORMAT.md; MATH §1.6).

Two accepted inputs, both parsed from a plain dict (file reading lives in ``io/missions.py``):

* a simulator world's ``metadata.yaml`` (``format: blueboat_world_meta/1``) -- only ``format``,
  ``world_name``, ``geo_anchor`` and ``limits`` are read; ground truth (``objects``) never is;
* a survey-area file (``format: blueboat_survey_area/1``) -- the same keys, strict.

The map datum is the area's ``geo_anchor``. The polygon (map frame, metres) is the
``limits.local`` rectangle. ``limits.corners_gps``, converted through ``core.geo``, must agree
with it within ``CORNER_TOL_M``, which cross-checks the two descriptions of one area.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Mapping, Tuple

import numpy as np

from .geo import latlon_to_local_en

FORMAT_WORLD = "blueboat_world_meta/1"
FORMAT_AREA = "blueboat_survey_area/1"
CORNER_TOL_M = 0.05
MIN_AREA_M2 = 1.0
_AREA_KEYS = {"format", "name", "created", "notes", "geo_anchor", "limits"}
_CORNERS = ("SW", "SE", "NE", "NW")


class AreaError(ValueError):
    """A survey-area description is invalid; the message names the offending item."""


@dataclass(frozen=True)
class SurveyArea:
    name: str
    source_format: str
    lat0: float                                   # datum (geo_anchor) [deg]
    lon0: float
    local: Tuple[float, float, float, float]      # x_min, y_min, x_max, y_max [m]
    polygon: Tuple[Tuple[float, float], ...]      # map frame, counter-clockwise, not closed
    corners_gps: Tuple[Tuple[float, float], ...]  # [lat, lon] SW, SE, NE, NW

    @property
    def polygon_xy(self) -> np.ndarray:
        return np.asarray(self.polygon, dtype=np.float64)


def _num(v: Any, where: str) -> float:
    if isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(float(v)):
        raise AreaError(f"{where}: expected a finite number, got {v!r}")
    return float(v)


def _polygon_area(poly: np.ndarray) -> float:
    x, y = poly[:, 0], poly[:, 1]
    return 0.5 * float(np.dot(x, np.roll(y, -1)) - np.dot(np.roll(x, -1), y))


def _segments_cross(p1, p2, p3, p4) -> bool:
    def orient(a, b, c):
        return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])
    d1, d2 = orient(p3, p4, p1), orient(p3, p4, p2)
    d3, d4 = orient(p1, p2, p3), orient(p1, p2, p4)
    return (d1 * d2 < 0) and (d3 * d4 < 0)


def _is_simple(poly: np.ndarray) -> bool:
    n = len(poly)
    for i in range(n):
        for j in range(i + 1, n):
            if abs(i - j) in (1, n - 1):
                continue
            if _segments_cross(poly[i], poly[(i + 1) % n], poly[j], poly[(j + 1) % n]):
                return False
    return True


def area_from_dict(d: Mapping[str, Any], source: str = "<dict>") -> SurveyArea:
    """Validate a world ``metadata.yaml`` or a ``blueboat_survey_area/1`` mapping."""
    if not isinstance(d, Mapping):
        raise AreaError(f"{source}: top level must be a mapping")
    fmt = d.get("format")
    if fmt == FORMAT_AREA:
        unknown = set(d) - _AREA_KEYS
        if unknown:
            raise AreaError(f"{source}: unknown keys {sorted(unknown)}")
        name = str(d.get("name", ""))
    elif fmt == FORMAT_WORLD:
        name = str(d.get("world_name", ""))           # every other world key is ignored on purpose
    else:
        raise AreaError(f"{source}: format must be {FORMAT_AREA!r} or {FORMAT_WORLD!r}, got {fmt!r}")

    ga = d.get("geo_anchor")
    if not isinstance(ga, Mapping):
        raise AreaError(f"{source}: geo_anchor missing")
    lat0 = _num(ga.get("lat0"), f"{source}: geo_anchor.lat0")
    lon0 = _num(ga.get("lon0"), f"{source}: geo_anchor.lon0")
    theta = _num(ga.get("theta_deg", 0.0), f"{source}: geo_anchor.theta_deg")
    if theta != 0.0:
        raise AreaError(f"{source}: geo_anchor.theta_deg must be 0 (the map frame is the anchor's ENU)")
    if not (-90.0 < lat0 < 90.0 and -180.0 <= lon0 <= 180.0):
        raise AreaError(f"{source}: geo_anchor out of range")

    lim = d.get("limits")
    if not isinstance(lim, Mapping) or not isinstance(lim.get("local"), Mapping):
        raise AreaError(f"{source}: limits.local missing")
    loc = lim["local"]
    xmin, ymin = _num(loc.get("x_min"), f"{source}: limits.local.x_min"), _num(loc.get("y_min"), f"{source}: limits.local.y_min")
    xmax, ymax = _num(loc.get("x_max"), f"{source}: limits.local.x_max"), _num(loc.get("y_max"), f"{source}: limits.local.y_max")
    if not (xmin < xmax and ymin < ymax):
        raise AreaError(f"{source}: limits.local must have x_min < x_max and y_min < y_max")

    cg = lim.get("corners_gps")
    if not isinstance(cg, (list, tuple)) or len(cg) != 4:
        raise AreaError(f"{source}: limits.corners_gps must list 4 corners [SW, SE, NE, NW]")
    expect = ((xmin, ymin), (xmax, ymin), (xmax, ymax), (xmin, ymax))
    poly, corners = [], []
    for k, (c, label, (ex, ey)) in enumerate(zip(cg, _CORNERS, expect)):
        if not isinstance(c, (list, tuple)) or len(c) != 2:
            raise AreaError(f"{source}: corners_gps[{k}] ({label}) must be [lat, lon]")
        lat = _num(c[0], f"{source}: corners_gps[{k}] ({label}) lat")
        lon = _num(c[1], f"{source}: corners_gps[{k}] ({label}) lon")
        e, n = latlon_to_local_en(lat, lon, lat0, lon0)
        if math.hypot(e - ex, n - ey) > CORNER_TOL_M:
            raise AreaError(
                f"{source}: corners_gps {label} is {math.hypot(e - ex, n - ey):.3f} m from "
                f"limits.local (tolerance {CORNER_TOL_M} m)")
        poly.append((ex, ey))                    # exact local frame; GPS is the cross-check
        corners.append((lat, lon))

    p = np.asarray(poly, dtype=np.float64)
    area = _polygon_area(p)
    if area < 0:                                      # store counter-clockwise
        p = p[::-1]
        area = -area
    if area < MIN_AREA_M2:
        raise AreaError(f"{source}: area {area:.3f} m^2 below {MIN_AREA_M2} m^2")
    if not _is_simple(p):
        raise AreaError(f"{source}: polygon is not simple")
    return SurveyArea(name=name, source_format=fmt, lat0=lat0, lon0=lon0,
                      local=(xmin, ymin, xmax, ymax),
                      polygon=tuple((float(a), float(b)) for a, b in p),
                      corners_gps=tuple(corners))


def make_area_dict(lat0: float, lon0: float, x_min: float, y_min: float, x_max: float,
                   y_max: float, name: str = "", created: str = "") -> dict:
    """A ``blueboat_survey_area/1`` mapping for a rectangle about ``geo_anchor``.

    ``corners_gps`` (SW, SE, NE, NW) are computed with the project pair. The result is validated
    by ``area_from_dict`` before return."""
    from .geo import local_en_to_latlon

    corners = [list(local_en_to_latlon(x, y, lat0, lon0))
               for x, y in ((x_min, y_min), (x_max, y_min), (x_max, y_max), (x_min, y_max))]
    d = {"format": FORMAT_AREA, "name": str(name),
         "geo_anchor": {"lat0": float(lat0), "lon0": float(lon0), "theta_deg": 0.0},
         "limits": {"local": {"x_min": float(x_min), "y_min": float(y_min),
                              "x_max": float(x_max), "y_max": float(y_max)},
                    "corners_gps": corners}}
    if created:
        d["created"] = str(created)
    area_from_dict(d, source="make_area_dict")
    return d


def points_in_polygon(x: np.ndarray, y: np.ndarray, poly: np.ndarray) -> np.ndarray:
    """Even-odd rule, vectorised over points. Points exactly on an edge may go either way."""
    x = np.asarray(x, dtype=np.float64)
    y = np.asarray(y, dtype=np.float64)
    inside = np.zeros(np.broadcast(x, y).shape, dtype=bool)
    n = len(poly)
    for i in range(n):
        x1, y1 = poly[i]
        x2, y2 = poly[(i + 1) % n]
        if y1 == y2:
            continue
        cond = (y1 > y) != (y2 > y)
        xint = x1 + (y - y1) * (x2 - x1) / (y2 - y1)
        inside ^= cond & (x < xint)
    return inside


@dataclass(frozen=True)
class GridGeometry:
    origin_x: float          # lower-left corner of cell (0, 0), map frame [m]
    origin_y: float
    width: int
    height: int
    resolution: float

    @property
    def n_cells(self) -> int:
        return self.width * self.height

    def cell_of(self, x, y) -> np.ndarray:
        """Flat cell index of map points; -1 outside the grid rectangle."""
        col = np.floor((np.asarray(x, dtype=np.float64) - self.origin_x) / self.resolution)
        row = np.floor((np.asarray(y, dtype=np.float64) - self.origin_y) / self.resolution)
        ok = (col >= 0) & (col < self.width) & (row >= 0) & (row < self.height)
        idx = np.where(ok, row * self.width + col, -1)
        return idx.astype(np.int64)

    def centres(self) -> Tuple[np.ndarray, np.ndarray]:
        cols = self.origin_x + (np.arange(self.width) + 0.5) * self.resolution
        rows = self.origin_y + (np.arange(self.height) + 0.5) * self.resolution
        cx, cy = np.meshgrid(cols, rows)
        return cx.ravel(), cy.ravel()


def grid_for_area(area: SurveyArea, resolution: float) -> Tuple[GridGeometry, np.ndarray]:
    """The area's bounding box at ``resolution``; returns (geometry, inside-mask per cell)."""
    p = area.polygon_xy
    x0, y0 = float(p[:, 0].min()), float(p[:, 1].min())
    x1, y1 = float(p[:, 0].max()), float(p[:, 1].max())
    width = max(1, int(math.ceil((x1 - x0) / resolution - 1e-9)))
    height = max(1, int(math.ceil((y1 - y0) / resolution - 1e-9)))
    geom = GridGeometry(origin_x=x0, origin_y=y0, width=width, height=height,
                        resolution=float(resolution))
    cx, cy = geom.centres()
    return geom, points_in_polygon(cx, cy, p)
