"""Look geometry (docs/MATH.md §1; contract belief_grid/0 C-5).

ENU, yaw psi in radians CCW from East; side s = +1 port, -1 starboard; look bearing
b = psi + s*pi/2; the processor's signed ground offsets y (port > 0) map to
p = (x - y sin psi, y + y cos psi) == p + |y| (cos b, sin b).
"""

from __future__ import annotations

import math

import numpy as np

PORT = 1
STARBOARD = -1
SECTOR_ROUND_DECIMALS = 9          # degrees rounded to 1e-9 before the sector floor


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    """ENU yaw of a quaternion (full or yaw-only)."""
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def look_bearing(yaw: float, side: int) -> float:
    return yaw + side * (math.pi / 2.0)


def sector_of(bearing_rad, n: int, origin_deg: float):
    """k = floor(mod(deg(b) - origin, 360) / (360/n)), on the directed bearing.

    The angle is rounded to 1e-9 deg first (``SECTOR_ROUND_DECIMALS``), so a bearing on a sector edge
    (e.g. radians(30) -> 29.999999999999996 deg) lands in the upper sector (docs/topics.md)."""
    b = np.asarray(bearing_rad, dtype=np.float64)
    a = np.mod(np.round(np.degrees(b) - origin_deg, SECTOR_ROUND_DECIMALS), 360.0)
    k = np.floor(a / (360.0 / n)).astype(np.int64)
    k = np.clip(k, 0, n - 1)          # a == 360 - eps rounding
    return int(k) if k.ndim == 0 else k


def sample_xy(x: float, y: float, yaw: float, yoff: np.ndarray):
    """Map the processor's signed ground offsets to positions (same frame as x, y)."""
    yoff = np.asarray(yoff, dtype=np.float64)
    s, c = math.sin(yaw), math.cos(yaw)
    return x - yoff * s, y + yoff * c


def side_of_point(x: float, y: float, yaw: float, px: float, py: float) -> int:
    """+1 (port) if the point is left of the heading, else -1 (starboard). On the line: port."""
    cross = math.cos(yaw) * (py - y) - math.sin(yaw) * (px - x)
    return PORT if cross >= 0.0 else STARBOARD


def wrap_pi(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi
