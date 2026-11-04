"""The project's single equirectangular pair -- an own copy (CM-3 forbids importing a neighbour).

Asserted bit-identical to ``BlueBoat-MCS/mcs/core/geo.py`` (``latlon_to_local_en`` /
``local_en_to_latlon``, ``EARTH_RADIUS_M``) by ``test/smoke_test.py [1]``. Do not "improve"
the expression order: bit-identity is the contract (root CLAUDE.md, brief §3.3).
"""

from __future__ import annotations

import math

EARTH_RADIUS_M = 6378137.0


def latlon_to_local_en(lat: float, lon: float, lat0: float, lon0: float) -> tuple[float, float]:
    east = math.radians(lon - lon0) * EARTH_RADIUS_M * math.cos(math.radians(lat0))
    north = math.radians(lat - lat0) * EARTH_RADIUS_M
    return east, north


def local_en_to_latlon(east: float, north: float, lat0: float, lon0: float) -> tuple[float, float]:
    lat = lat0 + math.degrees(north / EARTH_RADIUS_M)
    lon = lon0 + math.degrees(east / (EARTH_RADIUS_M * math.cos(math.radians(lat0))))
    return lat, lon
