"""Read the survey area (docs/SURVEY_AREA_FORMAT.md): a world folder / ``metadata.yaml`` or a
``blueboat_survey_area/1`` file. Read-only (CM-7); PyYAML imported lazily.

``mission_rectangle`` reads a BlueBoat-Replanner mission file for ``blueboat-belief-make-area
--from-mission``, so that Belief's area is derived from the Replanner's polygon instead of being
typed a second time. It is a file-level handoff: no Replanner code is imported.
"""

from __future__ import annotations

import math
from pathlib import Path
from typing import Optional, Sequence, Tuple

from ..core.area import AreaError, SurveyArea, area_from_dict
from ..core.geo import latlon_to_local_en


def load_area_file(path: str) -> SurveyArea:
    if not str(path).strip():
        raise AreaError("no survey area given: set area.file (launch argument area_file:=<world "
                        "folder | metadata.yaml | survey-area yaml>)")
    p = Path(path).expanduser()
    if p.is_dir():
        p = p / "metadata.yaml"
    if not p.is_file():
        raise AreaError(f"survey area file not found: {p}")
    import yaml  # noqa: PLC0415

    with open(p, "r", encoding="utf-8") as fh:
        data = yaml.safe_load(fh)
    return area_from_dict(data, source=str(p))


def mission_rectangle(path: str, anchor: Optional[Sequence[float]] = None,
                      margin_m: float = 0.0) -> Tuple[float, float, float, float, float, float, str]:
    """A Replanner mission file -> ``(lat0, lon0, x_min, y_min, x_max, y_max, name)``.

    The file is ``mission: {name, areas: [[[lat, lon], ...]], ...}`` with exactly one WGS-84
    polygon (BlueBoat-Replanner ``config/mission_example.yaml``); every other key is ignored.
    The rectangle is the polygon's bounding box in ENU metres about the anchor, grown by
    ``margin_m`` and rounded outwards to 1 cm. The anchor is ``anchor`` if given, else the
    bounding box's south-west corner."""
    p = Path(path).expanduser()
    if not p.is_file():
        raise AreaError(f"mission file not found: {p}")
    import yaml  # noqa: PLC0415

    with open(p, "r", encoding="utf-8") as fh:
        data = yaml.safe_load(fh)
    mission = data.get("mission") if isinstance(data, dict) else None
    if not isinstance(mission, dict):
        raise AreaError(f"{p}: top-level key 'mission' missing")
    areas = mission.get("areas")
    if not isinstance(areas, (list, tuple)) or len(areas) != 1:
        raise AreaError(f"{p}: mission.areas must hold exactly one polygon")
    poly = areas[0]
    if not isinstance(poly, (list, tuple)) or len(poly) < 3:
        raise AreaError(f"{p}: mission.areas[0] must list at least 3 [lat, lon] vertices")
    pts = []
    for k, v in enumerate(poly):
        ok = isinstance(v, (list, tuple)) and len(v) == 2 and all(
            isinstance(c, (int, float)) and not isinstance(c, bool) and math.isfinite(float(c)) for c in v)
        if not ok:
            raise AreaError(f"{p}: mission.areas[0][{k}] must be [lat, lon] (finite numbers)")
        pts.append((float(v[0]), float(v[1])))
    if isinstance(margin_m, bool) or not math.isfinite(float(margin_m)) or margin_m < 0:
        raise AreaError("margin must be a finite number >= 0")
    if anchor is None:
        lat0, lon0 = min(a for a, _ in pts), min(b for _, b in pts)
    else:
        lat0, lon0 = float(anchor[0]), float(anchor[1])
    en = [latlon_to_local_en(lat, lon, lat0, lon0) for lat, lon in pts]
    cm = 100.0
    x_min = math.floor((min(e for e, _ in en) - margin_m) * cm) / cm
    y_min = math.floor((min(n for _, n in en) - margin_m) * cm) / cm
    x_max = math.ceil((max(e for e, _ in en) + margin_m) * cm) / cm
    y_max = math.ceil((max(n for _, n in en) + margin_m) * cm) / cm
    return lat0, lon0, x_min, y_min, x_max, y_max, str(mission.get("name", ""))
