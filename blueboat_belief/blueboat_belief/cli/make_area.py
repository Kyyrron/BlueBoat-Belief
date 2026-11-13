"""blueboat-belief-make-area: write a survey-area file (docs/SURVEY_AREA_FORMAT.md).

A minimal first version of the configuration tool (owner decision, 2026-11-04). It writes a
validated ``blueboat_survey_area/1`` file, either:

- from a rectangle about an anchor:
  ``--anchor LAT LON --local X_MIN Y_MIN X_MAX Y_MAX`` (metres, ENU about the anchor); or
- from a simulator world: ``--from-world <world folder | metadata.yaml>``, which copies its
  ``geo_anchor`` and ``limits``. Ground truth (``objects``) is not read; or
- from a BlueBoat-Replanner mission file: ``--from-mission <mission.yaml> [--margin-m M]
  [--anchor LAT LON]``. The area is the bounding rectangle of the mission's single WGS-84
  polygon, so the two modules describe the same area from one file.

The output is created exclusively (O_EXCL): an existing file is never overwritten (CM-7).
"""

from __future__ import annotations

import argparse
import datetime as _dt
import os
import sys
from pathlib import Path
from typing import List, Optional

from ..core.area import AreaError, make_area_dict
from ..io.missions import load_area_file, mission_rectangle


def build(args) -> dict:
    created = args.created or _dt.datetime.now(_dt.timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")
    if args.from_world and args.from_mission:
        raise AreaError("give --from-world or --from-mission, not both")
    if args.from_world:
        w = load_area_file(args.from_world)
        x0, y0, x1, y1 = w.local
        return make_area_dict(w.lat0, w.lon0, x0, y0, x1, y1, name=args.name or w.name, created=created)
    if args.from_mission:
        lat0, lon0, x0, y0, x1, y1, name = mission_rectangle(args.from_mission, args.anchor, args.margin_m)
        return make_area_dict(lat0, lon0, x0, y0, x1, y1, name=args.name or name, created=created)
    if args.anchor is None or args.local is None:
        raise AreaError("give --anchor LAT LON and --local X_MIN Y_MIN X_MAX Y_MAX, "
                        "or --from-world, or --from-mission")
    lat0, lon0 = args.anchor
    x0, y0, x1, y1 = args.local
    return make_area_dict(lat0, lon0, x0, y0, x1, y1, name=args.name, created=created)


def write_exclusive(path: Path, d: dict) -> None:
    import yaml  # noqa: PLC0415 - lazy

    text = ("# Survey area for BlueBoat-Belief (docs/SURVEY_AREA_FORMAT.md); written by "
            "blueboat-belief-make-area.\n" + yaml.safe_dump(d, sort_keys=False, default_flow_style=None))
    fd = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o644)
    with os.fdopen(fd, "w", encoding="utf-8") as fh:
        fh.write(text)


def main(argv: Optional[List[str]] = None) -> int:
    ap = argparse.ArgumentParser(prog="blueboat-belief-make-area", description=__doc__.splitlines()[0])
    ap.add_argument("--out", required=True, help="file to create (never overwritten)")
    ap.add_argument("--name", default="")
    ap.add_argument("--created", default="", help="ISO timestamp (default: now, UTC)")
    ap.add_argument("--anchor", nargs=2, type=float, metavar=("LAT", "LON"))
    ap.add_argument("--local", nargs=4, type=float, metavar=("X_MIN", "Y_MIN", "X_MAX", "Y_MAX"))
    ap.add_argument("--from-world", default="", help="world folder or metadata.yaml")
    ap.add_argument("--from-mission", default="", help="BlueBoat-Replanner mission file (one WGS-84 polygon)")
    ap.add_argument("--margin-m", type=float, default=0.0,
                    help="with --from-mission: grow the bounding rectangle by this many metres")
    args = ap.parse_args(argv)
    try:
        d = build(args)
        write_exclusive(Path(args.out), d)
        load_area_file(args.out)                      # read back through the same validator
    except FileExistsError:
        print(f"refused: {args.out} exists (never overwritten)", file=sys.stderr)
        return 1
    except (AreaError, ValueError) as exc:
        print(f"refused: {exc}", file=sys.stderr)
        return 2
    print(args.out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
