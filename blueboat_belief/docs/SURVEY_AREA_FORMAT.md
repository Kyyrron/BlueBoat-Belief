# Survey-area file format (`blueboat_survey_area/1`)

Owner decision D-4 (2026-11-04). Belief needs one survey area, which is also its map datum. In
simulation it comes from the world's `metadata.yaml`. On the real boat it comes from a file of
**the same shape**. This page is the specification a future configuration tool must produce.

## Accepted inputs (`area.file` / launch `area_file:=`)

1. A simulator **world folder** or its `metadata.yaml` (`format: blueboat_world_meta/1`, written
   by `BlueBoat-SSS-Sim/.../worldgen/world_folder.py`). Only `format`, `world_name`,
   `geo_anchor` and `limits` are read. Every other key (`objects`, `walls`, `depth`, …) is
   ignored, and `objects` in particular is simulation ground truth, never read online.
2. A **survey-area file**:

```yaml
format: blueboat_survey_area/1           # required
name: harbour_east_2026_11               # optional, free text
created: "2026-11-04T10:00:00Z"          # optional
notes: "…"                               # optional
geo_anchor: {lat0: 33.89, lon0: 130.88, theta_deg: 0.0}   # required; THE MAP DATUM
limits:                                  # required, same keys as the world metadata
  local: {x_min: -20.0, y_min: -10.0, x_max: 80.0, y_max: 50.0}   # metres, ENU about geo_anchor
  corners_gps:                           # [lat, lon] in this order: SW, SE, NE, NW
    - [33.8899101685, 130.8797835671]
    - [33.8899101685, 130.8808657317]
    - [33.8904491576, 130.8808657317]
    - [33.8904491576, 130.8797835671]
```

Any other top-level key is refused, by name. Example: `config/survey_area.example.yaml`.

## Validation (`core/area.py:area_from_dict`)

1. `format` is one of the two values above.
2. `geo_anchor.lat0`, `lon0` are finite and in range; `theta_deg` is 0 (the map frame is the
   anchor's ENU frame, as SSS-Sim also requires).
3. `limits.local` values are finite, with `x_min < x_max` and `y_min < y_max`.
4. Exactly 4 `corners_gps`. Each corner converted with the project pair (`latlon_to_local_en`,
   R = 6378137.0) must be within **0.05 m** of its `limits.local` corner. Otherwise the file is
   refused, and the message names the corner and its distance.
5. The polygon is simple, with area ≥ 1 m².

## What Belief takes from it

- **Datum** = `geo_anchor`: `datum_lat`, `datum_lon` in every `BeliefGrid`.
- **Polygon** = the `limits.local` rectangle (exact), published as `area_x`, `area_y`.
- **Grid** = the bounding box at `grid.resolution_m`, origin at (`x_min`, `y_min`). A cell whose
  centre is outside the polygon is NaN.
- In a world folder the odom origin is the world anchor (the boat spawns at world (0, 0)), so the
  fitted `odom_to_map` translation must come out at about 0. If it does not, `/diagnostics` says
  WARN (cross-check).

- **Empty-world Gazebo runs** have no world folder, so they need a survey-area file too.
  - Its `geo_anchor` should be the mission designer file's anchor; MCS's SimGps fixes
    (status −2, which counts as a fix) are synthesised about it, offset 10 m north of the first
    point.
  - The fit then recovers the odom origin with σ ≈ 0.4 m.

## The minimal tool (built 2026-11-04)

```bash
blueboat-belief-make-area --out harbour_east.yaml --anchor 33.89 130.88 --local -20 -10 80 50 --name harbour_east
blueboat-belief-make-area --out sim_area.yaml --from-world ~/worlds/<path>/<world>
```

It computes `corners_gps` with the project pair and validates through the same reader Belief
uses. It creates the file exclusively (exit 1 if it exists, exit 2 if invalid). `--from-world`
copies only `geo_anchor` and `limits`.

## For the future configuration tool

- Write form 2. Choose `geo_anchor` (for example the mission designer file's `geo_anchor`, or the
  rectangle's corner) and `limits.local`. Compute `corners_gps` with the same pair: SW
  `(x_min, y_min)`, SE `(x_max, y_min)`, NE `(x_max, y_max)`, NW `(x_min, y_max)`.
- Name the file and never overwrite one in use (CM-7). Belief reads it once at start.
- **Reserved, not implemented:** an additive `polygon_gps: [[lat, lon], …]` (≥ 3 vertices) for
  non-rectangular areas, with `limits` kept as its bounding box. Readers older than that key
  would refuse it as unknown, so it needs a new `format` version or a reader update first.
