"""BlueBoat-Belief offline gate: ``python3 -m test.smoke_test`` from ``blueboat_belief/``, no ROS.

SSS-Sim style: ``check()`` prints ``[PASS]``/``[FAIL]`` and stops at the first failure (exit 1);
``[N]`` section headers; a visible ``SKIP`` when an optional dependency is missing; the run ends
with ``ALL CHECKS PASSED`` and the check count. Sections follow docs/DESIGN.md (Q-6) and
brief §11 (closed forms, determinism, localisation, rejections, message layouts, performance).
"""

from __future__ import annotations

import ast
import importlib
import importlib.abc
import importlib.util
import io
import itertools
import json
import math
import os
import pkgutil
import sys
import tempfile
import time
from pathlib import Path
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

import numpy as np  # noqa: E402

N_CHECKS = 0
N_SKIPS = 0


def check(name: str, cond, detail: str = "") -> None:
    global N_CHECKS
    N_CHECKS += 1
    if cond:
        print(f"  [PASS] {name}" + (f" -- {detail}" if detail else ""))
        return
    print(f"  [FAIL] {name}" + (f" -- {detail}" if detail else ""))
    raise SystemExit(f"smoke test failed at: {name}")


def skip(name: str, why: str) -> None:
    global N_SKIPS
    N_SKIPS += 1
    print(f"  SKIP: {name} ({why})")


def close(a, b, tol=1e-12) -> bool:
    return abs(float(a) - float(b)) <= tol * max(1.0, abs(float(a)), abs(float(b)))


# ============================================================================ [0] import sweep
BLOCKED = ("rclpy", "rcl_interfaces", "std_msgs", "sensor_msgs", "nav_msgs", "geometry_msgs",
           "vision_msgs", "visualization_msgs", "diagnostic_msgs", "builtin_interfaces",
           "blueboat_interfaces", "blueboat_belief_interfaces", "launch", "launch_ros",
           "ament_index_python", "PySide6", "PyQt5", "cv2", "torch", "rosbags", "yaml")


class _Blocker(importlib.abc.MetaPathFinder):
    def find_spec(self, name, path=None, target=None):
        if name.split(".")[0] in BLOCKED:
            raise ImportError(f"blocked for the import sweep: {name}")
        return None


def section_import_sweep() -> None:
    print("[0] import sweep: core / io / adapters / cli import with ROS, Qt, cv2, rosbags, yaml blocked")
    saved = {k: v for k, v in sys.modules.items() if k.split(".")[0] in BLOCKED + ("blueboat_belief",)}
    for k in saved:
        del sys.modules[k]
    blocker = _Blocker()
    sys.meta_path.insert(0, blocker)
    try:
        import blueboat_belief  # noqa: F401
        mods = []
        for sub in ("core", "io"):
            pkg = importlib.import_module(f"blueboat_belief.{sub}")
            for m in pkgutil.walk_packages(pkg.__path__, f"blueboat_belief.{sub}."):
                importlib.import_module(m.name)
                mods.append(m.name)
        for extra in ("blueboat_belief.ros.adapters", "blueboat_belief.cli.replay"):
            importlib.import_module(extra)
            mods.append(extra)
        check("ROS-free modules import with ROS/Qt/cv2/rosbags/yaml blocked", len(mods) >= 14,
              f"{len(mods)} modules")
        try:
            importlib.import_module("blueboat_belief.ros.belief_node")
            ok = False
        except ImportError:
            ok = True
        check("ros/belief_node.py needs rclpy (it is the only ROS file)", ok)
    finally:
        sys.meta_path.remove(blocker)
        for k in [k for k in sys.modules if k.split(".")[0] in BLOCKED + ("blueboat_belief",)]:
            del sys.modules[k]
        sys.modules.update(saved)
    # static: no ROS import anywhere but the node; no wall clock / randomness in core
    pkg_dir = ROOT / "blueboat_belief"
    offenders, clock = [], []
    for f in sorted(pkg_dir.rglob("*.py")):
        tree = ast.parse(f.read_text(encoding="utf-8"))
        for node in ast.walk(tree):
            names = []
            if isinstance(node, ast.Import):
                names = [a.name for a in node.names]
            elif isinstance(node, ast.ImportFrom) and node.module and node.level == 0:
                names = [node.module]
            for n in names:
                if n.split(".")[0] in BLOCKED[:13] and f.name != "belief_node.py":
                    offenders.append(f"{f.relative_to(ROOT)}:{n}")
        if "core" in f.parts:
            src = f.read_text(encoding="utf-8")
            for bad in ("time.time(", "datetime.now", "random.", "np.random", "default_rng"):
                if bad in src:
                    clock.append(f"{f.name}:{bad}")
    check("no ROS / message import outside ros/belief_node.py (AST)", not offenders, ", ".join(offenders))
    check("core uses no wall clock and no randomness", not clock, ", ".join(clock))


# ============================================================================ [1] config, geo
def section_config_geo() -> None:
    from blueboat_belief.core.config import BeliefConfig, ConfigError, load_yaml
    from blueboat_belief.core import geo

    print("[1] configuration and geodesy")
    c = BeliefConfig()
    check("defaults validate", c.grid.resolution_m == 0.5 and c.aspect.sector_count == 12)
    check("detector.calibrated defaults to false (calibration is a priority)", c.detector.calibrated is False)
    for bad in ({"grid": {"resolution": 1.0}}, {"bogus": {"x": 1}}, {"grid.nope": 1}):
        try:
            BeliefConfig.from_dict(bad)
            ok = False
        except ConfigError as exc:
            ok = "nope" in str(exc) or "resolution" in str(exc) or "bogus" in str(exc)
        check(f"unknown key raises by name: {list(bad)[0]}", ok)
    for bad in ({"aspect": {"sector_count": 17}}, {"rho": {"ordering": "x"}},
                {"clamp": {"p_min": 0.5, "p_max": 0.4}}, {"grid": {"resolution_m": True}},
                {"alignment": {"origin_lat": 33.0}}):
        try:
            BeliefConfig.from_dict(bad)
            ok = False
        except ConfigError:
            ok = True
        check(f"invalid value refused: {bad}", ok)
    flat = c.to_flat()
    c2 = BeliefConfig.from_flat(flat)
    same = all((flat[k] == v) or (isinstance(v, float) and math.isnan(v) and math.isnan(flat[k]))
               for k, v in c2.to_flat().items())
    check("flat section.key parameters round-trip", same, f"{len(flat)} parameters")
    wrapped = BeliefConfig.from_dict({"belief_map": {"ros__parameters": {"grid": {"resolution_m": 0.25}}}})
    check("ROS parameter-file wrapper accepted", wrapped.grid.resolution_m == 0.25)
    if importlib.util.find_spec("yaml"):
        ly = load_yaml(str(ROOT / "config" / "belief.yaml")).to_flat()
        same = all((ly[k] == v) or (isinstance(v, float) and math.isnan(v) and math.isnan(ly[k]))
                   for k, v in flat.items())
        check("config/belief.yaml equals the code defaults", same)
    else:
        skip("config/belief.yaml load", "PyYAML absent")
    # geodesy: verbatim expressions, bit-identical against the MCS source when present beside
    pts = [(33.8912, 130.8834, 33.89, 130.88), (-12.5, 45.25, -12.49, 45.2), (60.1, -3.3, 60.0, -3.31)]
    for lat, lon, lat0, lon0 in pts:
        e, n = geo.latlon_to_local_en(lat, lon, lat0, lon0)
        e_ref = math.radians(lon - lon0) * 6378137.0 * math.cos(math.radians(lat0))
        n_ref = math.radians(lat - lat0) * 6378137.0
        la, lo = geo.local_en_to_latlon(e, n, lat0, lon0)
        check(f"geo pair bit-identical to the project formula at ({lat}, {lon})",
              e == e_ref and n == n_ref and abs(la - lat) < 1e-12 and abs(lo - lon) < 1e-12)
    mcs = Path(os.environ.get("BLUEBOAT_MCS_DIR", ROOT.parents[1] / "BlueBoat-MCS")) / "mcs" / "core" / "geo.py"
    if mcs.is_file():
        ns: dict = {}
        tree = ast.parse(mcs.read_text(encoding="utf-8"))
        keep = [n for n in tree.body if (isinstance(n, ast.FunctionDef) and n.name in
                ("latlon_to_local_en", "local_en_to_latlon")) or (isinstance(n, ast.Assign) and any(
                getattr(t, "id", "") == "EARTH_RADIUS_M" for t in n.targets))]
        mod = ast.fix_missing_locations(ast.Module(body=[ast.Import(names=[ast.alias(name="math")])] + keep, type_ignores=[]))
        exec(compile(mod, str(mcs), "exec"), ns)
        ok = ns["EARTH_RADIUS_M"] == geo.EARTH_RADIUS_M
        for lat, lon, lat0, lon0 in pts:
            ok &= ns["latlon_to_local_en"](lat, lon, lat0, lon0) == geo.latlon_to_local_en(lat, lon, lat0, lon0)
            e, n = geo.latlon_to_local_en(lat, lon, lat0, lon0)
            ok &= ns["local_en_to_latlon"](e, n, lat0, lon0) == geo.local_en_to_latlon(e, n, lat0, lon0)
        check("geo pair bit-identical to BlueBoat-MCS/mcs/core/geo.py (read as text, not imported)", ok)
    else:
        skip("bit-identity against BlueBoat-MCS geo.py", f"{mcs} not found")


# ============================================================================ [2] survey area
def section_area() -> None:
    from blueboat_belief.core.area import AreaError, area_from_dict, grid_for_area, points_in_polygon
    from test import scenarios as sc

    print("[2] survey area: world metadata and survey-area files, refusals, grid mask")
    a = area_from_dict(sc.area_dict(-10, -20, 30, 20))
    check("survey-area file accepted; polygon = limits", np.allclose(a.polygon_xy,
          [[-10, -20], [30, -20], [30, 20], [-10, 20]], atol=1e-6))
    w = sc.area_dict(-5, -5, 5, 5, fmt="blueboat_world_meta/1", objects=[{"type": "tyre"}],
                     walls=[], spawn={"x": 0, "y": 0})
    aw = area_from_dict(w)
    check("world metadata.yaml accepted; ground-truth keys ignored", aw.source_format == "blueboat_world_meta/1")
    refusals = {
        "unknown key in survey file": dict(sc.area_dict(0, 0, 10, 10), objects=[]),
        "theta != 0": dict(sc.area_dict(0, 0, 10, 10), geo_anchor={"lat0": sc.LAT0, "lon0": sc.LON0, "theta_deg": 5.0}),
        "wrong format": dict(sc.area_dict(0, 0, 10, 10), format="x"),
        "corner disagrees": None,
        "x_min >= x_max": None,
        "non-finite": None,
        "too small": None,
    }
    d = sc.area_dict(0, 0, 10, 10)
    d["limits"]["corners_gps"][2] = [d["limits"]["corners_gps"][2][0] + 1e-5, d["limits"]["corners_gps"][2][1]]
    refusals["corner disagrees"] = d
    d = sc.area_dict(0, 0, 10, 10)
    d["limits"]["local"]["x_min"] = 20.0
    refusals["x_min >= x_max"] = d
    d = sc.area_dict(0, 0, 10, 10)
    d["geo_anchor"]["lat0"] = float("nan")
    refusals["non-finite"] = d
    refusals["too small"] = sc.area_dict(0, 0, 0.5, 0.5)
    for name, bad in refusals.items():
        try:
            area_from_dict(bad)
            ok, msg = False, ""
        except AreaError as exc:
            ok, msg = True, str(exc)
        check(f"refused: {name}", ok, msg[:90])
    geom, inside = grid_for_area(a, 0.5)
    check("grid geometry from the bounding box", geom.width == 80 and geom.height == 80
          and geom.origin_x == -10.0 and geom.origin_y == -20.0)
    check("rectangle: every cell centre inside", bool(inside.all()))
    tri = np.array([[0.0, 0.0], [10.0, 0.0], [0.0, 10.0]])
    m = points_in_polygon(np.array([1.0, 6.0, 9.0]), np.array([1.0, 6.0, 0.5]), tri)
    check("point-in-polygon (triangle)", list(m) == [True, False, True])
    check("cell_of: row 0 south, col 0 west, -1 outside",
          int(geom.cell_of(-9.9, -19.9)) == 0 and int(geom.cell_of(29.9, 19.9)) == geom.n_cells - 1
          and int(geom.cell_of(-10.1, 0.0)) == -1)


# ============================================================================ [3] alignment
def section_alignment() -> None:
    from blueboat_belief.core.alignment import Aligner
    from blueboat_belief.core.area import area_from_dict
    from blueboat_belief.core.belief import Belief, FixObs, OdomObs
    from blueboat_belief.core.config import BeliefConfig
    from blueboat_belief.core.geo import local_en_to_latlon
    from blueboat_belief.core.pose import PoseBuffer
    from test import scenarios as sc

    print("[3] odom -> map alignment (D-3): fit, spread, latch, override, no-fix rules, queue")
    cfg = BeliefConfig().alignment
    al = Aligner(cfg, sc.LAT0, sc.LON0)
    buf = PoseBuffer(0.25, 30.0)
    off = (12.0, -7.5)
    for i in range(30):
        t = 10.0 + 0.2 * i
        buf.add(t, 0.5 * i, 0.1 * i, 0.0)
        lat, lon = local_en_to_latlon(0.5 * i + off[0], 0.1 * i + off[1], sc.LAT0, sc.LON0)
        al.add_fix(t, lat, lon, 0, buf)
        if i == cfg.min_pairs - 2:
            check("not valid before min_pairs", not al.valid)
    check("translation recovered exactly (noise-free)", abs(al.tx - off[0]) < 1e-6 and abs(al.ty - off[1]) < 1e-6,
          f"t=({al.tx:.6f}, {al.ty:.6f})")
    check("valid and latched after min_pairs", al.valid and al.latched and al.source == "fit")
    tx = al.tx
    lat, lon = local_en_to_latlon(500.0, 500.0, sc.LAT0, sc.LON0)
    buf.add(20.0, 0.0, 0.0, 0.0)
    al.add_fix(20.0, lat, lon, 0, buf)
    check("latched: a later outlier does not move the map", al.tx == tx)
    check("status -1 is no fix", Aligner.fix_is_valid(1.0, 1.0, -1) is False)
    check("status -2 (MCS SimGps) is a fix", Aligner.fix_is_valid(33.9, 130.9, -2) is True)
    check("lat == lon == 0 is no fix", Aligner.fix_is_valid(0.0, 0.0, 0) is False)
    rng = np.random.default_rng(3)
    al2 = Aligner(cfg.__class__(**{**cfg.__dict__, "latch": False}), sc.LAT0, sc.LON0)
    buf2 = PoseBuffer(0.25, 300.0)
    for i in range(200):
        t = float(i)
        buf2.add(t, float(i), 0.0, 0.0)
        e, n = i + off[0] + rng.normal(0, 0.4), off[1] + rng.normal(0, 0.4)
        lat, lon = local_en_to_latlon(e, n, sc.LAT0, sc.LON0)
        al2.add_fix(t, lat, lon, -2, buf2)
    check("noisy fixes (sigma 0.4 m, MCS-like): median within 0.15 m, spread ~0.4 m",
          math.hypot(al2.tx - off[0], al2.ty - off[1]) < 0.15 and 0.25 < al2.sigma < 0.6,
          f"err={math.hypot(al2.tx - off[0], al2.ty - off[1]):.3f} sigma={al2.sigma:.3f}")
    ov = Aligner(cfg.__class__(**{**cfg.__dict__, "origin_lat": local_en_to_latlon(3.0, 4.0, sc.LAT0, sc.LON0)[0],
                                  "origin_lon": local_en_to_latlon(3.0, 4.0, sc.LAT0, sc.LON0)[1]}), sc.LAT0, sc.LON0)
    check("override: valid at once, t = EN(origin)", ov.valid and ov.source == "override"
          and abs(ov.tx - 3.0) < 1e-9 and abs(ov.ty - 4.0) < 1e-9)
    # queue/replay order: observations before alignment are kept and replayed (v1 bug 1)
    area = area_from_dict(sc.area_dict(-10, -40, 110, 40))
    b = Belief(BeliefConfig().with_values(coverage={"require_detector": False}), area)
    pings = sc.straight_pass(0, 0, 0.0, 1.0, 20.0)
    for p in pings[:40]:
        b.ingest_odom(OdomObs(p.t, p.x, p.y, p.yaw))
        b.ingest_ping(p)
    check("pings before alignment are queued, not dropped", b.counters["alignment_pending"] == 40
          and b.stats["pings"] == 0)
    for f in sc.fixes_for(pings[:40], every=1):
        b.ingest_fix(f)
    check("queue replayed in order once aligned", b.aligner.valid and b.stats["pings"] == 40,
          f"pings={b.stats['pings']}")
    world = area_from_dict(sc.area_dict(-10, -40, 110, 40, fmt="blueboat_world_meta/1"))
    b2 = Belief(BeliefConfig(), world)
    for p in pings[:40]:
        b2.ingest_odom(OdomObs(p.t, p.x, p.y, p.yaw))
    for f in sc.fixes_for(pings[:40], offset=(2.0, 0.0), every=1):
        b2.ingest_fix(f)
    check("sim cross-check: world anchor != odom origin -> WARN", b2.aligner.sim_check_failed
          and b2.health()[0] == "WARN")


# ============================================================================ [4] pose
def section_pose() -> None:
    from blueboat_belief.core.pose import FrozenPoseDetector, PoseBuffer

    print("[4] pose buffer and frozen-pose detector")
    b = PoseBuffer(0.25, 5.0)
    b.add(1.0, 0.0, 0.0, math.radians(170))
    b.add(1.2, 2.0, 1.0, math.radians(-170))
    x, y, a = b.at(1.1)
    check("interpolation (position linear, yaw on the short arc)", close(x, 1.0) and close(y, 0.5)
          and abs(abs(math.degrees(a)) - 180.0) < 1e-9)
    check("nearest within tolerance when not bracketed", b.at(1.3) == (2.0, 1.0, math.radians(-170)))
    check("None beyond tolerance", b.at(2.0) is None)
    b.add(1.1, 9.0, 9.0, 0.0)
    check("out-of-order insertion stays sorted", b.at(1.1) == (9.0, 9.0, 0.0))
    b.add(10.0, 0.0, 0.0, 0.0)
    check("prune older than buffer_s", len(b) == 1)
    f = FrozenPoseDetector(20)
    rej = [f.update(0.0, 0.0) for _ in range(25)]
    check("frozen after 20 consecutive (0, 0) poses", rej[:19] == [False] * 19 and all(rej[19:]) and f.active)
    f.update(0.1, 0.0)
    check("a moving pose clears the streak", not f.active)


# ============================================================================ [5] geometry
def section_geometry() -> None:
    from blueboat_belief.core.geometry import look_bearing, sample_xy, sector_of, side_of_point, yaw_from_quaternion
    from test import scenarios as sc

    print("[5] look geometry (contract C-5)")
    yaw = math.radians(37.0)
    yoff = np.array([2.0, -3.5])
    px, py = sample_xy(10.0, 20.0, yaw, yoff)
    for i, s in ((0, 1), (1, -1)):
        b = look_bearing(yaw, s)
        r = abs(yoff[i])
        check(f"sample = p + |y|(cos b, sin b), side {s:+d}", close(px[i], 10.0 + r * math.cos(b))
              and close(py[i], 20.0 + r * math.sin(b)))
    check("processor geometry (x - y sin psi, y + y cos psi)",
          close(px[0], 10.0 - 2.0 * math.sin(yaw)) and close(py[0], 20.0 + 2.0 * math.cos(yaw)))
    check("sector 0 = [0, 30) deg CCW from East", sector_of(math.radians(0.0), 12, 0.0) == 0
          and sector_of(math.radians(29.999), 12, 0.0) == 0 and sector_of(math.radians(30.0), 12, 0.0) == 1)
    check("sector of a negative / wrapped bearing", sector_of(math.radians(-1.0), 12, 0.0) == 11
          and sector_of(math.radians(721.0), 12, 0.0) == 0)
    check("sector origin offset", sector_of(math.radians(10.0), 12, 15.0) == 11)
    check("heading East: port looks North (sector 3), starboard South (sector 9)",
          sector_of(look_bearing(0.0, 1), 12, 0.0) == 3 and sector_of(look_bearing(0.0, -1), 12, 0.0) == 9)
    check("side of a point", side_of_point(0, 0, 0.0, 5.0, 2.0) == 1 and side_of_point(0, 0, 0.0, 5.0, -2.0) == -1)
    q = (0.0, 0.0, math.sin(0.6), math.cos(0.6))
    check("yaw from a yaw-only quaternion (real-boat robot_orientation)", close(yaw_from_quaternion(*q), 1.2))
    y = sc.side_offsets(2.0, +1)
    check("synthetic offsets reproduce the processor (count = 1199 - floor(59.95 h))",
          y.size == 1199 - int(math.floor(59.95 * 2.0)) and float(y.min()) > 0.285, f"{y.size} samples")


# ============================================================================ [6] grid closed forms
def _grid(S=12, **kw):
    from blueboat_belief.core.area import area_from_dict, grid_for_area
    from blueboat_belief.core.config import BeliefConfig
    from blueboat_belief.core.grid import Grid
    from test import scenarios as sc

    cfg = BeliefConfig().with_values(aspect={"sector_count": S}, **kw)
    area = area_from_dict(sc.area_dict(0, 0, 10, 10))
    geom, inside = grid_for_area(area, 0.5)
    return Grid(cfg, geom, inside), cfg


def _look(g, t, yaw, side, pd, covered=True, cell=42):
    cells = np.array([cell])
    return g.observe_side(t, 0, yaw, side, cells, np.array([pd]), np.array([-20.0]), 2.2, covered)


def section_grid() -> None:
    from blueboat_belief.core.models import pd_range, prior_cell, rho_table
    from blueboat_belief.core.config import PdCfg, PriorCfg

    print("[6] grid closed forms (MATH §2-§7)")
    lam = prior_cell(PriorCfg(), 0.5)
    check("prior p0 = lambda = 5/ha * 0.25 m^2", close(lam, 1.25e-4))
    ex = prior_cell(PriorCfg(exact_poisson=True), 0.5)
    check("exact Poisson differs by ~lambda^2/2 (7.8e-9)", abs((lam - ex) - 7.8e-9) < 2e-11, f"{lam - ex:.3e}")
    pd = pd_range(np.array([-1.0, 0.0, 1.5, 3.0, 10.0, 25.0, 26.5, 28.0, 30.0]), PdCfg())
    check("PD placeholder trapezoid (0.7 plateau 3-25 m, 3 m ramps)",
          np.allclose(pd, [0, 0, 0.35, 0.7, 0.7, 0.7, 0.35, 0, 0]))
    g, cfg = _grid()
    p0 = g.p0
    _look(g, 1.0, 0.0, 1, 0.7)
    cov = float(g.coverage()[42])
    pb = float(g.p_background()[42])
    bourgault = p0 * 0.3 / (p0 * 0.3 + 1 - p0)
    check("one look: coverage = PD", close(cov, 0.7, 1e-6), f"{cov:.7f}")
    check("one look: p_background = Bourgault p(1-PD)/(p(1-PD)+1-p)", close(pb, bourgault, 1e-6), f"{pb:.4e}")
    check("n_looks 1, aspect_counts[sector 3] 1, n_aspects 1",
          g.n_looks[42] == 1 and g.aspect_counts[42 * 12 + 3] == 1 and g.n_aspects()[42] == 1)
    _look(g, 1.05, 0.0, 1, 0.3)
    check("same look, lower PD: no change (best PD kept)", close(g.coverage()[42], 0.7, 1e-6) and g.n_looks[42] == 1)
    lnq1 = float(g.lnq[42])
    _look(g, 40.0, 0.0, 1, 0.7)
    check("same-sector repeat after timeout: +1 look, no information",
          g.n_looks[42] == 2 and g.lnq[42] == lnq1 and g.aspect_counts[42 * 12 + 3] == 2)
    _look(g, 80.0, math.pi / 2, 1, 0.7)
    check("orthogonal look at equal PD doubles ln Q", close(g.lnq[42], 2 * lnq1, 1e-12),
          f"{g.lnq[42]:.6f} vs {2 * lnq1:.6f}")
    # replacement within a look
    g, _ = _grid()
    _look(g, 1.0, 0.0, 1, 0.3)
    _look(g, 1.1, 0.0, 1, 0.7)
    check("same-look replacement: coverage = best PD, not accumulated", close(g.coverage()[42], 0.7, 1e-6))
    # 45 deg with S = 8 (Paull Eq. 24, sequential form)
    g, _ = _grid(S=8)
    _look(g, 1.0, 0.0, 1, 0.6)
    _look(g, 2.0, math.pi / 4, 1, 0.5)
    c1, c2 = 0.6, 0.5
    check("45 deg look: rho = 1/2, c = c1 + 1/2 c2 (1 - c1) (Paull Eq. 24)",
          close(g.coverage()[42], c1 + 0.5 * c2 * (1 - c1), 1e-6), f"{float(g.coverage()[42]):.6f}")
    # fold_opposite
    t_fold = rho_table(12, 0.0, True, 90.0)
    t_dir = rho_table(12, 0.0, False, 90.0)
    check("opposite looks: folded rho = 0 (Paull line angle), directed rho = 1",
          t_fold[9, 1 << 3] == 0.0 and t_dir[9, 1 << 3] == 1.0)
    check("rho uses the minimum over searched sectors", t_fold[3, (1 << 3) | (1 << 6)] == 0.0
          and close(t_fold[4, (1 << 0) | (1 << 6)], 2.0 / 3.0))
    # paull_pairwise (default since 2026-11-04) vs sequential (S-5 example)
    gs, _ = _grid(S=8, rho={"ordering": "sequential"})
    _look(gs, 1.0, 0.0, 1, 0.3)
    _look(gs, 2.0, math.pi / 4, 1, 0.7)
    gp, _ = _grid(S=8, rho={"ordering": "paull_pairwise"})
    _look(gp, 1.0, 0.0, 1, 0.3)
    _look(gp, 2.0, math.pi / 4, 1, 0.7)
    check("sequential: 0.3 then 0.7 at rho 1/2 -> 0.545", close(gs.coverage()[42], 0.545, 1e-6))
    check("paull_pairwise: same -> 0.745 (c_hi + rho c_lo (1 - c_hi))", close(gp.coverage()[42], 0.745, 1e-6))
    gd, cfgd = _grid()
    check("default ordering is paull_pairwise (Paull §III-C, c1 >= c2)", cfgd.rho.ordering == "paull_pairwise")
    _look(gd, 1.0, 0.0, 1, 0.3)
    _look(gd, 40.0, 0.0, 1, 0.7)
    check("pairwise: a stronger same-sector look raises coverage to its PD (Paull Eq. 22: max)",
          close(gd.coverage()[42], 0.7, 1e-6) and gd.n_looks[42] == 2)
    _look(gd, 80.0, 0.0, 1, 0.5)
    check("pairwise: a weaker same-sector look adds nothing", close(gd.coverage()[42], 0.7, 1e-6))
    gq, _ = _grid(rho={"ordering": "sequential"})
    _look(gq, 1.0, 0.0, 1, 0.3)
    _look(gq, 40.0, 0.0, 1, 0.7)
    check("sequential (switch): the stronger same-sector look adds nothing (departs from Paull)",
          close(gq.coverage()[42], 0.3, 1e-6))
    # per-sector max rule
    g, _ = _grid()
    _look(g, 1.0, 0.0, 1, 0.3)
    _look(g, 40.0, 0.0, 1, 0.7)
    check("per-sector ln Q = ln(1 - max PD) (Paull alpha = 0)", close(g.aspect_log_miss()[42 * 12 + 3], math.log(0.3), 1e-6)
          and g.aspect_log_miss()[42 * 12 + 4] == 0.0)
    # geometry-only
    g, _ = _grid()
    _look(g, 1.0, 0.0, 1, 0.7, covered=False)
    check("geometry-only ping: look and aspect counted, coverage 0",
          g.n_looks[42] == 1 and g.aspect_counts[42 * 12 + 3] == 1 and g.coverage()[42] == 0.0)
    _look(g, 2.0, math.pi / 2, 1, 0.7)
    check("a geometry-only look does not make later looks dependent", close(g.coverage()[42], 0.7, 1e-6))
    # PFA term
    g, _ = _grid(pd={"pfa": 0.1})
    _look(g, 1.0, 0.0, 1, 0.7)
    l0 = math.log(p0) - math.log1p(-p0)
    check("PFA term: l = l0 + ln((1 - PD)/(1 - PFA))", close(g.ell[42], l0 + math.log(0.3 / 0.9), 1e-12))
    # clamp at read
    g, _ = _grid()
    for i in range(12):
        _look(g, 30.0 * i, i * math.pi / 6, 1, 0.7)
    check("p_background clamped at p_min on read", close(g.p_background()[42], 1e-5, 1e-5))
    check("NaN outside the area only", not np.isnan(g.p_background()).any())
    # numerical form of the pairwise rule (MATH §7): exact no-op at rho = 0, Q never rises
    from blueboat_belief.core.models import combine_miss

    lq = np.log1p(-np.array([0.0, 0.3, 0.7, 0.7, 0.2]))
    out = combine_miss(lq, np.array([1.0, 0.5, 0.0, 0.0, 1.0]), np.array([0.7, 0.7, 0.7, 0.9, 0.1]),
                       "paull_pairwise")
    exp = np.log([0.3, 0.3 * (1 - 0.5 * 0.3), 0.3, 0.1, 0.8 * 0.9])
    check("combine_miss closed forms: fresh, stronger at rho 1/2, equal repeat, stronger repeat, weaker independent",
          np.allclose(out, exp, atol=1e-12) and out[2] == lq[2], f"{np.round(-np.expm1(out), 6).tolist()}")
    drift = 0.0
    for pd0 in (0.05, 0.123456789, 0.3, 0.35, 0.5, 0.6, 0.7, 0.9, 0.95):
        g, _ = _grid()
        _look(g, 1.0, 0.0, 1, pd0)
        l0, e0 = float(g.lnq[42]), float(g.ell[42])
        for i in range(1000):                   # yaw 0 / pi, port / starboard: all on one line
            _look(g, 100.0 * (i + 1), 0.0 if i % 2 else math.pi, 1 if i % 3 else -1, pd0)
        drift = max(drift, abs(float(g.lnq[42]) - l0), abs(float(g.ell[42]) - e0))
        if pd0 == 0.7:
            n_rep = int(g.n_looks[42])
    check("rho = 0 repeats (1000 per PD, both sides of one line): ln Q and the log-odds stay bit-identical",
          drift == 0.0 and n_rep == 1001, f"max drift {drift:.1e}, n_looks {n_rep}")
    rng = np.random.default_rng(11)
    for ordering in ("paull_pairwise", "sequential"):
        up = 0.0
        for _ in range(400):
            g, _ = _grid(rho={"ordering": ordering})
            prev = 0.0
            for i in range(int(rng.integers(2, 9))):
                _look(g, 100.0 * (i + 1), math.radians(30.0 * float(rng.integers(0, 12))),
                      int(rng.choice([1, -1])), float(rng.uniform(0.0, 0.95)))
                up = max(up, float(g.lnq[42]) - prev)
                prev = float(g.lnq[42])
        check(f"{ordering}: ln Q never rises over 400 random look sequences", up <= 0.0, f"largest step {up:.1e}")

    # order dependence beyond two looks: documented, kept, pinned (MATH §7)
    def _cov(angles_deg) -> float:
        g, _ = _grid()
        for i, a in enumerate(angles_deg):
            _look(g, 100.0 * (i + 1), math.radians(a), 1, 0.7)
        return -math.expm1(float(g.lnq[42]))

    check("three looks, same set, two orders: 0,90,30 -> 0.9310 and 0,30,90 -> 0.8773 (MATH §7 table)",
          close(_cov((0, 90, 30)), 1 - 0.09 * (1 - 0.7 / 3), 1e-9)
          and close(_cov((0, 30, 90)), 1 - 0.3 * (1 - 0.7 / 3) * (1 - 1.4 / 3), 1e-9),
          f"{_cov((0, 90, 30)):.4f} / {_cov((0, 30, 90)):.4f}")
    four = [_cov(p) for p in itertools.permutations((0, 30, 60, 90))]
    check("four looks 0,30,60,90 in every order: coverage between 0.8648 and 0.9471",
          close(min(four), 0.8648, 1e-4) and close(max(four), 0.9471, 1e-4), f"{min(four):.4f} .. {max(four):.4f}")
    two_pass = [_cov(p) for p in itertools.permutations((0, 180, 90, 270))]
    check("two-pass orthogonal with both sides is order-independent: 1 - 0.3^2 in all 24 orders",
          all(close(c, 0.91, 1e-12) for c in two_pass), f"{min(two_pass):.6f} .. {max(two_pass):.6f}")


# ============================================================================ [7] contacts
def _table(**kw):
    from blueboat_belief.core.config import BeliefConfig
    from blueboat_belief.core.contacts import ContactTable

    return ContactTable(BeliefConfig().with_values(**kw))


def section_contacts() -> None:
    from blueboat_belief.core.models import logit_scalar

    print("[7] contacts (MATH §9, §10)")
    R = np.eye(2) * 1.09
    tb = _table()
    lp = logit_scalar(0.5)
    c, created = tb.add_detection(1.0, 0, 3, 0.8, np.array([5.0, 5.0]), R, "a", "block", 90.0, 8.0)
    check("creation: L = logit q", created and close(c.L, logit_scalar(0.8)))
    tb.add_detection(1.2, 0, 3, 0.9, np.array([5.1, 5.0]), R, "b", "block", 90.0, 8.0)
    check("same look, better q: replaces (L = logit 0.9), one detected look",
          close(c.L, logit_scalar(0.9)) and c.n_detected() == 1 and len(tb.contacts) == 1)
    tb.add_detection(1.3, 0, 3, 0.6, np.array([5.0, 5.1]), R, "c", "block", 90.0, 8.0)
    check("same look, worse q: unchanged", close(c.L, logit_scalar(0.9)))
    tb.add_detection(40.0, 0, 6, 0.7, np.array([5.0, 5.0]), R, "d", "block", 180.0, 8.0)
    check("orthogonal detected look: L += rho (logit q - logit pi), rho = 1",
          close(c.L, logit_scalar(0.9) + 1.0 * (logit_scalar(0.7) - lp)), f"p={c.p_exists:.4f}")
    check("confirmed after 2 detected looks", c.state == 1)
    L_before = c.L
    tb.observe(80.0, 0, 1, {7: 0.7}, {c.uid: 7}, True, (0.0, 0.0), 45.0)
    check("an open look is not yet a miss", c.L == L_before)
    tb.close_due(80.0 + 25.0, 0)
    check("missed look applied at close: L += ln(1 - rho PD), rho(sector 1 | {3, 6}) = 1/3 (folded)",
          close(c.L, L_before + math.log1p(-0.7 / 3.0)))
    L_after = c.L
    tb.close_due(200.0, 0)
    check("a miss is applied once per look", c.L == L_after)
    d = tb.to_dicts(("block", "pipe", "cage", "tyre", "plate", "other"))[0]
    check("look history: sectors detected [3, 6], missed [1]",
          list(d["sectors_detected"]) == [3, 6] and list(d["sectors_missed"]) == [1]
          and list(d["look_detected"]) == [True, True, False])
    check("first_confidence / first_sector kept", close(d["first_confidence"], 0.8, 1e-6) and int(d["first_sector"]) == 3)
    # fused covariance: common mode removed then added once
    t2 = _table()
    for k, s in enumerate((0, 3, 6, 9)):
        t2.add_detection(30.0 * k, 0, s, 0.8, np.array([0.0, 0.0]), np.eye(2) * 1.09, str(k), "block", 0, 5)
    c2 = t2.contacts[1]
    exp_var = 0.09 / 4 + 1.0 + 0.25
    check("4 looks fused: P = R_ind/4 + (sigma_cm^2 + floor^2) I (common mode not shrunk)",
          np.allclose(c2.P, np.eye(2) * exp_var, atol=1e-9), f"var={c2.P[0, 0]:.4f}")
    # association gates
    t3 = _table()
    t3.add_detection(0.0, 0, 3, 0.8, np.array([0.0, 0.0]), np.eye(2) * 0.01, "a", "block", 0, 5)
    t3.contacts[1].P = np.eye(2) * 0.01
    _, created = t3.add_detection(0.1, 0, 3, 0.8, np.array([1.5, 0.0]), np.eye(2) * 0.01, "b", "block", 0, 5)
    check("tight covariance: 1.5 m fails chi-square -> new contact", created and len(t3.contacts) == 2)
    t4 = _table()
    t4.add_detection(0.0, 0, 3, 0.8, np.array([0.0, 0.0]), np.eye(2) * 4.0, "a", "block", 0, 5)
    _, created = t4.add_detection(0.1, 0, 3, 0.8, np.array([2.5, 0.0]), np.eye(2) * 4.0, "b", "block", 0, 5)
    check("broad covariance: 2.5 m fails the 2 m gate -> new contact", created)
    _, created = t4.add_detection(0.2, 0, 3, 0.8, np.array([1.5, 0.0]), np.eye(2) * 4.0, "c", "block", 0, 5)
    check("broad covariance: 1.5 m associates (nearest contact)", not created)
    t5 = _table()
    t5.add_detection(0.0, 0, 3, 0.8, np.array([0.0, 0.0]), R, "a", "block", 0, 5)
    t5.add_detection(0.0, 0, 3, 0.8, np.array([3.0, 0.0]), R, "b", "block", 0, 5)
    c, _ = t5.add_detection(1.0, 0, 3, 0.8, np.array([1.8, 0.0]), R, "c", "block", 0, 5)
    check("two nearby contacts: a detection joins the nearer", c.uid == 2)
    # pruning, uid retirement, cap
    t6 = _table(contact={"prune_p": 0.5})
    c, _ = t6.add_detection(0.0, 0, 3, 0.4, np.array([0.0, 0.0]), R, "a", "block", 0, 5)
    t6.close_due(100.0, 0)
    t6.add_detection(101.0, 0, 3, 0.8, np.array([0.0, 0.0]), R, "b", "block", 0, 5)
    check("pruned candidate's uid is never reused", 1 not in t6.contacts and 2 in t6.contacts and t6.retired == 1)
    t7 = _table(contact={"max_contacts": 3})
    for k in range(5):
        t7.add_detection(float(k), 0, 3, 0.5 + 0.1 * k, np.array([10.0 * k, 0.0]), R, str(k), "block", 0, 5)
    check("table capped (lowest p first)", sorted(t7.contacts) == [3, 4, 5])
    # class distribution and unknown names
    t8 = _table()
    c, _ = t8.add_detection(0.0, 0, 3, 0.8, np.array([0.0, 0.0]), R, "a", "tyre", 0, 5)
    t8.add_detection(40.0, 0, 6, 0.6, np.array([0.0, 0.0]), R, "b", "weird_class", 0, 5)
    a = c.alpha / c.alpha.sum()
    check("class: Dirichlet q-weighted counts; unknown name -> 'other'",
          close(a[3], 1.8 / 7.4) and close(a[5], 1.6 / 7.4))


# ============================================================================ [8] rendering, query
def section_render() -> None:
    from blueboat_belief.core.area import area_from_dict
    from blueboat_belief.core.belief import Belief
    from blueboat_belief.core.config import BeliefConfig
    from test import scenarios as sc

    print("[8] rendering and disc query (MATH §11, §12)")
    b = Belief(BeliefConfig().with_values(coverage={"require_detector": False}),
               area_from_dict(sc.area_dict(-20, -20, 20, 20)))
    c, _ = b.contacts.add_detection(0.0, 0, 3, 0.8, np.array([0.25, 0.25]), np.eye(2) * 1.09, "a", "block", 0, 5)
    sig2 = c.P[0, 0]
    pt = b.render_p_target()
    pbg = b.grid.p_background()
    j = int(b.geom.cell_of(0.25, 0.25))
    w = 0.25 / (2 * math.pi * sig2)
    expect = 1 - (1 - pbg[j]) * (1 - c.p_exists * w)
    check("p_target = 1 - (1 - p_bg)(1 - p_c w), w = pdf(centre) * cell area", close(pt[j], expect, 1e-6),
          f"{float(pt[j]):.5f}")
    for R in (0.5, 1.0, 2.0, 4.0):
        m = b.contact_disc_mass(c, 0.25, 0.25, R)
        check(f"disc mass of an isotropic contact, R = {R}: 1 - exp(-R^2 / 2 sigma^2)",
              abs(m - (1 - math.exp(-R * R / (2 * sig2)))) < 2e-3, f"{m:.4f}")
    q = b.query_disc(0.25, 0.25, 3.0)
    cx, cy = b.geom.centres()
    sel = ((cx - 0.25) ** 2 + (cy - 0.25) ** 2 <= 9.0)
    naive = 1 - np.prod(1 - pt[sel].astype(np.float64))
    check("p_any uses the contact's disc mass; the rendered-map product under-counts (v1 bug 2)",
          q["p_any"] > naive + 0.05, f"exact={q['p_any']:.4f} naive={naive:.4f}")
    check("query: contacts inside, cells counted, class distribution normalised",
          q["contact_uids"] == [1] and q["total_cells"] == int(sel.sum()) and close(sum(q["class_distribution"]), 1.0))
    check("query refuses a non-positive radius", b.query_disc(0, 0, 0)["ok"] is False)


# ============================================================================ [9] contract dict
def section_contract() -> None:
    from blueboat_belief.core.area import area_from_dict
    from blueboat_belief.core.belief import Belief
    from blueboat_belief.core.config import BeliefConfig
    from blueboat_belief.core.contract import ADDED_FIELDS, CONTACT_FIELDS, GRID_FIELDS, to_grid_dict
    from test import scenarios as sc

    print("[9] to_grid_dict: every C-3 / C-4 name with the contract dtype (C-7)")
    b = Belief(BeliefConfig(), area_from_dict(sc.area_dict(-10, -40, 110, 40)))
    pings = sc.straight_pass(0, 0, 0.0, 1.0, 60.0)
    sc.feed(b, sc.events(pings, sc.odom_for(pings), sc.fixes_for(pings), sc.arrays_for(pings, [(30.0, 10.0)])))
    b.finish()
    d = to_grid_dict(b, grid_seq=7)
    check("exactly the contract fields", set(d) == set(GRID_FIELDS), str(sorted(set(d) ^ set(GRID_FIELDS))))
    n, S = int(d["width"]) * int(d["height"]), int(d["aspect_sector_count"])
    dt = {"float32[]": np.float32, "uint16[]": np.uint16, "uint32[]": np.uint32, "uint8[]": np.uint8,
          "float64[]": np.float64}
    bad = []
    for name, typ in GRID_FIELDS.items():
        v = d[name]
        if typ in dt:
            if not (isinstance(v, np.ndarray) and v.dtype == dt[typ]):
                bad.append(name)
            elif name.startswith("aspect_"):
                if v.size != n * S:
                    bad.append(name + "(size)")
            elif not name.startswith("area_") and v.size != n:
                bad.append(name + "(size)")
    check("array fields: exact dtype and size (w*h, or w*h*S cell-major)", not bad, ", ".join(bad))
    check("contract_version and grid_seq", d["contract_version"] == "belief_grid/0" and int(d["grid_seq"]) == 7)
    inside = b.inside
    nan_ok = all(np.array_equal(np.isnan(d[k]), ~inside) for k in ("p_background", "p_target", "coverage"))
    check("probability channels NaN outside the area only", nan_ok)
    unseen = inside & (d["n_looks"] == 0)
    check("unseen inside: coverage 0, aspect_counts 0, depth/intensity NaN",
          bool((d["coverage"][unseen] == 0).all()) and bool(np.isnan(d["depth_m"][unseen]).all())
          and int(d["aspect_counts"].reshape(-1, S)[unseen].sum()) == 0)
    mp = json.loads(d["model_params_json"])
    check("model_params_json: schema 1, calibration flag, gating, ordering", mp["schema"] == 1
          and mp["detector_calibrated"] is False and mp["negative_evidence"] == "gated"
          and mp["rho_model"]["ordering"] == "paull_pairwise" and mp["turn_gating"]["enabled"] is True)
    check("odom_to_map valid with sigma", bool(d["odom_to_map_valid"]) and math.isfinite(float(d["odom_to_map_sigma_m"])))
    cs = d["contacts"]
    check("one contact with every C-4 field", len(cs) == 1 and set(cs[0]) == set(CONTACT_FIELDS))
    check("ADDED fields recorded", ADDED_FIELDS <= set(GRID_FIELDS) | set(CONTACT_FIELDS))


# ============================================================================ [10] message layouts
_DT = {"float32": np.float32, "float64": np.float64, "uint8": np.uint8, "int8": np.int8,
       "uint16": np.uint16, "int16": np.int16, "uint32": np.uint32, "int32": np.int32,
       "uint64": np.uint64, "int64": np.int64, "byte": np.uint8, "char": np.uint8,
       "octet": np.uint8, "bool": np.bool_}


def _typestore():
    from rosbags.typesys import Stores, get_types_from_msg, get_typestore

    ts = get_typestore(Stores.ROS2_JAZZY)
    add = {}
    ref = ROOT / "test" / "msg_reference"
    for f in sorted(ref.glob("*/msg/*.msg")):
        add.update(get_types_from_msg(f.read_text(), f"{f.parent.parent.name}/msg/{f.stem}"))
    ip = ROOT.parent / "blueboat_belief_interfaces"
    for n in ("BeliefContact", "BeliefGrid"):
        add.update(get_types_from_msg((ip / "msg" / f"{n}.msg").read_text(), f"blueboat_belief_interfaces/msg/{n}"))
    req, resp = (ip / "srv" / "QueryBelief.srv").read_text().split("---")
    add.update(get_types_from_msg(req, "blueboat_belief_interfaces/msg/QueryBelief_Request"))
    add.update(get_types_from_msg(resp, "blueboat_belief_interfaces/msg/QueryBelief_Response"))
    ts.register(add)
    return ts


def _type_str(nt, det) -> str:
    from rosbags.interfaces import Nodetype  # type: ignore
    if nt == Nodetype.BASE:
        return det[0]
    if nt == Nodetype.NAME:
        return det.replace("/msg/", "/")
    if nt == Nodetype.ARRAY:
        return f"{_type_str(*det[0])}[{det[1]}]"
    if nt == Nodetype.SEQUENCE:
        return f"{_type_str(*det[0])}[]"
    return "?"


def materialize(ts, typ: str, val):
    """Build a rosbags message dataclass from an adapter field dict (missing fields = defaults)."""
    from rosbags.interfaces import Nodetype  # type: ignore

    _, fields = ts.fielddefs[typ]
    kw = {}
    for name, (nt, det) in fields:
        v = val.get(name) if isinstance(val, dict) else None
        if nt == Nodetype.BASE:
            base = det[0]
            if base == "string":
                kw[name] = "" if v is None else str(v)
            elif base == "bool":
                kw[name] = bool(v) if v is not None else False
            elif base in ("float32", "float64"):
                kw[name] = float(v) if v is not None else 0.0
            else:
                kw[name] = int(v) if v is not None else 0
        elif nt == Nodetype.NAME:
            kw[name] = materialize(ts, det, v or {})
        elif nt in (Nodetype.ARRAY, Nodetype.SEQUENCE):
            (snt, sdet), size = det
            if snt == Nodetype.NAME:
                kw[name] = [materialize(ts, sdet, e) for e in (v or [])]
            elif sdet[0] == "string":
                kw[name] = [str(e) for e in (v or [])]
            else:
                arr = np.asarray(v if v is not None else ([0] * size if nt == Nodetype.ARRAY else []),
                                 dtype=_DT[sdet[0]])
                kw[name] = arr
    return ts.types[typ](**kw)


def section_layouts() -> None:
    print("[10] message layouts with rosbags (ROS-free): .msg parse, dtypes, CDR round-trips")
    if not importlib.util.find_spec("rosbags"):
        skip("message layouts", "rosbags not installed (pip install rosbags)")
        return
    from blueboat_belief.core.area import area_from_dict
    from blueboat_belief.core.belief import Belief
    from blueboat_belief.core.config import BeliefConfig
    from blueboat_belief.core.contract import CONTACT_FIELDS, GRID_FIELDS, to_grid_dict
    from blueboat_belief.ros import adapters as ad
    from test import scenarios as sc

    ts = _typestore()
    gdef = dict(ts.fielddefs["blueboat_belief_interfaces/msg/BeliefGrid"][1])
    cdef = dict(ts.fielddefs["blueboat_belief_interfaces/msg/BeliefContact"][1])
    gmis = [k for k, t in GRID_FIELDS.items() if k not in gdef or _type_str(*gdef[k]) != t]
    cmis = [k for k, t in CONTACT_FIELDS.items() if k not in cdef or _type_str(*cdef[k]) != t]
    check("BeliefGrid.msg = contract GRID_FIELDS (names and types)", not gmis and len(gdef) == len(GRID_FIELDS),
          ", ".join(f"{k}:{_type_str(*gdef[k]) if k in gdef else '-'}" for k in gmis))
    check("BeliefContact.msg = contract CONTACT_FIELDS", not cmis and len(cdef) == len(CONTACT_FIELDS),
          ", ".join(cmis))
    consts = dict((c[0], c[2]) for c in ts.fielddefs["blueboat_belief_interfaces/msg/BeliefContact"][0])
    check("BeliefContact constants", consts == {"STATE_CANDIDATE": 0, "STATE_CONFIRMED": 1, "SECTOR_UNKNOWN": 255})
    b = Belief(BeliefConfig(), area_from_dict(sc.area_dict(-10, -40, 110, 40)))
    pings = sc.straight_pass(0, 0, 0.0, 1.0, 60.0)
    sc.feed(b, sc.events(pings, sc.odom_for(pings), sc.fixes_for(pings), sc.arrays_for(pings, [(30.0, 10.0)])))
    b.finish()
    d = to_grid_dict(b, grid_seq=3)
    msg = materialize(ts, "blueboat_belief_interfaces/msg/BeliefGrid", ad.grid_to_fields(d))
    raw = ts.serialize_cdr(msg, "blueboat_belief_interfaces/msg/BeliefGrid")
    back = ts.deserialize_cdr(raw, "blueboat_belief_interfaces/msg/BeliefGrid")
    same = all(np.array_equal(getattr(back, k), d[k], equal_nan=True) for k in
               ("p_background", "coverage", "n_looks", "n_pings", "aspect_counts", "aspect_log_miss", "area_x"))
    check("BeliefGrid CDR round-trip (arrays bit-equal)", same, f"{len(raw) / 1e6:.2f} MB")
    check("contacts survive CDR (uid, covariance, look_detected, stamps)",
          len(back.contacts) == 1 and int(back.contacts[0].uid) == 1
          and np.array_equal(back.contacts[0].covariance, d["contacts"][0]["covariance"])
          and list(back.contacts[0].look_detected) == list(d["contacts"][0]["look_detected"])
          and back.header.frame_id == "map")
    q = b.query_disc(30.0, 10.0, 3.0)
    qf = ad.query_to_fields(q, 3)
    rdef = dict(ts.fielddefs["blueboat_belief_interfaces/msg/QueryBelief_Response"][1])
    check("QueryBelief response fields = adapter output", set(qf) == set(rdef), str(set(qf) ^ set(rdef)))
    qm = materialize(ts, "blueboat_belief_interfaces/msg/QueryBelief_Response", qf)
    qb = ts.deserialize_cdr(ts.serialize_cdr(qm, "blueboat_belief_interfaces/msg/QueryBelief_Response"),
                            "blueboat_belief_interfaces/msg/QueryBelief_Response")
    check("QueryBelief response CDR round-trip", qb.ok and abs(qb.p_any - q["p_any"]) < 1e-6)
    for ch in ("coverage", "p_target"):
        of = ad.occupancy_fields(d, ch)
        om = materialize(ts, "nav_msgs/msg/OccupancyGrid", of)
        ob = ts.deserialize_cdr(ts.serialize_cdr(om, "nav_msgs/msg/OccupancyGrid"), "nav_msgs/msg/OccupancyGrid")
        check(f"OccupancyGrid ({ch}): int8 data in [-1, 100], CDR round-trip",
              ob.data.dtype == np.int8 and ob.data.min() >= -1 and ob.data.max() <= 100
              and ob.info.width == int(d["width"]) and np.array_equal(ob.data, of["data"]))
    mf = ad.markers_fields(d)
    mm = materialize(ts, "visualization_msgs/msg/MarkerArray", mf)
    mb = ts.deserialize_cdr(ts.serialize_cdr(mm, "visualization_msgs/msg/MarkerArray"), "visualization_msgs/msg/MarkerArray")
    check("MarkerArray: DELETEALL + ellipse + label per contact", len(mb.markers) == 3 and mb.markers[0].action == 3)
    df = ad.diagnostics_fields("WARN", "x", b.counters, b.stats)
    dm = materialize(ts, "diagnostic_msgs/msg/DiagnosticStatus", df)
    db = ts.deserialize_cdr(ts.serialize_cdr(dm, "diagnostic_msgs/msg/DiagnosticStatus"), "diagnostic_msgs/msg/DiagnosticStatus")
    check("DiagnosticStatus: level, every rejection counter", db.level == 1
          and sum(1 for kv in db.values if kv.key.startswith("rejected.")) == len(b.counters))
    # inputs: real layouts in, adapters out
    p = pings[100]
    pm = materialize(ts, "blueboat_interfaces/msg/ProcessedSSSPing", {
        "port_stamp": ad.s_to_stamp(p.t), "starboard_stamp": ad.s_to_stamp(p.t),
        "port_ping_number": p.port_ping_number, "starboard_ping_number": p.starboard_ping_number,
        "robot_x": p.x, "robot_y": p.y, "robot_orientation": {"x": 0.0, "y": 0.0, "z": math.sin(0.3), "w": math.cos(0.3)},
        "water_depth": p.water_depth, "transducer_x_offset": 0.0,
        "port_intensity_db": p.port_db, "port_y": p.port_y,
        "starboard_intensity_db": p.starboard_db, "starboard_y": p.starboard_y})
    pb = ts.deserialize_cdr(ts.serialize_cdr(pm, "blueboat_interfaces/msg/ProcessedSSSPing"),
                            "blueboat_interfaces/msg/ProcessedSSSPing")
    po, used = ad.ping_from_msg(pb)
    check("ProcessedSSSPing (reference .msg) -> PingObs", not used and abs(po.t - p.t) < 1e-9
          and abs(po.yaw - 0.6) < 1e-6 and np.array_equal(po.port_y, p.port_y) and po.port_ping_number == p.port_ping_number)
    dm = materialize(ts, "vision_msgs/msg/Detection2DArray", {
        "header": {"stamp": ad.s_to_stamp(130.0), "frame_id": "odom"},
        "detections": [{"header": {"stamp": ad.s_to_stamp(129.5), "frame_id": "odom"},
                        "id": "2000016",
                        "bbox": {"center": {"position": {"x": 30.0, "y": 10.0}, "theta": 0.1}, "size_x": 1.0, "size_y": 1.0},
                        "results": [{"hypothesis": {"class_id": "tyre", "score": 0.42},
                                     "pose": {"pose": {"position": {"x": 30.0, "y": 10.0, "z": 0.0},
                                                       "orientation": {"w": 1.0}},
                                              "covariance": [1.09, 0.01] + [0.0] * 4 + [0.01, 1.2] + [0.0] * 28}}]}]})
    dbk = ts.deserialize_cdr(ts.serialize_cdr(dm, "vision_msgs/msg/Detection2DArray"), "vision_msgs/msg/Detection2DArray")
    arr, skipped = ad.detections_from_msg(dbk)
    det = arr.detections[0]
    check("Detection2DArray (vision_msgs reference, Perception fill) -> DetArray",
          skipped == 0 and abs(arr.t - 130.0) < 1e-9 and abs(det.t_row - 129.5) < 1e-9 and det.id == "2000016"
          and det.class_name == "tyre" and abs(det.score - 0.42) < 1e-12
          and np.allclose(det.cov, [[1.09, 0.01], [0.01, 1.2]]))
    om = materialize(ts, "nav_msgs/msg/Odometry", {"header": {"stamp": {"sec": 0, "nanosec": 0}},
                     "pose": {"pose": {"position": {"x": 1.0, "y": 2.0}, "orientation": {"z": math.sin(0.5), "w": math.cos(0.5)}}}})
    ob = ts.deserialize_cdr(ts.serialize_cdr(om, "nav_msgs/msg/Odometry"), "nav_msgs/msg/Odometry")
    oo, used = ad.odom_from_msg(ob, arrival_t=77.0)
    check("Odometry with a zero stamp -> arrival time used", used and oo.t == 77.0 and abs(oo.yaw - 1.0) < 1e-9)
    fm = materialize(ts, "sensor_msgs/msg/NavSatFix", {"header": {"stamp": ad.s_to_stamp(5.0)}, "status": {"status": -2},
                     "latitude": 33.9, "longitude": 130.9})
    fb = ts.deserialize_cdr(ts.serialize_cdr(fm, "sensor_msgs/msg/NavSatFix"), "sensor_msgs/msg/NavSatFix")
    fo, used = ad.fix_from_msg(fb)
    check("NavSatFix -> FixObs (status -2 kept)", not used and fo.status == -2 and fo.lat == 33.9 and fo.t == 5.0)


# ============================================================================ [11] adapters (stubs)
def section_adapters() -> None:
    from blueboat_belief.ros import adapters as ad

    print("[11] adapters with stub messages (attribute and key access)")
    ns = SimpleNamespace
    stub = ns(port_stamp=ns(sec=0, nanosec=0), starboard_stamp=ns(sec=12, nanosec=500000000),
              port_ping_number=0, starboard_ping_number=7, robot_x=1.0, robot_y=2.0,
              robot_orientation=ns(x=0.0, y=0.0, z=0.0, w=1.0), water_depth=2.2, transducer_x_offset=0.0,
              port_intensity_db=[], port_y=[], starboard_intensity_db=[-20.0], starboard_y=[-3.0])
    p, used = ad.ping_from_msg(stub, arrival_t=99.0)
    check("one-sided row: starboard stamp used (presence = ping_number != 0)", not used and p.t == 12.5)
    d = {"port_stamp": {"sec": 0, "nanosec": 0}, "starboard_stamp": {"sec": 0, "nanosec": 0},
         "port_ping_number": 3, "starboard_ping_number": 3, "robot_x": 0.0, "robot_y": 0.0,
         "robot_orientation": {"x": 0, "y": 0, "z": 0, "w": 1}, "water_depth": 2.0,
         "port_y": [1.0], "port_intensity_db": [-1.0], "starboard_y": [], "starboard_intensity_db": []}
    p, used = ad.ping_from_msg(d, arrival_t=42.0)
    check("dict access; zero stamp -> arrival time", used and p.t == 42.0 and p.port_y.dtype == np.float32)
    arr, skipped = ad.detections_from_msg(ns(header=ns(stamp=ns(sec=10, nanosec=0)),
                                             detections=[ns(results=[], id="x")]))
    check("detection without results is skipped and counted", skipped == 1 and arr.detections == ())
    st = ad.s_to_stamp(12.999999999)
    check("s_to_stamp carries nanosecond overflow", st == {"sec": 13, "nanosec": 0} or st == {"sec": 12, "nanosec": 999999999})


# ============================================================================ [12] determinism
def _run_scenario(seed: int = 7, cfg=None):
    from blueboat_belief.core.area import area_from_dict
    from blueboat_belief.core.belief import Belief
    from blueboat_belief.core.config import BeliefConfig
    from test import scenarios as sc

    cfg = cfg or BeliefConfig()
    b = Belief(cfg, area_from_dict(sc.area_dict(-10, -40, 110, 60)))
    pings = sc.lawn(0, 0, 3, 18.0, 100.0)
    targets = [(25.0, 9.0), (55.0, 27.0), (80.0, -8.0), (40.0, 45.0)]
    arrays = sc.arrays_for(pings, targets, seed=seed)
    sc.feed(b, sc.events(pings, sc.odom_for(pings), sc.fixes_for(pings, noise_m=0.3, seed=seed), arrays))
    b.finish()
    return b, targets


def section_determinism() -> None:
    from blueboat_belief.core.config import BeliefConfig
    from blueboat_belief.core.contract import to_grid_dict
    from blueboat_belief.io.snapshot import encode_npz, manifest_for

    print("[12] determinism: the same input sequence twice -> byte-identical snapshots (T12)")
    b1, _ = _run_scenario()
    b2, _ = _run_scenario()
    n1, n2 = encode_npz(to_grid_dict(b1, 1)), encode_npz(to_grid_dict(b2, 1))
    check("snapshot .npz byte-identical on replay", n1 == n2, f"{len(n1)} bytes")
    m1 = manifest_for(to_grid_dict(b1, 1), n1, b1.cfg.to_flat(), b1.counters, b1.stats)
    m2 = manifest_for(to_grid_dict(b2, 1), n2, b2.cfg.to_flat(), b2.counters, b2.stats)
    check("manifest identical (no wall clock outside provenance)", json.dumps(m1, sort_keys=True) == json.dumps(m2, sort_keys=True))
    cfg_a = BeliefConfig.from_dict({"look": {"timeout_s": 20.0}, "grid": {"resolution_m": 0.5}})
    cfg_b = BeliefConfig.from_dict({"grid": {"resolution_m": 0.5}, "look": {"timeout_s": 20.0}})
    b3, _ = _run_scenario(cfg=cfg_a)
    b4, _ = _run_scenario(cfg=cfg_b)
    check("insensitive to config key order", encode_npz(to_grid_dict(b3, 1)) == encode_npz(to_grid_dict(b4, 1)))
    b5, _ = _run_scenario(seed=8)
    check("a different detection draw changes the snapshot (the check is not vacuous)",
          encode_npz(to_grid_dict(b5, 1)) != n1)


# ============================================================================ [13] localisation
def section_localisation() -> None:
    print("[13] localisation: synthetic objects from one pass within 2 m (T12), no duplicates")
    b, targets = _run_scenario()
    cs = sorted(b.contacts.contacts.values(), key=lambda c: c.uid)
    errs = []
    for tx, ty in targets:
        d = min(math.hypot(c.x - tx, c.y - ty) for c in cs) if cs else math.inf
        errs.append(d)
    check("every target localised within 2 m", max(errs) <= 2.0, "errors " + ", ".join(f"{e:.2f}" for e in errs))
    check("one contact per target (image overlap merged within the look)", len(cs) == len(targets),
          f"{len(cs)} contacts")
    cov = b.grid.coverage()
    check("coverage reached the placeholder plateau on the swath", float(np.nanmax(cov)) >= 0.69)
    check("geometry-only pings exist where arrays did not bracket", b.stats["pings_geometry_only"] > 0
          and b.stats["pings_covered"] > b.stats["pings_geometry_only"])


# ============================================================================ [14] rejections
def section_rejections() -> None:
    from blueboat_belief.core.area import area_from_dict
    from blueboat_belief.core.belief import (REASONS, Belief, Det, DetArray, FixObs, ImageRec,
                                             OdomObs, PingObs)
    from blueboat_belief.core.config import BeliefConfig
    from blueboat_belief.ros import adapters as ad
    from test import scenarios as sc

    print("[14] rejections: every bad-input path counted, no exception escapes")
    area = area_from_dict(sc.area_dict(-10, -40, 110, 40))
    cfg = BeliefConfig().with_values(detector={"class_names": ["block", "pipe"]},
                                     alignment={"queue_max": 5})
    b = Belief(cfg, area)
    pings = sc.straight_pass(0, 0, 0.0, 1.0, 30.0)
    for p in pings[:8]:
        b.ingest_ping(p)                                         # alignment_pending, overflow
    for p in pings:
        b.ingest_odom(OdomObs(p.t, p.x, p.y, p.yaw))
    for f in sc.fixes_for(pings, every=1):
        b.ingest_fix(f)
    b.ingest_fix(FixObs(200.0, 0.0, 0.0, 0))                     # no_fix (0, 0)
    b.ingest_fix(FixObs(200.0, 33.9, 130.9, -1))                 # no_fix (status -1)
    for p in pings[8:]:
        b.ingest_ping(p)
    P0 = pings[0]
    rep = lambda **kw: PingObs(**{**P0.__dict__, **kw})  # noqa: E731
    b.ingest_ping(rep(t=float("nan")))                           # nan_input
    b.ingest_ping(rep(t=pings[-1].t + 1, water_depth=0.22))      # altitude_unknown
    b.ingest_ping(rep(t=pings[-1].t + 1.1, port_ping_number=0, starboard_ping_number=0))  # empty_ping
    b.ingest_odom(OdomObs(float("inf"), 0, 0, 0))                # nan_input
    R = np.eye(2)
    t_last = pings[-1].t
    dets = (
        Det("1", t_last - 5, 30.0, 10.0, 0.0, "dummy_center", 0.5, R),        # ignored_class
        Det("2", t_last - 5, 500.0, 10.0, 0.0, "block", 0.5, R),              # outside_area
        Det("3", t_last - 6, 20.0, 10.0, 0.0, "block", 0.5, np.full((2, 2), np.nan)),  # bad_covariance
        Det("4", t_last + 50, 20.0, 10.0, 0.0, "block", 0.5, R),              # future_detection
        Det("5", 1.0, 20.0, 10.0, 0.0, "block", 0.5, R),                      # stale_detection / no pose
        Det("6", t_last - 7, 25.0, 12.0, 0.0, "nope", 0.5, R),                # unknown_class
    )
    b.ingest_detections(DetArray(t_last, dets))
    b.ingest_detections(DetArray(t_last + 0.5, (dets[2],)))                 # duplicate_detection
    b.ingest_detections(DetArray(0.0, ()))                                  # zero_stamp_array
    b.ingest_ping(rep(t=t_last + 200.0, x=1.0))                             # pending, then flushed
    b.ingest_ping(rep(t=t_last + 100.0, x=2.0))                             # time_reset (backwards)
    b.ingest_ping(rep(t=t_last + 170.0, x=3.0))                             # +100 ping expires (no array)
    b.ingest_detections(DetArray(t_last + 171.0, (Det("7", t_last + 170.5, 30.0, 10.0, 0.0, "block", 0.5, R),)))
    b.ingest_detections(DetArray(t_last + 172.0, (Det("8", t_last + 90.0, 30.0, 10.0, 0.0, "block", 0.5, R),)))
    b.ingest_record(ImageRec("s/0", float("nan"), t_last, True))            # bad_record (no span)
    b.ingest_record(ImageRec("s/1", t_last + 5.0, t_last, True))            # bad_record (t_min > t_max)
    b.reject_record()                                                       # bad_record (unreadable)
    b.finish()
    for p in [rep(t=t_last + 300 + i, x=0.0, y=0.0) for i in range(25)]:
        b.ingest_ping(p)                                                    # frozen_pose
    missing = [r for r in REASONS if b.counters[r] == 0]
    check("every rejection reason was triggered and counted", not missing, ", ".join(missing))
    check("frozen pose -> ERROR in health", b.health()[0] == "ERROR")
    # fuzz: malformed inputs never raise
    rng = np.random.default_rng(0)
    ok = True
    try:
        junk = ("", "{", "null", "[1, 2]", '{"schema": "x"}', '{"schema": "blueboat_perception/image_record/1"}',
                '{"schema": "blueboat_perception/image_record/1", "time": {"t_min_s": null, "t_max_s": 3}, "processing": {}}',
                '{"schema": "blueboat_perception/image_record/1", "time": {"t_min_s": NaN, "t_max_s": 3}, "processing": 7}',
                '{"schema": "blueboat_perception/image_record/1", "time": {"t_min_s": true, "t_max_s": "3"}, "processing": {}}',
                b"\xff\xfe", {"data": 5}, {"data": None}, 12, None)
        for j in junk:                                           # image records: never raise
            rec = ad.record_from_msg(j if isinstance(j, dict) else {"data": j})
            b.reject_record() if rec is None else b.ingest_record(rec)
        for _ in range(200):
            k = rng.integers(0, 4)
            if k == 3:
                b.ingest_record(ImageRec("f", float(rng.uniform(-5, 1e3)), float(rng.uniform(-5, 1e3)),
                                         bool(rng.integers(0, 2)), bool(rng.integers(0, 2))))
            elif k == 0:
                n = int(rng.integers(0, 5))
                b.ingest_ping(rep(t=float(rng.uniform(0, 1e3)), port_y=rng.normal(size=n).astype(np.float32),
                                  port_db=rng.normal(size=int(rng.integers(0, 5))).astype(np.float32)))
            elif k == 1:
                b.ingest_detections(DetArray(float(rng.uniform(-5, 1e3)), (Det("z", float(rng.uniform(0, 1e3)),
                                    float(rng.normal(0, 1e3)), float(rng.normal()), 0.0, "block",
                                    float(rng.uniform(-1, 2)), rng.normal(size=(2, 2))),)))
            else:
                b.ingest_fix(FixObs(float(rng.uniform(0, 1e3)), float(rng.normal()), float(rng.normal()), int(rng.integers(-3, 3))))
    except Exception as exc:  # noqa: BLE001 - the check is that nothing escapes
        ok = False
        print("   ", type(exc).__name__, exc)
    check("fuzzed malformed inputs: no exception escapes", ok)


# ============================================================================ [15] performance
def section_performance() -> None:
    from blueboat_belief.core.area import area_from_dict
    from blueboat_belief.core.belief import Belief
    from blueboat_belief.core.config import BeliefConfig
    from blueboat_belief.core.contract import to_grid_dict
    from blueboat_belief.io.snapshot import encode_npz
    from test import scenarios as sc

    print("[15] performance (indicative: one run on this machine, under whatever load it has)")
    b = Belief(BeliefConfig().with_values(coverage={"require_detector": False}),
               area_from_dict(sc.area_dict(-10, -40, 110, 40)))
    pings = sc.straight_pass(0, 0, 0.3, 1.0, 50.0, h=0.7)
    g = b.grid
    t0 = time.perf_counter()
    for p in pings:
        for side, yoff, db in ((1, p.port_y, p.port_db), (-1, p.starboard_y, p.starboard_db)):
            res = g.side_cells(p.x, p.y, p.yaw, np.asarray(yoff, np.float64), np.asarray(db, np.float64))
            if res:
                g.observe_side(p.t, 0, p.yaw, side, res[0], res[1], res[2], p.water_depth, True)
    per = (time.perf_counter() - t0) / len(pings) * 1e3
    check("grid update < 1 ms per swath at 0.5 m (2 x ~1170 samples)", per < 1.0, f"{per * 1000:.0f} us/swath")
    sc.feed(b, sc.events(pings, sc.odom_for(pings), sc.fixes_for(pings)))
    d = to_grid_dict(b, 1)
    t0 = time.perf_counter()
    blob = encode_npz(d)
    enc = (time.perf_counter() - t0) * 1e3
    t0 = time.perf_counter()
    to_grid_dict(b, 2)
    gd = (time.perf_counter() - t0) * 1e3
    print(f"    info: snapshot encode {enc:.1f} ms ({len(blob) / 1e6:.2f} MB, {int(d['width'])}x{int(d['height'])} cells); "
          f"to_grid_dict {gd:.1f} ms")
    check("snapshot encode measured", enc > 0)


# ============================================================================ [16] snapshot codec
def section_snapshot() -> None:
    from blueboat_belief.core.contract import to_grid_dict
    from blueboat_belief.io.snapshot import read_snapshot, write_snapshot

    print("[16] snapshot files: O_EXCL claim, read-back, manifest")
    b, _ = _run_scenario()
    d = to_grid_dict(b, 5)
    with tempfile.TemporaryDirectory() as tmp:
        n1, m1 = write_snapshot(Path(tmp), "snapshot_final", d, b.cfg.to_flat(), b.counters, b.stats,
                                input_recording="synthetic", provenance={"wall_utc": "x"})
        n2, m2 = write_snapshot(Path(tmp), "snapshot_final", d, b.cfg.to_flat(), b.counters, b.stats)
        check("second write never overwrites: '-2' suffix", n1.name == "snapshot_final.npz"
              and n2.name == "snapshot_final-2.npz" and m2.name == "snapshot_final-2.json")
        r = read_snapshot(n1)
        same = all(np.array_equal(r[k], d[k], equal_nan=True) for k in ("p_background", "coverage", "aspect_counts", "n_pings"))
        check("read-back arrays equal", same and len(r["contacts"]) == len(d["contacts"]))
        man = json.loads(m1.read_text())
        check("manifest keys (schema, contract, model params, config, counters, input, sha256)",
              {"schema", "contract_version", "model_params", "config", "counters", "stats",
               "input_recording", "npz_sha256", "provenance"} <= set(man))


# ============================================================================ [17] packaging
def section_packaging() -> None:
    print("[17] packaging, launch helpers, node file")
    setup = (ROOT / "setup.py").read_text()
    check("console scripts: belief_node, blueboat-belief-replay, blueboat-belief-make-area",
          'f"belief_node = {PACKAGE}.ros.belief_node:main"' in setup and 'PACKAGE = "blueboat_belief"' in setup
          and 'f"blueboat-belief-replay = {PACKAGE}.cli.replay:main"' in setup
          and 'f"blueboat-belief-make-area = {PACKAGE}.cli.make_area:main"' in setup)
    pkg = (ROOT / "package.xml").read_text()
    check("package.xml depends on both interfaces packages and vision_msgs",
          all(f"<depend>{d}</depend>" in pkg for d in ("blueboat_belief_interfaces", "blueboat_interfaces", "vision_msgs")))
    spec = importlib.util.spec_from_file_location("belief_launch", ROOT / "launch" / "belief_launch.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    check("launch imports offline; record command covers the four inputs and the image records",
          mod.record_command("/tmp/x")[-5:] == list(mod.INPUT_TOPICS) and len(mod.INPUT_TOPICS) == 5
          and mod.INPUT_TOPICS[-1] == "/sss_ai/image_records")
    from blueboat_belief.cli import replay as _rp
    check("the replay CLI reads exactly the topics the launch file records",
          set(_rp.TOPICS) == set(mod.INPUT_TOPICS))
    node = (ROOT / "blueboat_belief" / "ros" / "belief_node.py").read_text()
    for needle in ('"/sss_processor/processed"', '"/sss_ai/detections"', '"/blueboat/odom"',
                   '"/mavros/global_position/global"', '"/sss_ai/image_records"', '"~/belief_grid"',
                   '"~/query_belief"', "DurabilityPolicy.TRANSIENT_LOCAL", "ReliabilityPolicy.BEST_EFFORT"):
        check(f"node declares {needle}", needle in node)
    check("node: detections are subscribed before image records (array handled first in a tie)",
          node.index('"/sss_ai/detections"') < node.index('"/sss_ai/image_records", self._on_record'))
    check("node: outputs paced by a steady clock; SIGINT/SIGTERM are normal stops; fatal start exits non-zero",
          "ClockType.STEADY_TIME" in node and "ExternalShutdownException" in node
          and "sys.exit(code)" in node and "dynamic_typing=True" in node)
    check("node: grid_seq advances only where the BeliefGrid is published",
          node.count("self.seq += 1") == 1 and "def _grid_dict" not in node)
    from blueboat_belief.core.config import BeliefConfig as _BC
    from blueboat_belief.ros.adapters import unknown_parameters
    decl = _BC().to_flat()
    check("a misspelt parameter name is reported; ROS's own parameters are not",
          unknown_parameters({"coverage.requir_detector": 1, "use_sim_time": 1, "pd.plateau": 1,
                              "qos_overrides./tf.publisher.depth": 1}, decl) == ["coverage.requir_detector"])
    check("launch records with --topics (positional topic names are deprecated in Jazzy)",
          "--topics" in mod.record_command("/tmp/x"))
    ip = (ROOT.parent / "blueboat_belief_interfaces" / "CMakeLists.txt").read_text()
    check("interfaces CMake generates the three types", all(s in ip for s in
          ('"msg/BeliefGrid.msg"', '"msg/BeliefContact.msg"', '"srv/QueryBelief.srv"')))


# ============================================================================ [18] bag replay
def _write_bag(ts, path: Path, evs) -> None:
    from rosbags.rosbag2 import StoragePlugin, Writer
    from blueboat_belief.ros import adapters as ad

    topics = {"ping": ("/sss_processor/processed", "blueboat_interfaces/msg/ProcessedSSSPing"),
              "dets": ("/sss_ai/detections", "vision_msgs/msg/Detection2DArray"),
              "odom": ("/blueboat/odom", "nav_msgs/msg/Odometry"),
              "fix": ("/mavros/global_position/global", "sensor_msgs/msg/NavSatFix")}
    with Writer(path, version=9, storage_plugin=StoragePlugin.MCAP) as w:
        conns = {k: w.add_connection(t, m, typestore=ts) for k, (t, m) in topics.items()}
        for arrival, order, kind, o in evs:
            if kind == "ping":
                f = {"port_stamp": ad.s_to_stamp(o.t), "starboard_stamp": ad.s_to_stamp(o.t),
                     "port_ping_number": o.port_ping_number, "starboard_ping_number": o.starboard_ping_number,
                     "robot_x": o.x, "robot_y": o.y,
                     "robot_orientation": {"z": math.sin(o.yaw / 2), "w": math.cos(o.yaw / 2)},
                     "water_depth": o.water_depth, "transducer_x_offset": o.transducer_x_offset,
                     "port_intensity_db": o.port_db, "port_y": o.port_y,
                     "starboard_intensity_db": o.starboard_db, "starboard_y": o.starboard_y}
            elif kind == "odom":   # zero stamp, as on the real launch: the log time is used
                f = {"pose": {"pose": {"position": {"x": o.x, "y": o.y},
                                       "orientation": {"z": math.sin(o.yaw / 2), "w": math.cos(o.yaw / 2)}}}}
            elif kind == "fix":
                f = {"header": {"stamp": ad.s_to_stamp(o.t)}, "status": {"status": o.status},
                     "latitude": o.lat, "longitude": o.lon}
            else:
                f = {"header": {"stamp": ad.s_to_stamp(o.t), "frame_id": "odom"},
                     "detections": [{"header": {"stamp": ad.s_to_stamp(d.t_row)}, "id": d.id,
                                     "bbox": {"center": {"position": {"x": d.x, "y": d.y}, "theta": d.theta}},
                                     "results": [{"hypothesis": {"class_id": d.class_name, "score": d.score},
                                                  "pose": {"pose": {"position": {"x": d.x, "y": d.y}},
                                                           "covariance": [d.cov[0, 0], d.cov[0, 1], 0, 0, 0, 0,
                                                                          d.cov[1, 0], d.cov[1, 1]] + [0.0] * 28}}]}
                                    for d in o.detections]}
            topic, mt = topics[kind]
            msg = materialize(ts, mt, f)
            w.write(conns[kind], int(round(arrival * 1e9)), ts.serialize_cdr(msg, mt))


def section_bag_replay() -> None:
    print("[18] offline replay of a recorded mcap bag (rosbags): deterministic, = direct feeding")
    if not importlib.util.find_spec("rosbags"):
        skip("bag replay", "rosbags not installed")
        return
    from blueboat_belief.cli.replay import replay
    from blueboat_belief.core.area import area_from_dict
    from blueboat_belief.core.belief import Belief, OdomObs
    from blueboat_belief.core.config import BeliefConfig
    from blueboat_belief.core.contract import to_grid_dict
    from blueboat_belief.io.snapshot import encode_npz
    from test import scenarios as sc

    ts = _typestore()
    area = area_from_dict(sc.area_dict(-10, -40, 110, 60))
    pings = sc.lawn(0, 0, 2, 18.0, 60.0)
    arrays = sc.arrays_for(pings, [(25.0, 9.0), (40.0, 27.0)])
    fixes = [f.__class__(t=f.t + 0.002, lat=f.lat, lon=f.lon, status=f.status) for f in sc.fixes_for(pings)]
    evs = sc.events(pings, sc.odom_for(pings), fixes, arrays)
    # direct feeding with exactly what the bag will deliver: odom stamped by its log time
    direct = Belief(BeliefConfig(), area)
    sc.feed(direct, [(a, o, k, OdomObs(a, x.x, x.y, x.yaw) if k == "odom" else x) for a, o, k, x in evs])
    direct.finish()
    ref = encode_npz(to_grid_dict(direct, 1))
    with tempfile.TemporaryDirectory() as tmp:
        bag = Path(tmp) / "inputs"
        _write_bag(ts, bag, evs)
        outs = []
        for _ in range(2):
            b = Belief(BeliefConfig(), area)
            n = replay(str(bag), b, [str(ROOT / "test" / "msg_reference")])
            outs.append(encode_npz(to_grid_dict(b, 1)))
        check("bag has every event", n == len(evs), f"{n} messages")
    check("replay twice: byte-identical snapshots", outs[0] == outs[1])
    check("replay = direct feeding (adapters and log-time order are exact)", outs[0] == ref)


# ============================================================================ [19] turns
def _arc(R=6.0, v=1.0, frac=0.5, rate=20.0, t0=100.0, h=2.0, left=True):
    from blueboat_belief.core.belief import PingObs
    from test import scenarios as sc

    py, sy = sc.side_offsets(h, +1), sc.side_offsets(h, -1)
    pdb, sdb = np.full(py.size, -20, np.float32), np.full(sy.size, -20, np.float32)
    w = v / R
    s = 1.0 if left else -1.0
    out = []
    for i in range(int(frac * 2 * math.pi / w * rate) + 1):
        t = t0 + i / rate
        th = w * (t - t0)
        out.append(PingObs(t, R * math.sin(th), s * (R - R * math.cos(th)), s * th, h + 0.22,
                           i + 1, i + 1, py, pdb, sy, sdb))
    return out


def section_turns() -> None:
    from blueboat_belief.core.area import area_from_dict
    from blueboat_belief.core.belief import Belief
    from blueboat_belief.core.config import BeliefConfig, TurnCfg
    from blueboat_belief.core.turn import TurnEstimator
    from test import scenarios as sc

    print("[19] turn back-scan gating (Paull 2013 §II-C; r* = v/|w|, MATH §4.6)")
    te = TurnEstimator(TurnCfg())
    for p in _arc()[:40]:
        om, v, r, inner = te.update(p.t, p.x, p.y, p.yaw)
    check("estimator on a left arc R = 6 m, v = 1 m/s: w = 1/6, r* = 6 m, inner = port",
          abs(om - 1 / 6) < 1e-3 and abs(r - 6.0) < 0.06 and inner == 1, f"w={om:.4f} r*={r:.3f}")
    te = TurnEstimator(TurnCfg())
    for p in _arc(left=False)[:40]:
        om, v, r, inner = te.update(p.t, p.x, p.y, p.yaw)
    check("right arc: inner = starboard", inner == -1 and om < 0)
    te = TurnEstimator(TurnCfg())
    for p in sc.straight_pass(0, 0, 0.4, 1.0, 3.0):
        om, v, r, inner = te.update(p.t, p.x, p.y, p.yaw)
    check("straight leg: not turning (r* = inf)", inner == 0 and math.isinf(r))
    te = TurnEstimator(TurnCfg())
    for i in range(30):
        om, v, r, inner = te.update(100 + i / 20, 0.0, 0.0, 0.2 * i / 20)
    check("pivot turn (v = 0): r* = 0, the whole inner side back-scanned", r == 0.0 and inner == 1)
    te = TurnEstimator(TurnCfg())
    te.update(100.0, 0.0, 0.0, 0.0)
    te.update(100.05, 0.05, 0.0, 0.0)
    om, v, r, inner = te.update(105.0, 0.0, 18.0, math.pi)    # next leg after a 5 s gap
    check("a gap longer than the window restarts the estimate (no turn across lawnmower gaps)",
          inner == 0 and math.isinf(r))
    area = area_from_dict(sc.area_dict(-30, -30, 30, 40))
    res = {}
    for en in (True, False):
        cfg = BeliefConfig().with_values(coverage={"require_detector": False}, turn={"enabled": en},
                                         alignment={"origin_lat": sc.LAT0, "origin_lon": sc.LON0})
        b = Belief(cfg, area)
        for p in _arc():
            b.ingest_ping(p)
        cx, cy = b.geom.centres()
        far = (cx < -1.5) & (cy > 1.0) & (cy < 11.0)       # beyond the turn centre (0, 6)
        res[en] = (b, int((b.grid.n_looks[far] > 0).sum()), int((b.grid.n_aspects() >= 2).sum()))
    check("gated: no cell beyond the turn centre gets a look (inner side beyond r*)", res[True][1] == 0
          and res[False][1] > 100, f"gated {res[True][1]}, ungated {res[False][1]}")
    check("gated: fewer multi-aspect cells than ungated (no fake aspects from one turn)",
          res[True][2] < res[False][2], f"{res[True][2]} vs {res[False][2]}")
    st = res[True][0].stats
    check("back-scan counted in stats", st["backscan_pings"] > 0 and st["backscan_samples"] > 0)
    b = Belief(BeliefConfig().with_values(coverage={"require_detector": False},
                                          alignment={"origin_lat": sc.LAT0, "origin_lon": sc.LON0}), area)
    for p in sc.straight_pass(-20, 0, 0.0, 1.0, 30.0):
        b.ingest_ping(p)
    check("straight pass: nothing dropped", b.stats["backscan_samples"] == 0)


# ============================================================================ [20] tools, summary
def section_tools() -> None:
    from blueboat_belief.cli.make_area import main as make_area
    from blueboat_belief.core.area import area_from_dict
    from blueboat_belief.io.missions import load_area_file
    from test import scenarios as sc

    print("[20] survey-area tool and run summary (Bourgault 2006 Eq. 7)")
    if not importlib.util.find_spec("yaml"):
        skip("survey-area tool", "PyYAML absent")
        return
    with tempfile.TemporaryDirectory() as tmp:
        out = Path(tmp) / "area.yaml"
        rc = make_area(["--out", str(out), "--anchor", str(sc.LAT0), str(sc.LON0), "--local", "-20", "-10", "80", "50",
                        "--name", "harbour", "--created", "2026-11-04T00:00:00Z"])
        a = load_area_file(str(out))
        check("make-area from an anchor: valid file, exact polygon", rc == 0 and a.name == "harbour"
              and np.allclose(a.polygon_xy, [[-20, -10], [80, -10], [80, 50], [-20, 50]]))
        before = out.read_bytes()
        rc2 = make_area(["--out", str(out), "--anchor", "0", "0", "--local", "0", "0", "5", "5"])
        check("make-area never overwrites (exit 1, file unchanged)", rc2 == 1 and out.read_bytes() == before)
        rc3 = make_area(["--out", str(Path(tmp) / "bad.yaml"), "--anchor", "33.9", "130.9", "--local", "5", "0", "1", "5"])
        check("make-area refuses an invalid rectangle (exit 2, nothing written)", rc3 == 2
              and not (Path(tmp) / "bad.yaml").exists())
        wdir = Path(tmp) / "world"
        wdir.mkdir()
        import yaml
        world = sc.area_dict(-15, -25, 35, 45, fmt="blueboat_world_meta/1", objects=[{"type": "tyre", "x": 1}])
        (wdir / "metadata.yaml").write_text(yaml.safe_dump(world))
        rc4 = make_area(["--out", str(Path(tmp) / "fromworld.yaml"), "--from-world", str(wdir)])
        fw = load_area_file(str(Path(tmp) / "fromworld.yaml"))
        check("make-area --from-world copies geo_anchor and limits (no ground truth)",
              rc4 == 0 and fw.lat0 == world["geo_anchor"]["lat0"] and fw.local == (-15.0, -25.0, 35.0, 45.0)
              and "objects" not in (Path(tmp) / "fromworld.yaml").read_text())
        # from a BlueBoat-Replanner mission file (its config/mission_example.yaml, verbatim polygon)
        from blueboat_belief.core.geo import latlon_to_local_en
        poly = [[34.5, 133.5], [34.5, 133.5013], [34.50054, 133.5013], [34.50054, 133.5]]
        mission = {"mission": {"name": "devloop_example", "condition": "C3", "areas": [poly],
                               "config": {"planning_depth_m": 2.0}}}
        mfile = Path(tmp) / "mission.yaml"
        mfile.write_text(yaml.safe_dump(mission))
        rc5 = make_area(["--out", str(Path(tmp) / "frommission.yaml"), "--from-mission", str(mfile)])
        fm = load_area_file(str(Path(tmp) / "frommission.yaml"))
        en = [latlon_to_local_en(la, lo, 34.5, 133.5) for la, lo in poly]
        ex, ny = max(e for e, _ in en), max(n for _, n in en)
        check("make-area --from-mission: the area is the mission polygon's bounding rectangle",
              rc5 == 0 and fm.name == "devloop_example" and (fm.lat0, fm.lon0) == (34.5, 133.5)
              and fm.local[:2] == (0.0, 0.0) and 0 <= fm.local[2] - ex < 0.01 and 0 <= fm.local[3] - ny < 0.01,
              f"{fm.local[2]:.2f} x {fm.local[3]:.2f} m")
        rc6 = make_area(["--out", str(Path(tmp) / "margin.yaml"), "--from-mission", str(mfile), "--margin-m", "20",
                         "--anchor", "34.5001", "133.5002"])
        mg = load_area_file(str(Path(tmp) / "margin.yaml"))
        check("--margin-m grows it on every side; --anchor moves the datum, not the area",
              rc6 == 0 and (mg.lat0, mg.lon0) == (34.5001, 133.5002)
              and close((mg.local[2] - mg.local[0]) - (fm.local[2] - fm.local[0]), 40.0, 1e-3)
              and close((mg.local[3] - mg.local[1]) - (fm.local[3] - fm.local[1]), 40.0, 1e-3))
        bad = {"mission": {"name": "two", "areas": [poly, poly]}}
        (Path(tmp) / "two.yaml").write_text(yaml.safe_dump(bad))
        rc7 = make_area(["--out", str(Path(tmp) / "two_out.yaml"), "--from-mission", str(Path(tmp) / "two.yaml")])
        rc8 = make_area(["--out", str(Path(tmp) / "both.yaml"), "--from-mission", str(mfile), "--from-world", str(wdir)])
        check("a mission with two areas, or two sources at once, is refused (exit 2, nothing written)",
              rc7 == 2 and rc8 == 2 and not (Path(tmp) / "two_out.yaml").exists()
              and not (Path(tmp) / "both.yaml").exists())
    b, _ = _run_scenario()
    cov = b.grid.coverage().astype(np.float64)[b.inside]
    q = -np.log1p(-np.clip(cov, 0, 1 - 1e-12))
    pk = 1.0 - float(np.mean(np.exp(-q)))
    check("area_cumulative_pd = 1 - sum pi_i Q_i (uniform pi) = mean in-area coverage",
          abs(b.area_cumulative_pd() - pk) < 1e-9 and 0 < pk < 1, f"P = {pk:.4f}")
    s = b.summary()
    check("summary: cumulative PD, contacts, confirmed, retired, alignment, range of the fore/aft arms",
          set(s) == {"area_cumulative_pd", "contacts", "contacts_confirmed", "contacts_retired", "alignment_valid",
                     "transducer_x_offset_min_m", "transducer_x_offset_max_m"})


# ============================================================================ [21] image records
def _records_run(source="auto", records=True, record_first=False, searched=True, dummy=False,
                 with_arrays=True, speed=0.5, finish=True, stop_after=None):
    """One straight 100 m pass with one detection array (and one image record) per image."""
    from blueboat_belief.core.area import area_from_dict
    from blueboat_belief.core.belief import Belief
    from blueboat_belief.core.config import BeliefConfig
    from test import scenarios as sc

    cfg = BeliefConfig().with_values(alignment={"origin_lat": sc.LAT0, "origin_lon": sc.LON0},
                                     coverage={"source": source})
    b = Belief(cfg, area_from_dict(sc.area_dict(-30, -30, 130, 30)))
    pings = sc.straight_pass(0.0, 0.0, 0.0, speed, 100.0 / speed)
    period = 2.135 / speed                       # the GCS image stride: 128 rows of 16.68 mm
    arrays = sc.arrays_for(pings, [(40.0, 8.0)], period_s=period)
    recs = sc.records_for(pings, arrays, period_s=period, searched=searched, dummy=dummy) if records else []
    evs = sc.events(pings, sc.odom_for(pings), (), arrays if with_arrays else (), records=recs,
                    record_first=record_first)
    if stop_after is not None:
        evs = [e for e in evs if e[0] <= stop_after]
    sc.feed(b, evs)
    if finish:
        b.finish()
    return b, pings, arrays, recs


def section_records() -> None:
    from blueboat_belief.core.contract import to_grid_dict
    from blueboat_belief.core.coverage import RecordTimeline
    from blueboat_belief.io.snapshot import encode_npz
    from blueboat_belief.ros import adapters as ad

    print("[21] image records as the optional coverage source (MATH §8.2; owner 2026-11-12)")
    mp = lambda b: json.loads(to_grid_dict(b, 1)["model_params_json"])  # noqa: E731
    snap = lambda b: encode_npz(to_grid_dict(b, 1))                     # noqa: E731

    b_arr, pings, arrays, _ = _records_run(source="arrays", records=False)
    b_rec, _, _, recs = _records_run()
    head = sum(1 for p in pings if p.t <= arrays[0].t)
    check("auto with records: coverage_source = records; without: arrays",
          mp(b_rec)["coverage_source"] == "records" and mp(b_arr)["coverage_source"] == "arrays")
    check("array heuristic: the first image is never credited (its pings stay geometry-only)",
          b_arr.stats["pings_geometry_only"] >= head > 0, f"{head} pings in the first image")
    check("records: exactly the first image's pings gain miss credit",
          b_rec.stats["pings_covered"] - b_arr.stats["pings_covered"] == head
          and b_rec.stats["pings"] == b_arr.stats["pings"] == len(pings),
          f"covered {b_arr.stats['pings_covered']} -> {b_rec.stats['pings_covered']} of {len(pings)}")
    check("records: every record searched, none waited for a missing array, no overlap violation",
          b_rec.stats["records"] == b_rec.stats["records_searched"] == len(recs)
          and b_rec.stats["records_without_array"] == 0 and b_rec.stats["records_overlap"] == 0)
    check("records: the detection still makes one contact (detections come from the arrays only)",
          len(b_rec.contacts.contacts) == 1 and len(b_arr.contacts.contacts) == 1)
    check("calibration flag observed from the records; null when no record was seen",
          mp(b_rec)["detector_calibrated_observed"] is False and mp(b_arr)["detector_calibrated_observed"] is None
          and mp(b_rec)["detector_calibrated"] is False)

    b_first, *_ = _records_run(record_first=True)
    check("record before its array, or after: byte-identical belief (MATH §8.2 item 3)",
          snap(b_first) == snap(b_rec) and b_first.stats == b_rec.stats)
    b_again, *_ = _records_run()
    check("deterministic with records: byte-identical on repeat", snap(b_again) == snap(b_rec))

    # two consecutive images without pixels: the pings only they contain are not searched
    gap = lambda k: k not in (5, 6)                                    # noqa: E731
    b_null, *_ = _records_run(searched=gap)
    only = sum(1 for p in pings if arrays[4].t < p.t <= arrays[5].t)
    check("two images with detections: null -> the pings only they hold lose their miss credit",
          b_rec.stats["pings_covered"] - b_null.stats["pings_covered"] == only > 0
          and b_null.stats["records_unsearched"] == 2, f"{only} pings")
    check("the array heuristic cannot see it: the same empty arrays are credited",
          b_arr.stats["pings_covered"] == len(pings) - head - sum(1 for p in pings if p.t > arrays[-1].t))

    b_dummy, *_ = _records_run(dummy=True)
    check("records from a dummy detector: nothing is searched, coverage stays 0",
          b_dummy.stats["pings_covered"] == 0 and b_dummy.stats["records_unsearched"] == len(recs)
          and b_dummy.area_cumulative_pd() == 0.0)

    b_only, *_ = _records_run(source="arrays")
    check("coverage.source = arrays ignores records (counted) and equals the arrays-only belief",
          snap(b_only) != b"" and b_only.stats["records"] == len(recs)
          and b_only.stats["pings_covered"] == b_arr.stats["pings_covered"]
          and mp(b_only)["coverage_source"] == "arrays")
    b_auto, *_ = _records_run(source="auto", records=False)
    check("auto without records = arrays, byte for byte (detections-only publishers are unaffected)",
          snap(b_auto) == snap(b_arr))
    b_strict, *_ = _records_run(source="records", records=False)
    check("coverage.source = records with no record: no ping is ever credited",
          b_strict.stats["pings_covered"] == 0 and mp(b_strict)["coverage_source"] == "records")
    b_noarr, *_ = _records_run(with_arrays=False)
    check("records whose arrays never come are applied one record late, and counted",
          b_noarr.stats["records_without_array"] == len(recs)
          and b_noarr.stats["pings_covered"] == b_rec.stats["pings_covered"])

    # release bound and the neighbours-only overlap assumption
    tl = RecordTimeline()
    tl.add(0.0, 10.0, True)
    b1 = tl.release_bound
    tl.add(5.0, 15.0, True)
    b2 = tl.release_bound
    ok3 = tl.add(10.0, 20.0, True)                                      # t_min(2) == t_max(0): allowed
    check("release bound: nothing after one record, then strictly below t_max of the previous record",
          b1 == float("-inf") and b2 < 10.0 and close(b2, 10.0) and not ok3 and tl.overlap_ok
          and close(tl.release_bound, 15.0) and tl.release_bound < 15.0)
    tl = RecordTimeline()
    for k in range(2):
        tl.add(2.5 * k, 10.0 + 2.5 * k, True)
    broke = tl.add(5.0, 15.0, True)                                     # 75 % overlap: t_min(2) < t_max(0)
    check("an image overlapping more than its neighbours is reported once; the bound falls back to t_min",
          broke and not tl.overlap_ok and tl.release_bound < 5.0 and close(tl.release_bound, 5.0)
          and not tl.add(7.5, 17.5, True))
    check("covered(tau): inside a searched span only", tl.covered(0.0) and tl.covered(17.5)
          and not tl.covered(17.6) and not tl.covered(-0.1))
    tl2 = RecordTimeline()
    tl2.add(0.0, 10.0, False)
    tl2.add(5.0, 15.0, True)
    check("an unsearched image covers nothing; its searched neighbour covers the overlap",
          not tl2.covered(2.0) and tl2.covered(7.0) and tl2.covered(12.0))

    # evidence horizon: everything at or before the header stamp has been applied
    b_live, pings_l, arrays_l, _ = _records_run(finish=False, stop_after=150.0)
    stamp = to_grid_dict(b_live, 1)["header"]["stamp"]
    fed = [p for p in pings_l if p.t + 0.01 <= 150.0]
    check("header.stamp is the evidence horizon: every ping at or before it is in the grid, none after",
          stamp == b_live.last_event_t and b_live.stats["pings"] == sum(1 for p in fed if p.t <= stamp)
          and 0 < b_live.stats["pings"] < len(fed), f"horizon {stamp:.2f} s, fed to {fed[-1].t:.2f} s")
    lag = fed[-1].t - stamp
    stride = 2.135 / 0.5
    check("release latency is between one and two image strides behind the newest ping",
          stride * 0.9 <= lag <= 2.0 * stride + 1.0 + 0.1, f"lag {lag:.2f} s, stride {stride:.2f} s")

    # the wire format: one verbatim record from a real Perception log
    raw = (ROOT / "test" / "data" / "image_record_real.json").read_text(encoding="utf-8")
    rec = ad.record_from_msg(SimpleNamespace(data=raw))
    check("a real Perception record parses: key, row span, searched, dummy, calibrated",
          rec is not None and rec.key == "gcs-20261028T025627Z-9e70/10" and rec.t_min < rec.t_max
          and rec.searched and rec.dummy and rec.calibrated is False and rec.detector == "dummy_center"
          and rec.near_range_mask_m == 0.0, f"{rec.t_max - rec.t_min:.3f} s span" if rec else "")
    d = json.loads(raw)
    same = ad.record_from_msg({"data": raw}) == rec == ad.record_from_msg(d) == ad.record_from_msg(raw)
    d2 = dict(d, detections=None)
    d2["processing"] = dict(d["processing"], detector=dict(d["processing"]["detector"], status="no_pixels"))
    check("String message, key access, decoded dict and raw text agree; detections: null -> not searched",
          same and ad.record_from_msg(d2).searched is False)
    check("another schema, or no time span, is refused (None), never raised",
          ad.record_from_msg(dict(d, schema="blueboat_perception/image_record/2")) is None
          and ad.record_from_msg(dict(d, time={"t_min_s": None, "t_max_s": None})) is None
          and ad.record_from_msg("{not json") is None)


# ============================================================================ [22] consumer semantics
def section_consumer() -> None:
    from blueboat_belief.core.area import area_from_dict
    from blueboat_belief.core.belief import Belief, Det, DetArray
    from blueboat_belief.core.config import BeliefConfig
    from blueboat_belief.core.contract import to_grid_dict
    from blueboat_belief.core.geometry import processor_side_offsets
    from blueboat_belief.ros import adapters as ad
    from test import scenarios as sc

    print("[22] what a consumer may rely on (docs/topics.md §3a; docs/OFFLINE_CORE_API.md)")

    def lawn_run(with_arrays=True):
        cfg = BeliefConfig().with_values(alignment={"origin_lat": sc.LAT0, "origin_lon": sc.LON0,
                                                    "override_sigma_m": 0.0})
        b = Belief(cfg, area_from_dict(sc.area_dict(-10, -30, 110, 60)))
        pings = sc.lawn(0.0, 0.0, legs=3, spacing=16.0, length=100.0, speed=0.5)
        arrays = sc.arrays_for(pings, [(30.0, 8.0), (60.0, 24.0), (80.0, -9.0)], period_s=4.27)
        sc.feed(b, sc.events(pings, sc.odom_for(pings), (), arrays if with_arrays else ()))
        b.finish()
        return b, to_grid_dict(b, grid_seq=7)

    b, d = lawn_run()
    S = b.grid.S
    inside = b.inside
    searched_core = (b.grid.searched[:, None] >> np.arange(S)[None, :]) & 1
    alm = np.asarray(d["aspect_log_miss"]).reshape(-1, S)
    counts = np.asarray(d["aspect_counts"]).reshape(-1, S)
    check("aspect_log_miss < 0 is exactly the set of searched sectors that rho is taken against",
          np.array_equal((alm[inside] < 0), searched_core[inside].astype(bool)))
    seen_not_searched = int(((counts > 0) & ~(alm < 0))[inside].sum())
    check("aspect_counts > 0 is a superset: looks without detector coverage are counted there too",
          bool(np.all((counts[inside] > 0) | ~(alm[inside] < 0))) and seen_not_searched > 0,
          f"{seen_not_searched} (cell, sector) pairs seen but not searched")
    b0, d0 = lawn_run(with_arrays=False)
    a0 = np.asarray(d0["aspect_log_miss"]).reshape(-1, S)
    c0 = np.asarray(d0["aspect_counts"]).reshape(-1, S)
    check("no detector at all: every look is counted, no sector is searched, coverage is 0",
          int((c0[inside] > 0).sum()) > 0 and not bool((a0[inside] < 0).any())
          and float(np.nanmax(d0["coverage"])) == 0.0)

    # a detection whose class is not configured is kept as "other", and counted
    cfg = BeliefConfig().with_values(alignment={"origin_lat": sc.LAT0, "origin_lon": sc.LON0})
    bc = Belief(cfg, area_from_dict(sc.area_dict(-10, -30, 110, 30)))
    pings = sc.straight_pass(0.0, 0.0, 0.0, 0.5, 60.0)
    for p in pings[:141]:
        bc.ingest_ping(p)
    t = pings[100].t
    R = np.eye(2)
    bc.ingest_detections(DetArray(pings[140].t, (Det("1", t, 3.0, 8.0, 0.0, "tyre", 0.8, R),
                                                 Det("2", t, 12.0, 9.0, 0.0, "shopping_cart", 0.8, R))))
    for p in pings[141:]:                       # 53 s more with no further array: released at finish
        bc.ingest_ping(p)
    bc.finish()
    check("a detection released long after its row keeps its pose (resolved at arrival, not at release)",
          pings[-1].t - t > bc.cfg.pose.buffer_s and bc.counters["no_pose_at_stamp"] == 0
          and bc.stats["detections"] == 2, f"released {pings[-1].t - t:.0f} s after its row")
    names = list(bc.cfg.detector.class_names)
    other = [c for c in bc.contacts.to_dicts(names) if int(np.argmax(c["class_probabilities"])) == names.index("other")]
    check("a class outside detector.class_names is kept as 'other' and counted, not rejected",
          bc.stats["detections_class_other"] == 1 and bc.counters["unknown_class"] == 0
          and len(bc.contacts.contacts) == 2 and len(other) == 1)

    # a known limitation, pinned so that it cannot change unnoticed (MATH §9, after item 10)
    from blueboat_belief.core.models import logit_scalar

    tb = _table()
    R1 = np.eye(2) * 1.09
    c1, _ = tb.add_detection(1.0, 0, 3, 0.4, np.array([5.0, 5.0]), R1, "a", "block", 90.0, 15.0)
    tb.add_detection(60.0, 0, 9, 0.9, np.array([5.0, 5.0]), R1, "b", "block", 270.0, 5.0)   # same line
    dd = tb.to_dicts(tuple(names))[0]
    check("contacts are sequential: a stronger later detection on the same line leaves p_exists at the "
          "first score, while state and max_confidence follow it",
          close(c1.L, logit_scalar(0.4)) and int(dd["state"]) == 1 and close(dd["max_confidence"], 0.9, 1e-6)
          and close(dd["first_confidence"], 0.4, 1e-6) and list(dd["sectors_detected"]) == [3, 9])

    check("processor_side_offsets reproduces the processor (1199 - floor(59.95 h) samples, signed, > lever arm)",
          processor_side_offsets(2.0, +1).size == 1199 - int(math.floor(59.95 * 2.0))
          and float(processor_side_offsets(2.0, -1).max()) < -0.285
          and np.array_equal(processor_side_offsets(2.0, +1), sc.side_offsets(2.0, +1)))

    # the example of docs/OFFLINE_CORE_API.md, run as written
    doc = (ROOT / "docs" / "OFFLINE_CORE_API.md").read_text(encoding="utf-8")
    code = doc.split("```python", 1)[1].split("```", 1)[0]
    ns: dict = {}
    exec(compile(code, "OFFLINE_CORE_API.md", "exec"), ns)  # noqa: S102 - our own documentation
    g = ns["grid"]
    check("docs/OFFLINE_CORE_API.md: the documented example runs and yields a contract dict",
          g["contract_version"] == "belief_grid/0" and len(g["contacts"]) == 1
          and float(np.nanmax(g["coverage"])) > 0.5 and bool(g["odom_to_map_valid"]))

    # the run-health surface of docs/OFFLINE_CORE_API.md §5 (Replanner RQ-5): the documented key
    # sets are exactly the code's, so a consumer can assert "nothing dropped" on named keys
    import re as _re
    sec5 = doc.split("## 5. Run health", 1)[1]

    def doc_keys(name):
        para = sec5.split(f"**`Belief.{name}`**", 1)[1].split("Keys:", 1)[1].split("\n\n", 1)[0]
        return set(_re.findall(r"`([a-z_]+)`", para))

    hb = ns["belief"]
    lvl, _msg = hb.health()
    check("docs/OFFLINE_CORE_API.md §5: counters, stats and summary() keys are exactly the documented ones; "
          "health() is (level, message)",
          doc_keys("counters") == set(hb.counters) and doc_keys("stats") == set(hb.stats)
          and doc_keys("summary()") == set(hb.summary()) and lvl in ("OK", "WARN", "ERROR"),
          f"counters {sorted(set(hb.counters) ^ doc_keys('counters'))} "
          f"stats {sorted(set(hb.stats) ^ doc_keys('stats'))} "
          f"summary {sorted(set(hb.summary()) ^ doc_keys('summary()'))}")

    # the real consumer: BlueBoat-Replanner's own validator, when it is checked out beside us
    rep = ROOT.parent.parent / "BlueBoat-Replanner" / "blueboat_replanner"
    if not (rep / "blueboat_replanner" / "core" / "evidence.py").is_file():
        skip("Replanner contract validator on a real Belief grid", "BlueBoat-Replanner not checked out beside this repo")
        return
    old_flag, sys.dont_write_bytecode = sys.dont_write_bytecode, True     # write nothing into its tree
    sys.path.append(str(rep))
    try:
        from blueboat_replanner.core import evidence as r_ev
        from blueboat_replanner.ros import adapters as r_ad

        def as_rclpy(v):
            """Scalars as an rclpy message holds them (python types); arrays stay arrays."""
            if isinstance(v, np.generic):
                return v.item()
            if isinstance(v, dict):
                return {k: as_rclpy(x) for k, x in v.items()}
            if isinstance(v, list):
                return [as_rclpy(x) for x in v]
            return v

        snap = r_ev.validate_grid(r_ad.grid_to_dict(as_rclpy(ad.grid_to_fields(d))))
        check("the Replanner's validate_grid accepts a real Belief grid with no fallback",
              snap.fallbacks == () and all(c.fallbacks == () for c in snap.contacts)
              and snap.coverage.shape == (b.geom.height, b.geom.width)
              and snap.aspect_counts.shape == (b.geom.height, b.geom.width, S)
              and len(snap.contacts) == len(d["contacts"]),
              f"{len(snap.contacts)} contacts, grid {snap.coverage.shape}")
        cov_r = snap.coverage.ravel()
        check("it reads the same cells: coverage and NaN mask equal Belief's, cell for cell",
              np.array_equal(np.isnan(cov_r), ~inside)
              and np.array_equal(cov_r[inside], np.asarray(d["coverage"], dtype=float)[inside]))
        try:
            r_ev.validate_grid(r_ad.grid_to_dict(as_rclpy(ad.grid_to_fields(
                to_grid_dict(Belief(BeliefConfig(), area_from_dict(sc.area_dict(0, 0, 10, 10))), 1)))))
            refused = False
        except r_ev.ContractError:
            refused = True
        check("before odom -> map is valid the Replanner refuses the grid (no plan on an unaligned map)", refused)
    finally:
        sys.path.remove(str(rep))
        sys.dont_write_bytecode = old_flag


# ============================================================================ [23] fore/aft arm
ARM = -0.345                   # the processor's TRANSDUCER_X_OFFSET_M since 2026-11-15 (adopted)


def _one_ping(t, x, y, yaw, port_y, pn, arm):
    """A ping with a single port sample (starboard absent)."""
    from blueboat_belief.core.belief import PingObs

    e = np.zeros(0, np.float32)
    return PingObs(t, x, y, yaw, 2.22, pn, 0, np.array([port_y], np.float32),
                   np.array([0.0], np.float32), e, e, arm)


def section_arm() -> None:
    from blueboat_belief.core.area import area_from_dict
    from blueboat_belief.core.belief import Belief, Det, DetArray, OdomObs, PingObs
    from blueboat_belief.core.config import BeliefConfig, ConfigError, load_yaml
    from blueboat_belief.core.contract import to_grid_dict
    from blueboat_belief.core.geometry import sample_xy, sensor_xy
    from blueboat_belief.io.snapshot import encode_npz
    from blueboat_belief.ros import adapters as ad
    from test import scenarios as sc

    print("[23] fore/aft arm: evidence placed from the transducers' point (MATH §1 items 2, 7; CR-BEL-1)")
    # -- closed form of the placement
    yoff = np.array([2.0, -3.5, 17.25])
    ok, exact0 = True, True
    for yaw in (0.0, math.radians(37.0), math.pi, math.radians(-120.0)):
        c, s_ = math.cos(yaw), math.sin(yaw)
        for a in (ARM, 0.25):
            px, py = sample_xy(*sensor_xy(10.0, 20.0, yaw, a), yaw, yoff)
            ok &= bool(np.allclose(px, 10.0 + a * c - yoff * s_, rtol=0, atol=1e-12)
                       and np.allclose(py, 20.0 + a * s_ + yoff * c, rtol=0, atol=1e-12))
        exact0 &= sensor_xy(10.0, 20.0, yaw, 0.0) == (10.0, 20.0) and sensor_xy(-3.5, 0.0, yaw, 0.0) == (-3.5, 0.0)
    check("p_i = (x + a cos psi - y_i sin psi, y + a sin psi + y_i cos psi) for four headings", ok)
    check("arm 0: the transducers' point is the navigation pose, bit for bit (the old placement)", exact0)

    base = BeliefConfig().with_values(coverage={"require_detector": False},
                                      alignment={"origin_lat": sc.LAT0, "origin_lon": sc.LON0,
                                                 "override_sigma_m": 0.0})
    as_zero = {"transducer_x_offset_source": "config", "transducer_x_offset_m": 0.0}

    # -- one target, an East and a West pass, each ping emitted when the TRANSDUCERS are abeam
    X, Y, res = 30.0025, 6.0025, 0.01
    area_p = area_from_dict(sc.area_dict(28, 5, 32, 7))

    def two_passes(ping_cfg):
        b = Belief(base.with_values(grid={"resolution_m": res}, ping=ping_cfg), area_p)
        b.ingest_ping(_one_ping(100.0, X - ARM, 0.0, 0.0, Y, 1, ARM))                # East: q = x + a
        b.ingest_ping(_one_ping(200.0, X + ARM, 12.005, math.pi, 12.005 - Y, 2, ARM))  # West: q = x - a
        b.finish()
        hit = np.flatnonzero(b.grid.n_pings)
        return b, hit
    b, hit = two_passes({})
    target = int(b.geom.cell_of(X, Y))
    check("with the arm both passes land in the target's cell (n_pings 2 there, nowhere else)",
          list(hit) == [target] and int(b.grid.n_pings[target]) == 2, f"cells {[int(h) for h in hit]}, target {target}")
    b0, hit0 = two_passes(as_zero)
    cols = sorted(int(h) % b0.geom.width for h in hit0)
    rows = {int(h) // b0.geom.width for h in hit0}
    check("placed from the navigation pose they are 2|a| apart: 69 columns of 0.01 m, one row",
          len(hit0) == 2 and cols[1] - cols[0] == round(2 * abs(ARM) / res) and rows == {target // b0.geom.width},
          f"columns {cols}")

    # -- the range of a look: a detection and a missed look, both abeam the transducers at 6 m
    Xc, Yc = 30.5, 6.0
    area_c = area_from_dict(sc.area_dict(20, -5, 40, 20))
    py_full, sy_full = sc.side_offsets(2.0, +1), sc.side_offsets(2.0, -1)
    dbp, dbs = np.full(py_full.size, -20, np.float32), np.full(sy_full.size, -20, np.float32)

    def looks(ping_cfg, odom_fallback=False):
        b = Belief(base.with_values(grid={"resolution_m": 1.0}, ping=ping_cfg), area_c)
        east = PingObs(100.0, Xc - ARM, 0.0, 0.0, 2.22, 1, 1, py_full, dbp, sy_full, dbs, ARM)
        if odom_fallback:          # no ping near the row: the pose comes from odometry
            for i in range(11):
                b.ingest_odom(OdomObs(99.5 + 0.1 * i, Xc - ARM, 0.0, 0.0))
            b.ingest_ping(PingObs(**{**east.__dict__, "t": 110.0, "x": 0.5, "y": 0.5}))
        else:
            b.ingest_ping(east)
        b.ingest_detections(DetArray(t=110.0 if odom_fallback else 100.0,
                                     detections=(Det("1", 100.0, Xc, Yc, 0.0, "block", 0.8, np.eye(2) * 1.09),)))
        if not odom_fallback:
            b.ingest_ping(PingObs(200.0, Xc + ARM, 12.0, math.pi, 2.22, 2, 2, py_full, dbp, sy_full, dbs, ARM))
        b.finish()
        (c,) = b.contacts.contacts.values()
        return c, b
    c, b = looks({})
    r = [lk.range_m for lk in c.looks]
    d = b.contacts.to_dicts(b.cfg.detector.class_names)[0]
    check("a detection and a later missed look abeam the transducers at 6 m: both look ranges are 6 m",
          len(r) == 2 and all(abs(v - 6.0) < 1e-9 for v in r) and c.looks[0].detected
          and not c.looks[1].detected and list(d["sectors_missed"]) == [9], f"ranges {r}")
    c0, _ = looks(as_zero)
    r0 = [lk.range_m for lk in c0.looks]
    check("from the navigation pose they would be sqrt(36 + a^2); the side (first sector 3) is the same",
          all(abs(v - math.hypot(6.0, ARM)) < 1e-9 for v in r0) and c.first_sector == c0.first_sector == 3,
          f"ranges {r0}")
    cf, _ = looks({}, odom_fallback=True)
    cf0, _ = looks(as_zero, odom_fallback=True)
    check("odometry fallback of a detection pose: moved by the newest ping's arm (6 m), else sqrt(36 + a^2)",
          abs(cf.looks[0].range_m - 6.0) < 1e-9 and abs(cf0.looks[0].range_m - math.hypot(6.0, ARM)) < 1e-9,
          f"{cf.looks[0].range_m:.6f}, {cf0.looks[0].range_m:.6f}")

    # -- the turn gate keeps the navigation pose
    arc = _arc()
    w, v = 1.0 / 6.0, 1.0
    along, speed = [], []
    for p0, p1 in zip(arc[10:20], arc[11:21]):
        q0, q1 = sensor_xy(p0.x, p0.y, p0.yaw, ARM), sensor_xy(p1.x, p1.y, p1.yaw, ARM)
        dt, ym = p1.t - p0.t, 0.5 * (p0.yaw + p1.yaw)
        vx, vy = (q1[0] - q0[0]) / dt, (q1[1] - q0[1]) / dt
        along.append(vx * math.cos(ym) + vy * math.sin(ym))
        speed.append(math.hypot(vx, vy))
    check("rigid body: the transducers move along the heading at v, their speed is sqrt(v^2 + w^2 a^2)",
          max(abs(u - v) for u in along) < 1e-4
          and max(abs(u - math.hypot(v, w * ARM)) for u in speed) < 1e-4,
          f"along {along[0]:.6f}, speed {speed[0]:.6f}")
    area_t = area_from_dict(sc.area_dict(-30, -30, 30, 40))
    runs = {}
    for a in (0.0, ARM):
        bt = Belief(base, area_t)
        for p in arc:
            bt.ingest_ping(PingObs(**{**p.__dict__, "transducer_x_offset": a}))
        runs[a] = bt
    st0, st1 = runs[0.0].stats, runs[ARM].stats
    check("turn gate: the same back-scanned pings and samples with and without the arm (r* unchanged)",
          st0["backscan_pings"] == st1["backscan_pings"] > 0 and st0["backscan_samples"] == st1["backscan_samples"]
          and not np.array_equal(runs[0.0].grid.n_looks, runs[ARM].grid.n_looks),
          f"{st1['backscan_pings']} pings, {st1['backscan_samples']} samples; placement differs")

    # -- the frozen-pose detector reads the published pose
    bf = Belief(BeliefConfig(), area_from_dict(sc.area_dict(-10, -40, 110, 40)))
    p0 = sc.straight_pass(0.0, 0.0, 0.7, 1.0, 1.0, arm=ARM)[0]
    for i in range(25):
        bf.ingest_ping(PingObs(**{**p0.__dict__, "t": 100.0 + i / 20.0, "x": 0.0, "y": 0.0}))
    check("frozen pose: 25 pings at (0, 0) with the arm -> 6 rejected, health ERROR",
          bf.counters["frozen_pose"] == 6 and bf.health()[0] == "ERROR")
    bn = Belief(base, area_t)
    bn.ingest_ping(PingObs(**{**p0.__dict__, "transducer_x_offset": float("nan")}))
    bc = Belief(base.with_values(ping={"transducer_x_offset_source": "config", "transducer_x_offset_m": ARM}), area_t)
    bc.ingest_ping(PingObs(**{**p0.__dict__, "transducer_x_offset": float("nan")}))
    check("a non-finite arm in a PingObs is nan_input, unless the configured arm replaces it",
          bn.counters["nan_input"] == 1 and bc.counters["nan_input"] == 0 and bc.stats["pings"] == 1)

    # -- adapter: the field is read; missing or not finite -> 0.0
    msg = {"port_stamp": {"sec": 5, "nanosec": 0}, "starboard_stamp": {"sec": 5, "nanosec": 0},
           "port_ping_number": 3, "starboard_ping_number": 3, "robot_x": 1.0, "robot_y": 2.0,
           "robot_orientation": {"x": 0, "y": 0, "z": 0, "w": 1}, "water_depth": 2.0,
           "port_y": [1.0], "port_intensity_db": [-1.0], "starboard_y": [], "starboard_intensity_db": []}
    got = [ad.ping_from_msg({**msg, "transducer_x_offset": ARM})[0].transducer_x_offset,
           ad.ping_from_msg(SimpleNamespace(**msg, transducer_x_offset=ARM))[0].transducer_x_offset,
           ad.ping_from_msg(msg)[0].transducer_x_offset]
    bad = [ad.ping_from_msg({**msg, "transducer_x_offset": v})[0].transducer_x_offset
           for v in (float("nan"), float("inf"), None, "x")]
    check("adapter: transducer_x_offset read by key and attribute; missing, NaN, inf, None -> 0.0",
          got == [ARM, ARM, 0.0] and bad == [0.0] * 4, f"{got} {bad}")

    # -- configuration: the switch and the value
    for badv in ({"ping": {"transducer_x_offset_source": "msg"}}, {"ping": {"transducer_x_offset_m": float("inf")}},
                 {"ping": {"transducer_x_offset_m": "x"}}):
        try:
            BeliefConfig.from_dict(badv)
            refused = False
        except ConfigError:
            refused = True
        check(f"refused: {badv}", refused)
    with tempfile.TemporaryDirectory() as tmp:
        f = Path(tmp) / "old_recording.yaml"
        f.write_text("ping:\n  transducer_x_offset_source: config\n  transducer_x_offset_m: -0.345\n")
        ly = load_yaml(str(f))
    check("the cli/replay --config path: a YAML file selects the configured arm",
          ly.ping.transducer_x_offset_source == "config" and ly.ping.transducer_x_offset_m == ARM)

    # -- provenance: model_params_json and the run summary
    bp = Belief(base, area_t)
    mp_before = json.loads(to_grid_dict(bp, 1)["model_params_json"])["transducer_x_offset"]
    sm = bp.summary()
    for p in sc.straight_pass(-20.0, 0.0, 0.0, 1.0, 2.0, arm=ARM):
        bp.ingest_ping(p)
    mp_after = json.loads(to_grid_dict(bp, 1)["model_params_json"])["transducer_x_offset"]
    check("model_params_json.transducer_x_offset: null before any ping, then source and min = max = arm; "
          "the same in summary()",
          mp_before == {"source": "message", "min_m": None, "max_m": None}
          and math.isnan(sm["transducer_x_offset_min_m"]) and math.isnan(sm["transducer_x_offset_max_m"])
          and mp_after == {"source": "message", "min_m": ARM, "max_m": ARM}
          and bp.summary()["transducer_x_offset_min_m"] == bp.summary()["transducer_x_offset_max_m"] == ARM,
          json.dumps(mp_after))

    # -- recorded inputs: the field from a bag, and a recording made before 2026-11-15 re-placed
    if not importlib.util.find_spec("rosbags"):
        skip("the arm through CDR and bag replay", "rosbags not installed")
        return
    from blueboat_belief.cli.replay import replay

    ts = _typestore()
    arm32 = float(np.float32(ARM))            # what a float32 message field holds
    pm = materialize(ts, "blueboat_interfaces/msg/ProcessedSSSPing", {
        "port_stamp": ad.s_to_stamp(100.0), "starboard_stamp": ad.s_to_stamp(100.0),
        "port_ping_number": 1, "starboard_ping_number": 1, "robot_x": 1.0, "robot_y": 2.0,
        "robot_orientation": {"w": 1.0}, "water_depth": 2.2, "transducer_x_offset": ARM,
        "port_intensity_db": dbp, "port_y": py_full, "starboard_intensity_db": dbs, "starboard_y": sy_full})
    pb = ts.deserialize_cdr(ts.serialize_cdr(pm, "blueboat_interfaces/msg/ProcessedSSSPing"),
                            "blueboat_interfaces/msg/ProcessedSSSPing")
    check("ProcessedSSSPing CDR round trip: the float32 arm reaches PingObs",
          ad.ping_from_msg(pb)[0].transducer_x_offset == arm32, repr(ad.ping_from_msg(pb)[0].transducer_x_offset))
    area18 = area_from_dict(sc.area_dict(-10, -40, 110, 60))
    pings0 = sc.lawn(0, 0, 2, 18.0, 60.0)
    pingsA = sc.lawn(0, 0, 2, 18.0, 60.0, arm=arm32)
    arrays = sc.arrays_for(pings0, [(25.0, 9.0), (40.0, 27.0)])
    fixes = [f.__class__(t=f.t + 0.002, lat=f.lat, lon=f.lon, status=f.status) for f in sc.fixes_for(pings0)]

    def evs(pings):
        return sc.events(pings, sc.odom_for(pings), fixes, arrays)

    def grid_bytes(b):
        d = to_grid_dict(b, 1)
        mp = json.loads(d.pop("model_params_json"))
        return encode_npz(d), mp
    direct = Belief(BeliefConfig(), area18)
    sc.feed(direct, [(a_, o, k, OdomObs(a_, x.x, x.y, x.yaw) if k == "odom" else x) for a_, o, k, x in evs(pingsA)])
    direct.finish()
    ref, ref_mp = grid_bytes(direct)
    cfg_old = BeliefConfig().with_values(ping={"transducer_x_offset_source": "config", "transducer_x_offset_m": arm32})
    with tempfile.TemporaryDirectory() as tmp:
        bagA, bag0 = Path(tmp) / "with_arm", Path(tmp) / "before_2026_11_15"
        _write_bag(ts, bagA, evs(pingsA))
        _write_bag(ts, bag0, evs(pings0))
        rA = Belief(BeliefConfig(), area18)
        replay(str(bagA), rA, [str(ROOT / "test" / "msg_reference")])
        rO = Belief(cfg_old, area18)
        replay(str(bag0), rO, [str(ROOT / "test" / "msg_reference")])
        r0 = Belief(BeliefConfig(), area18)
        replay(str(bag0), r0, [str(ROOT / "test" / "msg_reference")])
    gA, mpA = grid_bytes(rA)
    gO, mpO = grid_bytes(rO)
    g0, _ = grid_bytes(r0)
    check("a bag whose pings carry the arm replays = direct feeding (grid and contacts bit for bit)",
          gA == ref and mpA == ref_mp and mpA["transducer_x_offset"] == {"source": "message", "min_m": arm32, "max_m": arm32})
    check("a bag recorded with 0.0, replayed with the configured arm, equals it bit for bit (source: config)",
          gO == gA and mpO["transducer_x_offset"] == {"source": "config", "min_m": arm32, "max_m": arm32})
    check("the same 0.0 bag with the default (the message) gives the old placement, which differs",
          g0 != gA and len(direct.contacts.contacts) == len(rO.contacts.contacts) == 2)


def main() -> int:
    t0 = time.perf_counter()
    for sec in (section_import_sweep, section_config_geo, section_area, section_alignment, section_pose,
                section_geometry, section_grid, section_contacts, section_render, section_contract,
                section_layouts, section_adapters, section_determinism, section_localisation,
                section_rejections, section_performance, section_snapshot, section_packaging,
                section_bag_replay, section_turns, section_tools, section_records, section_consumer,
                section_arm):
        sec()
    print(f"\nALL CHECKS PASSED ({N_CHECKS} checks, {N_SKIPS} skipped, {time.perf_counter() - t0:.1f} s)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
