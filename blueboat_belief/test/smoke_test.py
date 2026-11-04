"""BlueBoat-Belief offline gate: ``python3 -m test.smoke_test`` from ``blueboat_belief/``, no ROS.

SSS-Sim style: ``check()`` prints ``[PASS]``/``[FAIL]`` and stops at the first failure (exit 1);
``[N]`` section headers; a visible ``SKIP`` when an optional dependency is missing; the run ends
with ``ALL CHECKS PASSED`` and the check count. Sections follow docs/PHASE2_DESIGN.md §7 and
brief §11 (closed forms, determinism, localisation, rejections, message layouts, performance).
"""

from __future__ import annotations

import ast
import importlib
import importlib.abc
import importlib.util
import io
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
    from blueboat_belief.core.belief import REASONS, Belief, Det, DetArray, FixObs, OdomObs, PingObs
    from blueboat_belief.core.config import BeliefConfig
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
        for _ in range(200):
            k = rng.integers(0, 3)
            if k == 0:
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

    print("[15] performance (indicative only: cloud machine, not the target laptop)")
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
    check("launch imports offline; record command covers the four inputs",
          mod.record_command("/tmp/x")[-4:] == list(mod.INPUT_TOPICS) and len(mod.INPUT_TOPICS) == 4)
    node = (ROOT / "blueboat_belief" / "ros" / "belief_node.py").read_text()
    for needle in ('"/sss_processor/processed"', '"/sss_ai/detections"', '"/blueboat/odom"',
                   '"/mavros/global_position/global"', '"~/belief_grid"', '"~/query_belief"',
                   "DurabilityPolicy.TRANSIENT_LOCAL", "ReliabilityPolicy.BEST_EFFORT"):
        check(f"node declares {needle}", needle in node)
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
                     "water_depth": o.water_depth, "port_intensity_db": o.port_db, "port_y": o.port_y,
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
    b, _ = _run_scenario()
    cov = b.grid.coverage().astype(np.float64)[b.inside]
    q = -np.log1p(-np.clip(cov, 0, 1 - 1e-12))
    pk = 1.0 - float(np.mean(np.exp(-q)))
    check("area_cumulative_pd = 1 - sum pi_i Q_i (uniform pi) = mean in-area coverage",
          abs(b.area_cumulative_pd() - pk) < 1e-9 and 0 < pk < 1, f"P = {pk:.4f}")
    s = b.summary()
    check("summary: cumulative PD, contacts, confirmed, retired, alignment",
          set(s) == {"area_cumulative_pd", "contacts", "contacts_confirmed", "contacts_retired", "alignment_valid"})


def main() -> int:
    t0 = time.perf_counter()
    for sec in (section_import_sweep, section_config_geo, section_area, section_alignment, section_pose,
                section_geometry, section_grid, section_contacts, section_render, section_contract,
                section_layouts, section_adapters, section_determinism, section_localisation,
                section_rejections, section_performance, section_snapshot, section_packaging,
                section_bag_replay, section_turns, section_tools):
        sec()
    print(f"\nALL CHECKS PASSED ({N_CHECKS} checks, {N_SKIPS} skipped, {time.perf_counter() - t0:.1f} s)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
