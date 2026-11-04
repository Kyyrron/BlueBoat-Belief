"""ROS-free adapters: duck-typed messages -> core dataclasses, and the grid dict -> message fields.

Every reader takes any object exposing the ROS field names by attribute **or** by key: an rclpy
message, a ``rosbags`` dataclass, or a plain dict. Every writer returns *field dicts* (nested
plain dicts, numpy arrays of the exact dtype), which a backend materialises:
- ``ros/belief_node.py`` for rclpy;
- ``test/smoke_test.py`` and ``cli/replay.py`` for rosbags.

The message is therefore always built from ``contract.to_grid_dict`` through one code path (C-7).
The contract field-name table is ``core.contract.GRID_FIELDS`` / ``CONTACT_FIELDS``: a rename
costs one line there.
"""

from __future__ import annotations

import math
from typing import Any, Dict, List, Optional, Tuple

import numpy as np

from ..core.belief import Det, DetArray, FixObs, OdomObs, PingObs
from ..core.contract import CONTACT_FIELDS, GRID_FIELDS
from ..core.geometry import yaw_from_quaternion


def get(obj: Any, name: str, default: Any = None) -> Any:
    if isinstance(obj, dict):
        return obj.get(name, default)
    return getattr(obj, name, default)


def stamp_to_s(stamp: Any) -> float:
    if stamp is None:
        return 0.0
    return float(get(stamp, "sec", 0)) + 1e-9 * float(get(stamp, "nanosec", 0))


def s_to_stamp(t: float) -> Dict[str, int]:
    if not math.isfinite(t) or t < 0:
        t = 0.0
    sec = int(math.floor(t))
    nsec = int(round((t - sec) * 1e9))
    if nsec >= 1_000_000_000:
        sec, nsec = sec + 1, nsec - 1_000_000_000
    return {"sec": sec, "nanosec": nsec}


def _f32(v: Any) -> np.ndarray:
    return np.asarray(v if v is not None else [], dtype=np.float32)


def _seq(v: Any) -> list:
    """A sequence field as a list; never truth-tests it (numpy arrays from rclpy / rosbags)."""
    return [] if v is None else list(v)


# ---------------------------------------------------------------------------- inputs
def ping_from_msg(msg: Any, arrival_t: float = 0.0) -> Tuple[PingObs, bool]:
    """``blueboat_interfaces/ProcessedSSSPing`` -> (PingObs, zero_stamp_used_arrival)."""
    pp = int(get(msg, "port_ping_number", 0))
    sp = int(get(msg, "starboard_ping_number", 0))
    t = stamp_to_s(get(msg, "port_stamp")) if pp != 0 else stamp_to_s(get(msg, "starboard_stamp"))
    used_arrival = t == 0.0
    if used_arrival:
        t = float(arrival_t)
    q = get(msg, "robot_orientation")
    yaw = yaw_from_quaternion(float(get(q, "x", 0.0)), float(get(q, "y", 0.0)),
                              float(get(q, "z", 0.0)), float(get(q, "w", 1.0)))
    return PingObs(t=t, x=float(get(msg, "robot_x")), y=float(get(msg, "robot_y")), yaw=yaw,
                   water_depth=float(get(msg, "water_depth")),
                   port_ping_number=pp, starboard_ping_number=sp,
                   port_y=_f32(get(msg, "port_y")), port_db=_f32(get(msg, "port_intensity_db")),
                   starboard_y=_f32(get(msg, "starboard_y")),
                   starboard_db=_f32(get(msg, "starboard_intensity_db"))), used_arrival


def odom_from_msg(msg: Any, arrival_t: float = 0.0) -> Tuple[OdomObs, bool]:
    """``nav_msgs/Odometry`` -> (OdomObs, zero_stamp_used_arrival). Zero stamps happen on the
    real launch (mavros on use_sim_time without /clock; LOCAL_CHECKLIST L-2)."""
    t = stamp_to_s(get(get(msg, "header"), "stamp"))
    used = t == 0.0
    if used:
        t = float(arrival_t)
    pose = get(get(msg, "pose"), "pose")
    pos, q = get(pose, "position"), get(pose, "orientation")
    yaw = yaw_from_quaternion(float(get(q, "x", 0.0)), float(get(q, "y", 0.0)),
                              float(get(q, "z", 0.0)), float(get(q, "w", 1.0)))
    return OdomObs(t=t, x=float(get(pos, "x")), y=float(get(pos, "y")), yaw=yaw), used


def fix_from_msg(msg: Any, arrival_t: float = 0.0) -> Tuple[FixObs, bool]:
    """``sensor_msgs/NavSatFix`` -> (FixObs, zero_stamp_used_arrival)."""
    t = stamp_to_s(get(get(msg, "header"), "stamp"))
    used = t == 0.0
    if used:
        t = float(arrival_t)
    return FixObs(t=t, lat=float(get(msg, "latitude")), lon=float(get(msg, "longitude")),
                  status=int(get(get(msg, "status"), "status", 0))), used


def detections_from_msg(msg: Any) -> Tuple[DetArray, int]:
    """``vision_msgs/Detection2DArray`` as Perception fills it (Perception ``ros/convert.py``):
    covariance block [0, 1, 6, 7]; class name in ``class_id``. Returns (array, n_without_results)."""
    t_img = stamp_to_s(get(get(msg, "header"), "stamp"))
    dets: List[Det] = []
    skipped = 0
    for d in _seq(get(msg, "detections")):
        results = _seq(get(d, "results"))
        if not results:
            skipped += 1
            continue
        r0 = results[0]
        hyp = get(r0, "hypothesis")
        cov = _seq(get(get(r0, "pose"), "covariance"))
        if len(cov) < 8:
            cov = [0.0] * 36
        R = np.array([[cov[0], cov[1]], [cov[6], cov[7]]], dtype=np.float64)
        bbox = get(d, "bbox")
        center = get(bbox, "center")
        pos = get(center, "position")
        t_row = stamp_to_s(get(get(d, "header"), "stamp")) or t_img
        dets.append(Det(id=str(get(d, "id", "")), t_row=t_row,
                        x=float(get(pos, "x")), y=float(get(pos, "y")),
                        theta=float(get(center, "theta", 0.0)),
                        class_name=str(get(hyp, "class_id", "")), score=float(get(hyp, "score", 0.0)),
                        cov=R, size_x=float(get(bbox, "size_x", float("nan"))),
                        size_y=float(get(bbox, "size_y", float("nan")))))
    return DetArray(t=t_img, detections=tuple(dets)), skipped


# ---------------------------------------------------------------------------- outputs
def _contact_fields(c: Dict[str, Any]) -> Dict[str, Any]:
    out = {}
    for name in CONTACT_FIELDS:
        v = c[name]
        if name in ("created", "updated"):
            v = s_to_stamp(float(v))
        out[name] = v
    return out


def grid_to_fields(d: Dict[str, Any]) -> Dict[str, Any]:
    """The BeliefGrid field dict, built only from ``to_grid_dict`` output."""
    out: Dict[str, Any] = {}
    for name in GRID_FIELDS:
        v = d[name]
        if name == "header":
            v = {"stamp": s_to_stamp(float(v["stamp"])), "frame_id": str(v["frame_id"])}
        elif name == "contacts":
            v = [_contact_fields(c) for c in v]
        out[name] = v
    return out


def query_to_fields(q: Dict[str, Any], grid_seq: int) -> Dict[str, Any]:
    if not q.get("ok"):
        return {"ok": False, "error": q.get("error", ""), "grid_seq": np.uint32(grid_seq)}
    return {
        "ok": True, "error": "", "grid_seq": np.uint32(grid_seq),
        "p_any": np.float32(q["p_any"]), "expected_count": np.float32(q["expected_count"]),
        "coverage_mean": np.float32(q["coverage_mean"]), "coverage_min": np.float32(q["coverage_min"]),
        "observed_cells": np.uint32(q["observed_cells"]), "total_cells": np.uint32(q["total_cells"]),
        "min_aspects": np.uint8(q["min_aspects"]), "max_aspects": np.uint8(q["max_aspects"]),
        "class_names": list(q["class_names"]),
        "class_distribution": np.asarray(q["class_distribution"], dtype=np.float32),
        "contact_uids": np.asarray(q["contact_uids"], dtype=np.uint32),
    }


def occupancy_fields(d: Dict[str, Any], channel: str, p_floor: float = 1e-5) -> Dict[str, Any]:
    """Display-only ``nav_msgs/OccupancyGrid`` (int8 data, v1 bug 4).

    - ``coverage``: round(100 * coverage).
    - ``p_target``: log scale, 0 at ``p_floor`` and 100 at 1.
    - -1 outside the area, or for a cell with no look."""
    v = np.asarray(d[channel], dtype=np.float64)
    if channel == "p_target":
        lo = math.log10(p_floor)
        x = (np.log10(np.clip(v, p_floor, 1.0)) - lo) / (0.0 - lo)
    else:
        x = v
    data = np.round(100.0 * np.clip(np.nan_to_num(x, nan=0.0), 0.0, 1.0)).astype(np.int8)
    data[~np.isfinite(v) | (np.asarray(d["n_looks"]) == 0)] = -1
    return {
        "header": {"stamp": s_to_stamp(float(d["header"]["stamp"])), "frame_id": "map"},
        "info": {"map_load_time": s_to_stamp(float(d["header"]["stamp"])),
                 "resolution": np.float32(d["resolution"]),
                 "width": np.uint32(d["width"]), "height": np.uint32(d["height"]),
                 "origin": {"position": {"x": float(d["origin_x"]), "y": float(d["origin_y"]), "z": 0.0},
                            "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}}},
        "data": data,
    }


def contact_ellipse(c: Dict[str, Any], k_sigma: float = 2.0) -> Tuple[float, float, float]:
    """(axis_a, axis_b, yaw) of the k-sigma ellipse (full axis lengths, metres)."""
    P = np.asarray(c["covariance"], dtype=np.float64).reshape(2, 2)
    w, V = np.linalg.eigh(P)
    w = np.maximum(w, 0.0)
    yaw = math.atan2(V[1, 1], V[0, 1])
    return 2 * k_sigma * math.sqrt(w[1]), 2 * k_sigma * math.sqrt(w[0]), yaw


MARKER_CYLINDER = 3
MARKER_TEXT = 9
MARKER_ADD = 0
MARKER_DELETEALL = 3


def markers_fields(d: Dict[str, Any], k_sigma: float = 2.0) -> Dict[str, Any]:
    """``visualization_msgs/MarkerArray``: DELETEALL, then per contact a flat 2-sigma ellipse
    (cylinder) and a text label "<uid> p=<p_exists> C|c" (C = confirmed)."""
    hdr = {"stamp": s_to_stamp(float(d["header"]["stamp"])), "frame_id": "map"}
    markers: List[Dict[str, Any]] = [{"header": hdr, "ns": "belief_contacts", "id": 0,
                                      "action": MARKER_DELETEALL}]
    for c in d["contacts"]:
        a, b, yaw = contact_ellipse(c, k_sigma)
        qz, qw = math.sin(yaw / 2.0), math.cos(yaw / 2.0)
        confirmed = int(c["state"]) == 1
        col = {"r": 0.1, "g": 0.8, "b": 0.2, "a": 0.6} if confirmed else {"r": 1.0, "g": 0.6, "b": 0.0, "a": 0.6}
        pose = {"position": {"x": float(c["x"]), "y": float(c["y"]), "z": 0.0},
                "orientation": {"x": 0.0, "y": 0.0, "z": qz, "w": qw}}
        uid = int(c["uid"])
        markers.append({"header": hdr, "ns": "belief_contacts", "id": 2 * uid, "type": MARKER_CYLINDER,
                        "action": MARKER_ADD, "pose": pose,
                        "scale": {"x": max(a, 0.05), "y": max(b, 0.05), "z": 0.05}, "color": col})
        markers.append({"header": hdr, "ns": "belief_contacts", "id": 2 * uid + 1, "type": MARKER_TEXT,
                        "action": MARKER_ADD, "pose": pose, "scale": {"x": 0.0, "y": 0.0, "z": 0.6},
                        "color": {"r": 1.0, "g": 1.0, "b": 1.0, "a": 1.0},
                        "text": f"{uid} p={float(c['p_exists']):.2f} {'C' if confirmed else 'c'}"})
    return {"markers": markers}


def diagnostics_fields(level: str, message: str, counters: Dict[str, int],
                       stats: Dict[str, int], extra: Optional[Dict[str, str]] = None) -> Dict[str, Any]:
    """One ``diagnostic_msgs/DiagnosticStatus`` (level 0 OK, 1 WARN, 2 ERROR); every counter."""
    lv = {"OK": 0, "WARN": 1, "ERROR": 2}[level]
    values = [{"key": f"rejected.{k}", "value": str(v)} for k, v in counters.items()]
    values += [{"key": f"stats.{k}", "value": str(v)} for k, v in stats.items()]
    for k, v in (extra or {}).items():
        values.append({"key": k, "value": str(v)})
    return {"level": lv, "name": "belief_map", "message": message,
            "hardware_id": "blueboat_belief", "values": values}
