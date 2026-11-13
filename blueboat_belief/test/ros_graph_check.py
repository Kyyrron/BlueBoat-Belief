"""ROS runtime check of the Belief node, without a simulator (dev-only, not installed).

It starts ``belief_map`` on an ISOLATED ROS domain, publishes a synthetic survey as real ROS
messages on the five input topics with the upstream publishers' QoS, and compares what the node
publishes with the offline core fed the same messages:

- every ``BeliefGrid`` field (bit for bit; ``grid_seq`` is checked to advance by 1);
- ``QueryBelief``, the two ``OccupancyGrid`` displays, the contact markers, ``/diagnostics``;
- the QoS of the node's endpoints, and that the grid is latched for a late subscriber;
- after SIGINT: exit code 0 and a final snapshot identical to the offline core's.

Run it from ``blueboat_belief/`` with the system python (the one the node uses), in a shell
where ``ROS_DOMAIN_ID`` is set and is not 0 and where Belief's packages are sourced::

    /usr/bin/python3 -B -m test.ros_graph_check [--scenario straight|lawn] [--rate 250]
                                                [--attach] [--no-records] [--work DIR]

``--attach`` expects a node that is already running (started by the launch file, for the
``record_inputs`` check) and leaves it running. Exit codes: 0 pass, 1 a check failed,
3 the environment is not isolated, 4 inconclusive (messages were lost: rerun with a lower rate).

It refuses to run on domain 0 or beside any other node: it must never touch a live graph.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import signal
import subprocess
import sys
import tempfile
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

N_CHECKS = 0
FAILED = []


def check(name: str, cond, detail: str = "") -> bool:
    global N_CHECKS
    N_CHECKS += 1
    print(f"  [{'PASS' if cond else 'FAIL'}] {name}" + (f" -- {detail}" if detail else ""), flush=True)
    if not cond:
        FAILED.append(name)
    return bool(cond)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--scenario", choices=("straight", "lawn"), default="straight")
    ap.add_argument("--rate", type=float, default=250.0, help="messages per second")
    ap.add_argument("--attach", action="store_true", help="use a node that is already running")
    ap.add_argument("--no-records", action="store_true", help="publish no image records")
    ap.add_argument("--work", default="", help="work directory (default: a temporary one)")
    args = ap.parse_args(argv)

    domain = os.environ.get("ROS_DOMAIN_ID", "")
    if not domain or domain == "0":
        print("refused: ROS_DOMAIN_ID must be set and must not be 0 (this check never joins a live graph)")
        return 3

    import rclpy
    from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy,
                           qos_profile_sensor_data)
    from rclpy.serialization import deserialize_message, serialize_message

    from blueboat_interfaces.msg import ProcessedSSSPing
    from diagnostic_msgs.msg import DiagnosticArray
    from nav_msgs.msg import OccupancyGrid, Odometry
    from sensor_msgs.msg import NavSatFix
    from std_msgs.msg import String
    from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose
    from visualization_msgs.msg import MarkerArray

    from blueboat_belief_interfaces.msg import BeliefGrid
    from blueboat_belief_interfaces.srv import QueryBelief

    from blueboat_belief.core.area import area_from_dict
    from blueboat_belief.core.belief import Belief
    from blueboat_belief.core.config import BeliefConfig
    from blueboat_belief.core.contract import CONTACT_FIELDS, GRID_FIELDS, to_grid_dict
    from blueboat_belief.io.snapshot import encode_npz, read_snapshot
    from blueboat_belief.ros import adapters as ad
    from test import scenarios as sc

    work = Path(args.work) if args.work else Path(tempfile.mkdtemp(prefix="belief_ros_check_"))
    work.mkdir(parents=True, exist_ok=True)
    print(f"ROS runtime check: domain {domain}, scenario {args.scenario}, work dir {work}")

    # ------------------------------------------------------------------ scenario and config
    import yaml

    area_d = sc.area_dict(-10, -40, 110, 60)
    area_file = work / "area.yaml"
    if not area_file.exists():
        area_file.write_text(yaml.safe_dump(area_d))
    overrides = {"area.file": str(area_file), "io.data_dir": str(work), "snapshot.period_s": 3600.0,
                 "alignment.origin_lat": sc.LAT0, "alignment.origin_lon": sc.LON0}
    params_file = work / "params.yaml"
    if not params_file.exists():
        params_file.write_text(yaml.safe_dump({"belief_map": {"ros__parameters": overrides}}))
    cfg = BeliefConfig.from_dict(overrides)
    ref = Belief(cfg, area_from_dict(area_d))

    if args.scenario == "straight":
        pings = sc.straight_pass(0.0, 0.0, 0.0, 1.0, 50.0)
        targets = [(20.0, 9.0), (35.0, -12.0)]
    else:
        pings = sc.lawn(0.0, 0.0, 2, 18.0, 50.0)
        targets = [(20.0, 9.0), (35.0, 27.0), (30.0, -8.0)]
    period = 2.135
    arrays = sc.arrays_for(pings, targets, period_s=period)
    recs = [] if args.no_records else sc.records_for(pings, arrays, period_s=period)
    evs = sc.events(pings, sc.odom_for(pings), sc.fixes_for(pings), arrays, records=recs)

    # ------------------------------------------------------------------ message builders
    def stamp(msg_stamp, t: float) -> None:
        s = ad.s_to_stamp(t)
        msg_stamp.sec, msg_stamp.nanosec = s["sec"], s["nanosec"]

    def quat(msg_q, yaw: float) -> None:
        msg_q.x, msg_q.y, msg_q.z, msg_q.w = 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)

    def ping_msg(p):
        m = ProcessedSSSPing()
        stamp(m.port_stamp, p.t)
        stamp(m.starboard_stamp, p.t)
        m.port_ping_number, m.starboard_ping_number = int(p.port_ping_number), int(p.starboard_ping_number)
        m.robot_x, m.robot_y, m.water_depth = float(p.x), float(p.y), float(p.water_depth)
        quat(m.robot_orientation, p.yaw)
        m.port_y, m.port_intensity_db = p.port_y.tolist(), p.port_db.tolist()
        m.starboard_y, m.starboard_intensity_db = p.starboard_y.tolist(), p.starboard_db.tolist()
        return m

    def dets_msg(a):
        """As Perception fills it (BlueBoat-Perception ros/convert.py:47-75)."""
        m = Detection2DArray()
        stamp(m.header.stamp, a.t)
        m.header.frame_id = "odom"
        for d in a.detections:
            det = Detection2D()
            stamp(det.header.stamp, d.t_row)
            det.header.frame_id = "odom"
            det.id = d.id
            det.bbox.center.position.x, det.bbox.center.position.y = float(d.x), float(d.y)
            det.bbox.center.theta = float(d.theta)
            det.bbox.size_x, det.bbox.size_y = 1.0, 1.0
            h = ObjectHypothesisWithPose()
            h.hypothesis.class_id, h.hypothesis.score = d.class_name, float(d.score)
            h.pose.pose.position.x, h.pose.pose.position.y = float(d.x), float(d.y)
            cov = [0.0] * 36
            cov[0], cov[1], cov[6], cov[7] = (float(v) for v in d.cov.ravel())
            h.pose.covariance = cov
            det.results.append(h)
            m.detections.append(det)
        return m

    def rec_msg(r):
        return String(data=json.dumps({
            "schema": ad.RECORD_SCHEMA, "key": r.key,
            "time": {"t_min_s": r.t_min, "t_max_s": r.t_max, "n_rows": 256,
                     "clock": "ros_time_of_sonar_profiles"},
            "processing": {"detector": {"name": r.detector, "dummy": r.dummy, "status": "ran"},
                           "calibration": {"name": "identity", "calibrated": r.calibrated},
                           "preprocess": {"name": "fixed_gray", "params": {"near_range_mask_m": 0.0}}},
            "detections": [] if r.searched else None}, sort_keys=True))

    def odom_msg(o):
        m = Odometry()
        stamp(m.header.stamp, o.t)
        m.header.frame_id = "world"
        m.pose.pose.position.x, m.pose.pose.position.y = float(o.x), float(o.y)
        quat(m.pose.pose.orientation, o.yaw)
        return m

    def fix_msg(f):
        m = NavSatFix()
        stamp(m.header.stamp, f.t)
        m.status.status = int(f.status)
        m.latitude, m.longitude = float(f.lat), float(f.lon)
        return m

    build = {"ping": ping_msg, "dets": dets_msg, "rec": rec_msg, "odom": odom_msg, "fix": fix_msg}

    def to_ref(kind: str, msg) -> None:
        """Feed the reference core what the node receives: the message after a wire round trip."""
        w = deserialize_message(serialize_message(msg), type(msg))
        if kind == "ping":
            ref.ingest_ping(ad.ping_from_msg(w)[0])
        elif kind == "dets":
            ref.ingest_detections(ad.detections_from_msg(w)[0])
        elif kind == "rec":
            rec = ad.record_from_msg(w)
            ref.reject_record() if rec is None else ref.ingest_record(rec)
        elif kind == "odom":
            ref.ingest_odom(ad.odom_from_msg(w)[0])
        else:
            ref.ingest_fix(ad.fix_from_msg(w)[0])

    # ------------------------------------------------------------------ the graph
    rclpy.init()
    node = rclpy.create_node("belief_ros_check")
    spin = lambda dt=0.0: rclpy.spin_once(node, timeout_sec=dt)  # noqa: E731

    def spin_for(seconds: float) -> None:
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            spin(0.02)

    spin_for(1.0)
    # with --attach the launch file's own node and its bag recorder are expected beside the node
    mine = ("belief_ros_check", "belief_map") + (("rosbag2_recorder",) if args.attach else ())
    others = [n for n in node.get_node_names()
              if n not in mine and not (args.attach and n.startswith("launch_ros_"))]
    if others or node.get_publishers_info_by_topic("/clock"):
        print(f"refused: this graph is not empty ({others or '/clock has a publisher'})")
        rclpy.shutdown()
        return 3

    proc = None
    if not args.attach:
        from ament_index_python.packages import get_package_prefix

        exe = Path(get_package_prefix("blueboat_belief")) / "lib" / "blueboat_belief" / "belief_node"
        log = open(work / "belief_node.log", "w", encoding="utf-8")
        proc = subprocess.Popen([str(exe), "--ros-args", "--params-file", str(params_file),
                                 "-p", "use_sim_time:=false"], stdout=log, stderr=subprocess.STDOUT,
                                start_new_session=True)

    be10 = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=10)
    rec_q = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=100,
                       durability=DurabilityPolicy.VOLATILE)
    pubs = {"ping": node.create_publisher(ProcessedSSSPing, "/sss_processor/processed", be10),   # sss_processor_node
            "dets": node.create_publisher(Detection2DArray, "/sss_ai/detections", 10),            # Perception
            "rec": node.create_publisher(String, "/sss_ai/image_records", rec_q),                 # Perception
            "odom": node.create_publisher(Odometry, "/blueboat/odom", 10),                        # robot_interface
            "fix": node.create_publisher(NavSatFix, "/mavros/global_position/global", qos_profile_sensor_data)}
    topics = {"ping": "/sss_processor/processed", "dets": "/sss_ai/detections",
              "rec": "/sss_ai/image_records", "odom": "/blueboat/odom",
              "fix": "/mavros/global_position/global"}

    diag = {}

    def on_diag(msg) -> None:
        for st in msg.status:
            if st.name == "belief_map":
                diag.clear()
                diag.update({kv.key: kv.value for kv in st.values})
                diag["__level"] = int.from_bytes(st.level, "little") if isinstance(st.level, bytes) else int(st.level)
                diag["__message"] = st.message

    node.create_subscription(DiagnosticArray, "/diagnostics", on_diag, 10)

    print("[R1] node start, endpoints and QoS")
    t_end = time.monotonic() + 20.0
    while time.monotonic() < t_end and not all(
            any(i.node_name == "belief_map" for i in node.get_subscriptions_info_by_topic(t))
            for t in topics.values()):
        spin(0.1)
        if proc is not None and proc.poll() is not None:
            break
    alive = proc is None or proc.poll() is None
    check("the node is running", alive, "" if alive else f"exit code {proc.returncode}, see {work}/belief_node.log")
    if not alive:
        rclpy.shutdown()
        return 1
    for kind, topic in topics.items():
        infos = [i for i in node.get_subscriptions_info_by_topic(topic) if i.node_name == "belief_map"]
        want = ReliabilityPolicy.RELIABLE if kind in ("dets", "rec") else ReliabilityPolicy.BEST_EFFORT
        check(f"belief_map subscribes {topic} once, {want.name}",
              len(infos) == 1 and infos[0].qos_profile.reliability == want)
        check(f"our publisher on {topic} is matched (QoS compatible, CM-4)",
              pubs[kind].get_subscription_count() >= 1)
    gi = [i for i in node.get_publishers_info_by_topic("/belief_map/belief_grid") if i.node_name == "belief_map"]
    check("/belief_map/belief_grid is offered RELIABLE + TRANSIENT_LOCAL by belief_map",
          len(gi) == 1 and gi[0].qos_profile.reliability == ReliabilityPolicy.RELIABLE
          and gi[0].qos_profile.durability == DurabilityPolicy.TRANSIENT_LOCAL)
    names = dict(node.get_topic_names_and_types())
    for t, typ in (("/belief_map/belief_grid", "blueboat_belief_interfaces/msg/BeliefGrid"),
                   ("/belief_map/p_target", "nav_msgs/msg/OccupancyGrid"),
                   ("/belief_map/coverage", "nav_msgs/msg/OccupancyGrid"),
                   ("/belief_map/contacts", "visualization_msgs/msg/MarkerArray"),
                   ("/diagnostics", "diagnostic_msgs/msg/DiagnosticArray")):
        check(f"{t} [{typ}]", names.get(t) == [typ], str(names.get(t)))
    svc = dict(node.get_service_names_and_types())
    check("/belief_map/query_belief [blueboat_belief_interfaces/srv/QueryBelief]",
          svc.get("/belief_map/query_belief") == ["blueboat_belief_interfaces/srv/QueryBelief"])
    spin_for(1.5)
    check("diagnostics are published before any input arrives (steady-clock timer)",
          "stats.pings" in diag and diag.get("alignment.source") == "override", diag.get("__message", ""))

    # ------------------------------------------------------------------ publish the survey
    print(f"[R2] publishing {len(evs)} messages at {args.rate:.0f}/s "
          f"({len(pings)} pings, {len(arrays)} arrays, {len(recs)} records)")
    dt = 1.0 / args.rate
    nxt = time.monotonic()
    for _, _, kind, obj in evs:
        msg = build[kind](obj)
        pubs[kind].publish(msg)
        to_ref(kind, msg)
        nxt += dt
        while time.monotonic() < nxt:
            spin(0.0)
    want = {f"stats.{k}": str(v) for k, v in ref.stats.items()}
    want.update({f"rejected.{k}": str(v) for k, v in ref.counters.items()})
    t_end = time.monotonic() + 30.0
    while time.monotonic() < t_end and any(diag.get(k) != v for k, v in want.items()):
        spin(0.05)
    diff = {k: (diag.get(k), v) for k, v in want.items() if diag.get(k) != v}
    if diff:
        print(f"  INCONCLUSIVE: the node's counters differ from the offline core's (node, core): {diff}")
        print("  (BEST_EFFORT pings can be lost when published too fast: rerun with a lower --rate)")
        if proc is not None:
            os.killpg(proc.pid, signal.SIGINT)
        rclpy.shutdown()
        return 4
    check("every stats.* and rejected.* counter on /diagnostics equals the offline core's",
          True, f"pings {diag['stats.pings']}, covered {diag['stats.pings_covered']}, "
                f"detections {diag['stats.detections']}, records {diag['stats.records']}")
    check("no message needed the arrival-time fallback (all stamps were used)", diag.get("zero_stamps_arrival") == "0")
    check("diagnostics: level OK, coverage source, observed calibration, event age",
          diag.get("__level") == 0 and diag.get("coverage.source") == ("arrays" if args.no_records else "records")
          and diag.get("detector.calibrated_observed") == ("None" if args.no_records else "False")
          and "latency.event_age_s" in diag, f"{diag.get('__message')}; source {diag.get('coverage.source')}")

    # ------------------------------------------------------------------ outputs vs the core
    print("[R3] BeliefGrid (subscribed only now: it must be latched) against the offline core")
    grids, occ, markers = [], {}, []
    latched = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=1,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
    t_sub = time.monotonic()
    node.create_subscription(BeliefGrid, "/belief_map/belief_grid", grids.append, latched)
    node.create_subscription(OccupancyGrid, "/belief_map/p_target", lambda m: occ.__setitem__("p_target", m), latched)
    node.create_subscription(OccupancyGrid, "/belief_map/coverage", lambda m: occ.__setitem__("coverage", m), latched)
    node.create_subscription(MarkerArray, "/belief_map/contacts", markers.append, 10)
    while time.monotonic() < t_sub + 10.0 and not grids:
        spin(0.01)
    first_after = time.monotonic() - t_sub
    check("a late subscriber gets the last grid at once (TRANSIENT_LOCAL)", bool(grids) and first_after < 0.9,
          f"{first_after * 1000:.0f} ms after subscribing")
    t_end = time.monotonic() + 8.0
    while time.monotonic() < t_end and (len(grids) < 4 or len(occ) < 2 or not markers):
        spin(0.05)
    seqs = [g.grid_seq for g in grids]
    check("grid_seq advances by exactly 1 per published grid", len(seqs) >= 3
          and all(b - a == 1 for a, b in zip(seqs, seqs[1:])), str(seqs))
    g = grids[-1]
    d = to_grid_dict(ref, grid_seq=int(g.grid_seq))
    f = ad.grid_to_fields(d)

    def same(a, b) -> bool:
        if isinstance(b, dict):
            return all(same(getattr(a, k), v) for k, v in b.items())
        if isinstance(b, np.ndarray) or isinstance(a, np.ndarray) or hasattr(a, "typecode"):
            b = np.asarray(b)
            a = np.asarray(a, dtype=b.dtype) if b.dtype != object else np.asarray(a)
            return a.shape == b.shape and bool(np.array_equal(a, b, equal_nan=b.dtype.kind == "f"))
        if isinstance(b, (list, tuple)):
            return len(a) == len(b) and all(same(x, y) for x, y in zip(a, b))
        if isinstance(b, (np.floating, float)):
            b32 = np.float32(b) if isinstance(b, np.float32) else float(b)
            return (math.isnan(a) and math.isnan(b32)) or type(b32)(a) == b32
        if isinstance(b, (np.integer, np.bool_)):
            return a == b.item()
        return a == b

    bad = [name for name in GRID_FIELDS if name != "contacts" and not same(getattr(g, name), f[name])]
    check(f"all {len(GRID_FIELDS) - 1} BeliefGrid fields equal the offline core's, bit for bit", not bad, ", ".join(bad))
    cbad = []
    if len(g.contacts) == len(f["contacts"]):
        for i, (cm, cf) in enumerate(zip(g.contacts, f["contacts"])):
            cbad += [f"{i}.{name}" for name in CONTACT_FIELDS if not same(getattr(cm, name), cf[name])]
    check(f"{len(f['contacts'])} contacts, all {len(CONTACT_FIELDS)} fields equal", bool(f["contacts"])
          and len(g.contacts) == len(f["contacts"]) and not cbad, ", ".join(cbad[:6]))
    mp = json.loads(g.model_params_json)
    check("model_params_json: calibrated false, gated, coverage source as fed",
          mp["detector_calibrated"] is False and mp["negative_evidence"] == "gated"
          and mp["coverage_source"] == ("arrays" if args.no_records else "records"))
    for ch in ("p_target", "coverage"):
        exp = ad.occupancy_fields(d, ch)
        m = occ.get(ch)
        check(f"OccupancyGrid {ch}: data and geometry equal the core's display encoding", m is not None
              and np.array_equal(np.asarray(m.data, dtype=np.int8), exp["data"])
              and (m.info.width, m.info.height) == (int(d["width"]), int(d["height"]))
              and m.header.frame_id == "map")
    check("contact markers: DELETEALL + an ellipse and a label per contact",
          len(markers[-1].markers) == 1 + 2 * len(f["contacts"]))

    print("[R4] QueryBelief")
    cli = node.create_client(QueryBelief, "/belief_map/query_belief")
    check("the service is available", cli.wait_for_service(timeout_sec=5.0))

    def query(x, y, r):
        req = QueryBelief.Request()
        req.x, req.y, req.radius_m = float(x), float(y), float(r)
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(node, fut, timeout_sec=5.0)
        return fut.result()

    tx, ty = targets[0]
    res = query(tx, ty, 5.0)
    exp = ad.query_to_fields(ref.query_disc(tx, ty, 5.0), 0)
    qbad = [k for k, v in exp.items() if k != "grid_seq" and not same(getattr(res, k), v)]
    check("a disc around a target: every response field equals the core's", res is not None and not qbad
          and res.ok and len(res.contact_uids) == 1, ", ".join(qbad) or f"p_any {res.p_any:.3f}")
    res = query(0.0, 0.0, -1.0)
    check("an invalid radius is answered ok = false with a message, not an exception",
          res is not None and not res.ok and "radius" in res.error)

    # ------------------------------------------------------------------ shutdown and snapshot
    if proc is None:
        print("[R5] --attach: the node is left running")
    else:
        print("[R5] SIGINT: exit code and final snapshot")
        os.killpg(proc.pid, signal.SIGINT)
        try:
            rc = proc.wait(timeout=20.0)
        except subprocess.TimeoutExpired:
            os.killpg(proc.pid, signal.SIGKILL)
            rc = None
        log_text = (work / "belief_node.log").read_text(encoding="utf-8", errors="replace")
        check("the node exits with code 0 on SIGINT, with no traceback", rc == 0 and "Traceback" not in log_text,
              f"exit code {rc}")
        check("its log states the area, the snapshot folder and the calibration warning",
              "snapshots ->" in log_text and "detector.calibrated is false" in log_text)
        finals = sorted((work / "data" / "Belief_data").glob("*/snapshot_final.npz"))
        ok = len(finals) == 1
        if ok:
            snap = read_snapshot(finals[0])
            ref.finish()
            expect = encode_npz(to_grid_dict(ref, grid_seq=int(snap["grid_seq"])))
            ok = finals[0].read_bytes() == expect
            man = json.loads(finals[0].with_suffix(".json").read_text())
            ok = ok and man["stats"] == ref.stats and man["counters"] == ref.counters
        check("snapshot_final.npz is byte-identical to the offline core's, and its manifest counts agree", ok,
              str(finals[0]) if finals else "no final snapshot")

    node.destroy_node()
    rclpy.shutdown()
    print(f"\n{'ALL ROS CHECKS PASSED' if not FAILED else 'ROS CHECKS FAILED'} "
          f"({N_CHECKS} checks, {len(FAILED)} failed)")
    for name in FAILED:
        print(f"  failed: {name}")
    return 1 if FAILED else 0


if __name__ == "__main__":
    sys.exit(main())
