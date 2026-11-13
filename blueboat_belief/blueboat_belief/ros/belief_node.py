"""belief_map: the BlueBoat-Belief ROS 2 node. The ONLY file that imports rclpy or message types.

Inputs (brief §3; owner decision D-2), QoS compatible with every publisher (checked on the live graph, VALIDATION_REPORT §5.2):
- ``/sss_processor/processed`` (``ProcessedSSSPing``): BEST_EFFORT, depth 50;
- ``/sss_ai/detections`` (``Detection2DArray``): RELIABLE, depth 50;
- ``/blueboat/odom`` (``Odometry``): BEST_EFFORT, depth 50;
- ``/mavros/global_position/global`` (``NavSatFix``): BEST_EFFORT, depth 10;
- ``/sss_ai/image_records`` (``String`` JSON): RELIABLE, depth 100. Optional companion (owner
  decision 2026-11-12): exact detector coverage, docs/MATH.md §8.2. Belief works without it.

Outputs (contract belief_grid/0; docs/topics.md):
- ``~/belief_grid``: RELIABLE, TRANSIENT_LOCAL depth 1, ``publish.grid_rate_hz``;
- ``~/query_belief``;
- ``~/p_target`` and ``~/coverage`` (``OccupancyGrid``, display);
- ``~/contacts`` (``MarkerArray``);
- ``/diagnostics``;
- snapshots under ``<root>/data/Belief_data/<run>/``.

A zero header stamp (the real launch, TODO real-boat check L-2) is replaced by the node clock at
arrival: run with ``use_sim_time:=false`` on the real boat and ``true`` under Gazebo.
"""

from __future__ import annotations

import array
import datetime as _dt
import signal
import sys
from typing import Any, Dict

import numpy as np
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.exceptions import InvalidParameterTypeException
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rcl_interfaces.msg import ParameterDescriptor, ParameterType

from blueboat_interfaces.msg import ProcessedSSSPing
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String
from vision_msgs.msg import Detection2DArray
from visualization_msgs.msg import Marker, MarkerArray

from blueboat_belief_interfaces.msg import BeliefContact, BeliefGrid
from blueboat_belief_interfaces.srv import QueryBelief

from ..core.area import AreaError
from ..core.belief import Belief
from ..core.config import BeliefConfig, ConfigError
from ..core.contract import to_grid_dict
from ..io.dataroot import belief_dir, claim_dir
from ..io.missions import load_area_file
from ..io.snapshot import write_snapshot
from . import adapters as ad

def fill(msg: Any, fields: Dict[str, Any], seq_types: Dict[str, type]) -> Any:
    """Materialise a field dict onto an rclpy message (exact dtypes; v1 bug 4)."""
    for name, value in fields.items():
        cur = getattr(msg, name)
        if isinstance(value, dict):
            fill(cur, value, seq_types)
        elif isinstance(value, list) and value and isinstance(value[0], dict):
            cls = seq_types[name]
            setattr(msg, name, [fill(cls(), v, seq_types) for v in value])
        elif isinstance(cur, np.ndarray):
            setattr(msg, name, np.asarray(value, dtype=cur.dtype))
        elif isinstance(cur, array.array):
            arr = np.ascontiguousarray(np.asarray(value))
            code = cur.typecode
            want = {"f": np.float32, "d": np.float64, "B": np.uint8, "b": np.int8, "H": np.uint16,
                    "h": np.int16, "I": np.uint32, "i": np.int32, "L": np.uint64, "l": np.int64,
                    "Q": np.uint64, "q": np.int64}[code]
            out = array.array(code)
            out.frombytes(arr.astype(want).tobytes())
            setattr(msg, name, out)
        elif isinstance(cur, bytes):
            setattr(msg, name, bytes([int(value)]))
        elif isinstance(cur, list):
            setattr(msg, name, [bool(v) if isinstance(v, (bool, np.bool_)) else v for v in
                                (value.tolist() if isinstance(value, np.ndarray) else value)])
        elif isinstance(cur, bool):
            setattr(msg, name, bool(value))
        elif isinstance(cur, int):
            setattr(msg, name, int(value))
        elif isinstance(cur, float):
            setattr(msg, name, float(value))
        else:
            setattr(msg, name, value)
    return msg


class BeliefNode(Node):
    def __init__(self) -> None:
        super().__init__("belief_map")
        self._fqn = self.get_fully_qualified_name()        # used after the context is shut down
        defaults = BeliefConfig().to_flat()
        try:
            for k, v in defaults.items():
                if isinstance(v, list):
                    self.declare_parameter(k, v, ParameterDescriptor(type=ParameterType.PARAMETER_STRING_ARRAY))
                elif isinstance(v, float):
                    # dynamic typing: "wait_s: 60" in a YAML file is an integer on the wire, and
                    # a statically typed double parameter would raise on it. core/config.py
                    # checks the type and converts.
                    self.declare_parameter(k, v, ParameterDescriptor(dynamic_typing=True))
                else:
                    self.declare_parameter(k, v)
            flat = {k: self.get_parameter(k).value for k in defaults}
            for k, v in list(flat.items()):
                if isinstance(defaults[k], float) and isinstance(v, int) and not isinstance(v, bool):
                    flat[k] = float(v)
                elif isinstance(defaults[k], list) and v is not None and list(v) == [""]:
                    flat[k] = []                           # ROS cannot carry an empty string list
            bad = ad.unknown_parameters(getattr(self, "_parameter_overrides", {}) or {}, defaults)
            if bad:
                raise ConfigError(f"unknown parameters: {bad}")
            self.cfg = BeliefConfig.from_flat(flat)
            area = load_area_file(self.cfg.area.file)
        except (ConfigError, AreaError, InvalidParameterTypeException) as exc:
            self.get_logger().fatal(f"belief_map cannot start: {exc}")
            raise SystemExit(2) from exc
        self.core = Belief(self.cfg, area)
        self.seq = 0                       # the sequence number of the last PUBLISHED grid
        self._last = None
        self.snap_n = 0
        stamp = _dt.datetime.now().strftime("%Y_%m_%d-%H_%M_%S")
        self.run_dir = claim_dir(belief_dir(self.cfg.io.data_dir), stamp)
        self.zero_stamps = 0
        self.get_logger().info(
            f"area '{area.name}' ({area.source_format}) datum {area.lat0:.7f},{area.lon0:.7f}; grid "
            f"{self.core.geom.width}x{self.core.geom.height} @ {self.core.geom.resolution} m; "
            f"snapshots -> {self.run_dir}")
        if not self.cfg.detector.calibrated:
            self.get_logger().warn("detector.calibrated is false: scores are raw, p_exists is not a "
                                   "probability (calibration is a priority, docs/MATH.md)")

        be = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=50)
        rel = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=50)
        fixq = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=10)
        latched = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST,
                             depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        recq = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST,
                          depth=100, durability=DurabilityPolicy.VOLATILE)
        self.create_subscription(ProcessedSSSPing, "/sss_processor/processed", self._on_ping, be)
        # The detections subscription is created before the records one: when an image's array
        # and its record are both ready in one executor cycle, the array is handled first.
        self.create_subscription(Detection2DArray, "/sss_ai/detections", self._on_dets, rel)
        self.create_subscription(String, "/sss_ai/image_records", self._on_record, recq)
        self.create_subscription(Odometry, "/blueboat/odom", self._on_odom, be)
        self.create_subscription(NavSatFix, "/mavros/global_position/global", self._on_fix, fixq)
        self._warned: set = set()
        self.pub_grid = self.create_publisher(BeliefGrid, "~/belief_grid", latched)
        self.pub_pt = self.create_publisher(OccupancyGrid, "~/p_target", latched)
        self.pub_cov = self.create_publisher(OccupancyGrid, "~/coverage", latched)
        self.pub_mk = self.create_publisher(MarkerArray, "~/contacts", 10)
        self.pub_diag = self.create_publisher(DiagnosticArray, "/diagnostics", 10)
        self.create_service(QueryBelief, "~/query_belief", self._on_query)
        # Outputs are paced by a steady clock, not by the node clock. With use_sim_time and no
        # /clock (simulator not started, or paused while a ping renders) node-clock timers never
        # fire, and the node would publish nothing, not even its diagnostics. Snapshots stay on
        # the node clock: one per snapshot.period_s of mission time.
        self._steady = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(1.0 / self.cfg.publish.grid_rate_hz, self._publish_grid, clock=self._steady)
        self.create_timer(1.0 / self.cfg.publish.display_rate_hz, self._publish_display, clock=self._steady)
        self.create_timer(1.0, self._publish_diag, clock=self._steady)
        self.create_timer(self.cfg.snapshot.period_s, self._snapshot)

    # ----------------------------------------------------------------- inputs
    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_ping(self, msg) -> None:
        p, used = ad.ping_from_msg(msg, self._now())
        self.zero_stamps += int(used)
        self.core.ingest_ping(p)

    def _on_dets(self, msg) -> None:
        a, _ = ad.detections_from_msg(msg)
        self.core.ingest_detections(a)

    def _on_record(self, msg) -> None:
        rec = ad.record_from_msg(msg)
        if rec is None:
            self.core.reject_record()
            return
        self.core.ingest_record(rec)

    def _warn_once(self, key: str, text: str) -> None:
        if key not in self._warned:
            self._warned.add(key)
            self.get_logger().warn(text)

    def _check_observed(self) -> None:
        """Things the operator must see once: they change what the published numbers mean."""
        c = self.core
        if self.cfg.detector.calibrated and c.observed_calibrated is False:
            self._warn_once("calibrated", "detector.calibrated is true, but Perception's image "
                            "records say calibrated: false. p_exists is not a probability.")
        if c.observed_near_range_mask_m > 0.0:
            self._warn_once("mask", f"Perception blanks slant ranges below "
                            f"{c.observed_near_range_mask_m:.2f} m (near_range_mask_m). Belief does "
                            "not model it: coverage there may be over-credited (docs/MATH.md §8.2).")
        if c.stats["detections_class_other"] > 0:
            self._warn_once("other", "a detection's class is not in detector.class_names and was "
                            "kept as 'other'. Set detector.class_names to the detector's classes "
                            "(stats.detections_class_other counts them).")
        if c.stats["records_overlap"] > 0:
            self._warn_once("overlap", "seabed images overlap more than their neighbours: the "
                            "release bound fell back to t_min (docs/MATH.md §8.2 item 2).")

    def _on_odom(self, msg) -> None:
        o, used = ad.odom_from_msg(msg, self._now())
        self.zero_stamps += int(used)
        self.core.ingest_odom(o)

    def _on_fix(self, msg) -> None:
        f, used = ad.fix_from_msg(msg, self._now())
        self.zero_stamps += int(used)
        self.core.ingest_fix(f)

    # ----------------------------------------------------------------- outputs
    def _publish_grid(self) -> None:
        """The only place grid_seq advances: +1 per published BeliefGrid (BeliefGrid.msg)."""
        self.seq += 1
        d = to_grid_dict(self.core, grid_seq=self.seq)
        self.pub_grid.publish(fill(BeliefGrid(), ad.grid_to_fields(d), {"contacts": BeliefContact}))
        self._last = d

    def _publish_display(self) -> None:
        d = self._last or to_grid_dict(self.core, grid_seq=self.seq)
        self.pub_pt.publish(fill(OccupancyGrid(), ad.occupancy_fields(d, "p_target"), {}))
        self.pub_cov.publish(fill(OccupancyGrid(), ad.occupancy_fields(d, "coverage"), {}))
        self.pub_mk.publish(fill(MarkerArray(), ad.markers_fields(d), {"markers": Marker}))

    def _publish_diag(self) -> None:
        self._check_observed()
        level, message = self.core.health()
        al = self.core.aligner
        # How far the belief lags the clock: the node clock minus the newest event applied
        # (the BeliefGrid header stamp). While pings flow this is the release latency of MATH §8.
        last = self.core.last_event_t
        age = self._now() - last if last > 0.0 else float("nan")
        extra = {"alignment.valid": al.valid, "alignment.source": al.source,
                 "alignment.pairs": al.n_pairs, "alignment.sigma_m": al.sigma,
                 "detector.calibrated": self.cfg.detector.calibrated,
                 "detector.calibrated_observed": self.core.observed_calibrated,
                 "detector.observed": self.core.observed_detector,
                 "coverage.source": ("none" if not self.cfg.coverage.require_detector else
                                     "records" if self.core.records_mode else "arrays"),
                 "latency.event_age_s": f"{age:.3f}",
                 "zero_stamps_arrival": self.zero_stamps}
        extra.update({f"summary.{k}": v for k, v in self.core.summary().items()})
        st = fill(DiagnosticStatus(), ad.diagnostics_fields(level, message, self.core.counters,
                                                           self.core.stats, extra), {"values": KeyValue})
        arr = DiagnosticArray()
        arr.header.stamp = self.get_clock().now().to_msg()
        arr.status = [st]
        self.pub_diag.publish(arr)

    def _on_query(self, req, resp):
        q = self.core.query_disc(float(req.x), float(req.y), float(req.radius_m))
        return fill(resp, ad.query_to_fields(q, self.seq), {})

    def _snapshot(self, final: bool = False) -> None:
        d = to_grid_dict(self.core, grid_seq=self.seq)      # the last published sequence number
        self.snap_n += 1
        stem = "snapshot_final" if final else f"snapshot_{self.snap_n:04d}"
        write_snapshot(self.run_dir, stem, d, self.cfg.to_flat(), self.core.counters, self.core.stats,
                       input_recording="live", summary=self.core.summary(),
                       provenance={"wall_utc": _dt.datetime.now(_dt.timezone.utc).isoformat(),
                                   "node": self._fqn})

    def shutdown(self) -> None:
        self.core.finish()
        self._snapshot(final=True)


def main(argv=None) -> None:
    """Exit code 0 after Ctrl-C or an external shutdown, 2 when the configuration or the
    survey-area file is refused (the launch file relies on it)."""
    rclpy.init(args=argv)
    node = None
    code = 0
    try:
        node = BeliefNode()
        rclpy.spin(node)
    except SystemExit as exc:                     # fatal start-up error: keep its exit code
        code = exc.code if isinstance(exc.code, int) else 1
    except (KeyboardInterrupt, ExternalShutdownException):
        pass                                      # SIGINT / SIGTERM: a normal stop
    finally:
        # A second SIGINT is normal: Ctrl-C reaches the whole process group and ``ros2 launch``
        # forwards it again. It must not interrupt the final snapshot.
        for sig in (signal.SIGINT, signal.SIGTERM):
            try:
                signal.signal(sig, signal.SIG_IGN)
            except (ValueError, OSError):
                pass
        if node is not None:
            try:
                node.shutdown()
            except Exception as exc:  # noqa: BLE001 - best effort final snapshot at teardown
                print(f"belief_map: final snapshot failed: {exc}", file=sys.stderr)
            try:
                node.destroy_node()
            except Exception:  # noqa: BLE001 - the context may already be gone
                pass
        if rclpy.ok():
            rclpy.shutdown()
    if code:
        sys.exit(code)


if __name__ == "__main__":
    main()
