"""blueboat-belief-replay: feed a recorded rosbag2 through the core offline (deterministic).

The T12 determinism route (docs/PHASE2_DESIGN.md §1).
1. Messages of the four input topics are read with ``rosbags`` (pure Python, no ROS) in
   **log-time order**.
2. Zero header stamps take the log (receive) time.
3. The final snapshot is written to ``--out``, or to ``<root>/data/Belief_data/<bag>-replay``.
4. The same bag and config give a byte-identical ``.npz``.

Message definitions come from the bag itself (mcap / rosbag2 store them). ``--msg-dir`` registers
extra ``.msg`` files laid out ``<dir>/<pkg>/msg/<Name>.msg`` for bags that lack them, e.g.
``test/msg_reference``.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Iterable, List, Optional

from ..core.belief import Belief
from ..core.config import BeliefConfig, load_yaml
from ..core.contract import to_grid_dict
from ..io.dataroot import belief_dir, claim_dir
from ..io.missions import load_area_file
from ..io.snapshot import write_snapshot
from ..ros import adapters as ad

TOPICS = {
    "/sss_processor/processed": "ping",
    "/sss_ai/detections": "dets",
    "/blueboat/odom": "odom",
    "/mavros/global_position/global": "fix",
}


def dispatch(belief: Belief, kind: str, msg, log_time_s: float) -> None:
    if kind == "ping":
        belief.ingest_ping(ad.ping_from_msg(msg, log_time_s)[0])
    elif kind == "odom":
        belief.ingest_odom(ad.odom_from_msg(msg, log_time_s)[0])
    elif kind == "fix":
        belief.ingest_fix(ad.fix_from_msg(msg, log_time_s)[0])
    elif kind == "dets":
        belief.ingest_detections(ad.detections_from_msg(msg)[0])


def _extra_types(msg_dirs: Iterable[str]) -> dict:
    from rosbags.typesys import get_types_from_msg  # noqa: PLC0415 - optional dependency

    types: dict = {}
    for d in msg_dirs:
        for f in sorted(Path(d).glob("*/msg/*.msg")):
            pkg = f.parent.parent.name
            types.update(get_types_from_msg(f.read_text(encoding="utf-8"), f"{pkg}/msg/{f.stem}"))
    return types


def replay(bag: str, belief: Belief, msg_dirs: Iterable[str] = ()) -> int:
    from rosbags.highlevel import AnyReader  # noqa: PLC0415
    from rosbags.typesys import Stores, get_typestore  # noqa: PLC0415

    ts = get_typestore(Stores.ROS2_JAZZY)
    extra = _extra_types(msg_dirs)
    if extra:
        ts.register(extra)
    n = 0
    with AnyReader([Path(bag)], default_typestore=ts) as reader:
        conns = [c for c in reader.connections if c.topic in TOPICS]
        for conn, timestamp, raw in reader.messages(connections=conns):
            msg = reader.deserialize(raw, conn.msgtype)
            dispatch(belief, TOPICS[conn.topic], msg, timestamp * 1e-9)
            n += 1
    belief.finish()
    return n


def main(argv: Optional[List[str]] = None) -> int:
    ap = argparse.ArgumentParser(prog="blueboat-belief-replay", description=__doc__.splitlines()[0])
    ap.add_argument("bag", help="rosbag2 folder or .mcap")
    ap.add_argument("--area", required=True, help="world folder / metadata.yaml / survey-area file")
    ap.add_argument("--config", default="", help="belief YAML (default: built-in defaults)")
    ap.add_argument("--out", default="", help="output folder (claimed exclusively)")
    ap.add_argument("--data-dir", default="", help="data root (else BLUEBOAT_DATA_DIR / workspace)")
    ap.add_argument("--msg-dir", action="append", default=[], help="extra .msg definitions")
    ap.add_argument("--no-require-detector", action="store_true",
                    help="credit misses without detector coverage (development only; labelled)")
    args = ap.parse_args(argv)
    cfg = load_yaml(args.config) if args.config else BeliefConfig()
    if args.no_require_detector:
        cfg = cfg.with_values(coverage={"require_detector": False})
    belief = Belief(cfg, load_area_file(args.area))
    n = replay(args.bag, belief, args.msg_dir)
    out = Path(args.out) if args.out else belief_dir(args.data_dir)
    run = claim_dir(out, f"{Path(args.bag).stem}-replay")
    d = to_grid_dict(belief, grid_seq=1)
    npz, man = write_snapshot(run, "snapshot_final", d, cfg.to_flat(), belief.counters,
                              belief.stats, input_recording=str(Path(args.bag).resolve()),
                              summary=belief.summary())
    print(json.dumps({"messages": n, "snapshot": str(npz), "manifest": str(man),
                      "summary": belief.summary(), "counters": belief.counters,
                      "stats": belief.stats}, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    sys.exit(main())
