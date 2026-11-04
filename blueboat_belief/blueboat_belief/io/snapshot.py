"""Snapshot files for D7 evaluation (brief §4.2, §12.5): ``<stem>.npz`` + ``<stem>.json``.

The ``.npz`` is byte-deterministic (fixed zip timestamps, sorted members, uncompressed). That is
what the T12 determinism criterion is checked on. The manifest carries the configuration, the
model parameters, counters and the input recording. Wall-clock values appear only under
``provenance``, which the determinism check excludes.

Files are claimed with O_EXCL and never overwritten (CM-7).
"""

from __future__ import annotations

import hashlib
import io
import json
import zipfile
from pathlib import Path
from typing import Any, Dict, Optional, Tuple

import numpy as np

from .dataroot import claim_file

SCHEMA = "blueboat_belief/snapshot/1"
_ZIP_TIME = (1980, 1, 1, 0, 0, 0)


def _jsonable(v: Any) -> Any:
    if isinstance(v, np.ndarray):
        return [_jsonable(x) for x in v.tolist()]
    if isinstance(v, (np.floating,)):
        return float(v)
    if isinstance(v, (np.integer,)):
        return int(v)
    if isinstance(v, (np.bool_,)):
        return bool(v)
    if isinstance(v, float) and v != v:
        return None
    if isinstance(v, dict):
        return {k: _jsonable(x) for k, x in v.items()}
    if isinstance(v, (list, tuple)):
        return [_jsonable(x) for x in v]
    return v


def encode_npz(grid: Dict[str, Any]) -> bytes:
    """Every array and scalar of the grid dict, plus the contacts as canonical JSON bytes."""
    members: Dict[str, np.ndarray] = {}
    for k, v in grid.items():
        if k == "contacts":
            js = json.dumps(_jsonable(v), sort_keys=True, separators=(",", ":"), allow_nan=False)
            members["contacts_json"] = np.frombuffer(js.encode("utf-8"), dtype=np.uint8)
        elif k == "header":
            members["header_stamp"] = np.asarray(v["stamp"], dtype=np.float64)
        elif isinstance(v, (list, tuple)):
            members[k] = np.asarray(v, dtype=np.str_)
        elif isinstance(v, str):
            members[k] = np.asarray(v, dtype=np.str_)
        else:
            members[k] = np.asarray(v)
    buf = io.BytesIO()
    with zipfile.ZipFile(buf, "w", compression=zipfile.ZIP_STORED) as zf:
        for name in sorted(members):
            arr_buf = io.BytesIO()
            np.save(arr_buf, members[name], allow_pickle=False)
            info = zipfile.ZipInfo(f"{name}.npy", date_time=_ZIP_TIME)
            info.external_attr = 0o644 << 16
            zf.writestr(info, arr_buf.getvalue())
    return buf.getvalue()


def decode_npz(data: bytes) -> Dict[str, Any]:
    out: Dict[str, Any] = {}
    with np.load(io.BytesIO(data), allow_pickle=False) as z:
        for k in z.files:
            out[k] = z[k]
    if "contacts_json" in out:
        out["contacts"] = json.loads(out.pop("contacts_json").tobytes().decode("utf-8"))
    return out


def manifest_for(grid: Dict[str, Any], npz_bytes: bytes, config_flat: Dict[str, Any],
                 counters: Dict[str, int], stats: Dict[str, int], input_recording: str = "",
                 provenance: Optional[Dict[str, Any]] = None,
                 summary: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
    m = {
        "schema": SCHEMA,
        "contract_version": grid["contract_version"],
        "grid_seq": int(grid["grid_seq"]),
        "stamp": float(grid["header"]["stamp"]),
        "model_params": json.loads(grid["model_params_json"]),
        "config": _jsonable(config_flat),
        "counters": dict(counters),
        "stats": dict(stats),
        "input_recording": input_recording,
        "npz_sha256": hashlib.sha256(npz_bytes).hexdigest(),
    }
    if summary:
        m["summary"] = _jsonable(summary)
    if provenance:
        m["provenance"] = _jsonable(provenance)
    return m


def write_snapshot(directory: Path, stem: str, grid: Dict[str, Any], config_flat: Dict[str, Any],
                   counters: Dict[str, int], stats: Dict[str, int], input_recording: str = "",
                   provenance: Optional[Dict[str, Any]] = None,
                   summary: Optional[Dict[str, Any]] = None) -> Tuple[Path, Path]:
    """Claim ``stem[-k].npz`` exclusively, write it and its manifest; return both paths."""
    npz_path = claim_file(Path(directory), stem, ".npz")
    data = encode_npz(grid)
    npz_path.write_bytes(data)
    man_path = npz_path.with_suffix(".json")
    man = manifest_for(grid, data, config_flat, counters, stats, input_recording, provenance, summary)
    fd_path = claim_file(man_path.parent, man_path.stem, ".json")
    fd_path.write_text(json.dumps(man, sort_keys=True, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    return npz_path, fd_path


def read_snapshot(npz_path: Path) -> Dict[str, Any]:
    return decode_npz(Path(npz_path).read_bytes())
