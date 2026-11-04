"""Contract ``belief_grid/0`` (C-3/C-4/C-7): the one dict every output is built from.

``to_grid_dict`` returns every C-3/C-4 field under its contract name, numpy arrays in the
exact message dtype, scalars as Python/numpy scalars, contacts as a list of dicts. The ROS
message (``ros/adapters.grid_to_msg``), the snapshot (``io/snapshot``) and the query service all
read this dict, so they cannot diverge. ``GRID_FIELDS`` / ``CONTACT_FIELDS`` are the reference
tables the smoke test checks against the ``.msg`` files.
"""

from __future__ import annotations

import json
import math
from typing import Dict, Optional

import numpy as np

from .. import __version__
from .belief import Belief

CONTRACT_VERSION = "belief_grid/0"
MODEL_PARAMS_SCHEMA = 1

# contract name -> ROS field type (exactly as in blueboat_belief_interfaces/msg/BeliefGrid.msg)
GRID_FIELDS: Dict[str, str] = {
    "header": "std_msgs/Header", "contract_version": "string", "grid_seq": "uint32",
    "width": "uint32", "height": "uint32", "resolution": "float32",
    "origin_x": "float64", "origin_y": "float64", "datum_lat": "float64", "datum_lon": "float64",
    "odom_to_map_x": "float64", "odom_to_map_y": "float64", "odom_to_map_yaw": "float64",
    "odom_to_map_sigma_m": "float32", "odom_to_map_valid": "bool",
    "aspect_sector_count": "uint8", "aspect_sector_origin_deg": "float32",
    "class_names": "string[]", "prior_density_per_ha": "float32", "model_params_json": "string",
    "p_background": "float32[]", "p_target": "float32[]", "coverage": "float32[]",
    "n_looks": "uint16[]", "n_pings": "uint32[]", "n_aspects": "uint8[]",
    "depth_m": "float32[]", "last_intensity_db": "float32[]", "max_confidence": "float32[]",
    "aspect_counts": "uint8[]", "aspect_log_miss": "float32[]",
    "area_x": "float64[]", "area_y": "float64[]",
    "contacts": "blueboat_belief_interfaces/BeliefContact[]",
}
CONTACT_FIELDS: Dict[str, str] = {
    "uid": "uint32", "x": "float64", "y": "float64", "covariance": "float64[4]",
    "p_exists": "float32", "state": "uint8", "class_probabilities": "float32[]",
    "sectors_detected": "uint8[]", "sectors_missed": "uint8[]", "n_looks_detected": "uint16",
    "first_confidence": "float32", "first_sector": "uint8", "max_confidence": "float32",
    "shadow_known": "bool", "has_shadow": "bool",
    "created": "builtin_interfaces/Time", "updated": "builtin_interfaces/Time",
    "look_bearings_deg": "float32[]", "look_ranges_m": "float32[]", "look_detected": "bool[]",
    "detection_ids": "string[]",
}
# fields added to belief_grid/0 by this producer (CONTRACT_DEVIATIONS.md D-C4, D-C5)
ADDED_FIELDS = {"area_x", "area_y", "look_detected"}


def model_params(b: Belief) -> dict:
    c = b.cfg
    return {
        "schema": MODEL_PARAMS_SCHEMA,
        "belief_version": __version__,
        "pd_model": {"name": "trapezoid_placeholder", "plateau": c.pd.plateau, "r_lo_m": c.pd.r_lo_m,
                     "r_hi_m": c.pd.r_hi_m, "ramp_m": c.pd.ramp_m, "lever_arm_m": c.pd.lever_arm_m},
        "pfa": c.pd.pfa,
        "rho_model": {"name": "paull2013_linear", "independence_angle_deg": c.rho.independence_angle_deg,
                      "fold_opposite": c.rho.fold_opposite, "ordering": c.rho.ordering},
        "base_rate": c.detector.base_rate,
        "detector_calibrated": c.detector.calibrated,
        "look_timeout_s": c.look.timeout_s,
        "prior_p0": b.grid.p0,
        "negative_evidence": "gated" if c.coverage.require_detector else "ungated",
        "turn_gating": {"enabled": c.turn.enabled, "window_s": c.turn.window_s,
                        "min_speed_mps": c.turn.min_speed_mps,
                        "min_yaw_rate_rad_s": c.turn.min_yaw_rate_rad_s},
        "alignment_source": b.aligner.source,
        "association": {"gate_m": c.association.gate_m, "chi2": c.association.chi2},
        "area": {"name": b.area.name, "format": b.area.source_format},
    }


def to_grid_dict(b: Belief, grid_seq: int = 0, stamp: Optional[float] = None) -> dict:
    """Every C-3 field (C-4 inside ``contacts``) under its contract name (contract C-7)."""
    g, al = b.geom, b.aligner
    t = b.last_event_t if stamp is None else float(stamp)
    poly = b.area.polygon_xy
    return {
        "header": {"stamp": float(t), "frame_id": "map"},
        "contract_version": CONTRACT_VERSION,
        "grid_seq": np.uint32(grid_seq),
        "width": np.uint32(g.width),
        "height": np.uint32(g.height),
        "resolution": np.float32(g.resolution),
        "origin_x": float(g.origin_x),
        "origin_y": float(g.origin_y),
        "datum_lat": float(b.area.lat0),
        "datum_lon": float(b.area.lon0),
        "odom_to_map_x": float(al.tx),
        "odom_to_map_y": float(al.ty),
        "odom_to_map_yaw": 0.0,
        "odom_to_map_sigma_m": np.float32(al.sigma if math.isfinite(al.sigma) else np.nan),
        "odom_to_map_valid": bool(al.valid),
        "aspect_sector_count": np.uint8(b.grid.S),
        "aspect_sector_origin_deg": np.float32(b.cfg.aspect.origin_deg),
        "class_names": list(b.cfg.detector.class_names),
        "prior_density_per_ha": np.float32(b.cfg.prior.density_per_ha),
        "model_params_json": json.dumps(model_params(b), sort_keys=True, separators=(",", ":")),
        "p_background": b.grid.p_background(),
        "p_target": b.render_p_target(),
        "coverage": b.grid.coverage(),
        "n_looks": b.grid.n_looks.copy(),
        "n_pings": b.grid.n_pings.copy(),
        "n_aspects": b.grid.n_aspects(),
        "depth_m": b.grid.masked_f32(b.grid.depth),
        "last_intensity_db": b.grid.masked_f32(b.grid.intensity),
        "max_confidence": b.grid.masked_f32(b.grid.max_conf),
        "aspect_counts": b.grid.aspect_counts.copy(),
        "aspect_log_miss": b.grid.aspect_log_miss(),
        "area_x": poly[:, 0].astype(np.float64),
        "area_y": poly[:, 1].astype(np.float64),
        "contacts": b.contacts.to_dicts(b.cfg.detector.class_names),
    }
