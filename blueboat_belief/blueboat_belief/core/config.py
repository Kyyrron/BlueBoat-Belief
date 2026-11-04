"""Validated configuration (docs/PHASE2_DESIGN.md §6).

Every section is a frozen dataclass. Unknown keys raise by name (the SSS-Sim ``_strict``
pattern). ROS parameters are flat ``section.key`` names (``to_flat`` / ``from_flat``).
Optional floats use NaN for "unset" (brief §7, v1 bug 5). Every numeric default is either a
cited value or a labelled placeholder (docs/MATH.md §0).
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field, fields, replace
from typing import Any, Dict, Mapping, Tuple

NAN = float("nan")


class ConfigError(ValueError):
    """A configuration value or key is invalid."""


@dataclass(frozen=True)
class GridCfg:
    resolution_m: float = 0.5            # brief §10 (engineering)


@dataclass(frozen=True)
class AspectCfg:
    sector_count: int = 12               # brief §10 (Fawcett 2010 increments; a discretisation choice)
    origin_deg: float = 0.0              # lower edge of sector 0, deg CCW from East


@dataclass(frozen=True)
class LookCfg:
    timeout_s: float = 20.0              # brief §10 (engineering)
    time_reset_s: float = 5.0            # backward jump that starts a new epoch (processor's 5 s rule)


@dataclass(frozen=True)
class PriorCfg:
    density_per_ha: float = 5.0          # site-dependent placeholder (brief §10)
    exact_poisson: bool = False          # p0 = 1 - exp(-lambda) instead of lambda


@dataclass(frozen=True)
class DetectorCfg:
    base_rate: float = 0.5               # pi; must equal the calibration split's base rate (brief §10)
    calibrated: bool = False             # PRIORITY: Perception calibration is identity today
    class_names: Tuple[str, ...] = ("block", "pipe", "cage", "tyre", "plate", "other")
    ignore_classes: Tuple[str, ...] = ("dummy_center",)


@dataclass(frozen=True)
class ClampCfg:
    p_min: float = 1e-5                  # brief §10
    p_max: float = 0.999


@dataclass(frozen=True)
class PdCfg:
    plateau: float = 0.7                 # PLACEHOLDER (brief §10)
    r_lo_m: float = 3.0
    r_hi_m: float = 25.0
    ramp_m: float = 3.0
    pfa: float = 0.0                     # neglected: Bourgault miss update = PFA = 0 special case
    lever_arm_m: float = 0.285           # processor's lateral transducer offset (PD argument only)


@dataclass(frozen=True)
class RhoCfg:
    independence_angle_deg: float = 90.0  # Paull 2013 (assumption), PLACEHOLDER until E3
    fold_opposite: bool = True            # owner 2026-11-04: Paull's line angle
    ordering: str = "paull_pairwise"      # owner 2026-11-04, after reading Paull §III-C (c1 >= c2); "sequential" = brief §6.4 form


@dataclass(frozen=True)
class AssociationCfg:
    gate_m: float = 2.0                  # project contact criterion
    chi2: float = 9.21                   # 99 % chi-square, 2 dof


@dataclass(frozen=True)
class ContactCfg:
    position_floor_sigma_m: float = 0.5  # brief §10 (engineering)
    common_mode_sigma_m: float = 1.0     # Perception's assumed sigma_nav (geometric_v1)
    min_indep_sigma_m: float = 0.05      # eigenvalue floor after removing the common mode
    prune_p: float = 1e-3
    max_contacts: int = 500
    confirm_looks: int = 2               # project contact criterion
    confirm_distinct_sectors: bool = False
    dirichlet_alpha0: float = 1.0


@dataclass(frozen=True)
class UncertaintyCfg:
    """Fallback detection covariance (brief §6.6) when a detection's covariance is invalid."""
    sigma_pose_m: float = 1.0
    sigma_yaw_deg: float = 2.0
    sigma_along_m: float = 0.3
    sigma_across_m: float = 0.2
    across_per_range: float = 0.01


@dataclass(frozen=True)
class PoseCfg:
    stamp_tolerance_s: float = 0.25      # brief §10
    frozen_count: int = 20               # brief §7 (>= 20 poses at exactly (0, 0))
    buffer_s: float = 30.0


@dataclass(frozen=True)
class PingCfg:
    unknown_water_depth_m: float = 0.22  # processor: altitude 0 (unknown) + 0.22 m submersion
    unknown_tol_m: float = 1e-4


@dataclass(frozen=True)
class AlignmentCfg:
    window: int = 200
    min_pairs: int = 10
    max_spread_m: float = 3.0
    latch: bool = True                   # owner 2026-11-04 (Q-14)
    origin_lat: float = NAN              # override: GPS of odom (0, 0)
    origin_lon: float = NAN
    override_sigma_m: float = 1.0
    queue_max: int = 6000
    sim_check_m: float = 0.5


@dataclass(frozen=True)
class CoverageCfg:
    require_detector: bool = True        # owner 2026-11-04 (Q-7)
    max_array_gap_s: float = 30.0
    wait_s: float = 60.0


@dataclass(frozen=True)
class TurnCfg:
    """Turn back-scan gating (Paull 2013 §II-C; Perception's r* = v/|w|, Cobra et al. 1992)."""
    enabled: bool = True
    window_s: float = 1.0                # Perception's turn_window_s (its decision 6.2)
    min_speed_mps: float = 0.05          # below this, r* = 0: the whole inner side is back-scanned
    min_yaw_rate_rad_s: float = 0.01     # below this, not turning (engineering)


@dataclass(frozen=True)
class AreaCfg:
    file: str = ""                       # world metadata.yaml or blueboat_survey_area/1 file


@dataclass(frozen=True)
class PublishCfg:
    grid_rate_hz: float = 1.0            # contract C-2
    display_rate_hz: float = 0.5


@dataclass(frozen=True)
class SnapshotCfg:
    period_s: float = 60.0


@dataclass(frozen=True)
class IoCfg:
    data_dir: str = ""


_SECTIONS = {
    "grid": GridCfg, "aspect": AspectCfg, "look": LookCfg, "prior": PriorCfg,
    "detector": DetectorCfg, "clamp": ClampCfg, "pd": PdCfg, "rho": RhoCfg,
    "association": AssociationCfg, "contact": ContactCfg, "uncertainty": UncertaintyCfg,
    "pose": PoseCfg, "ping": PingCfg, "alignment": AlignmentCfg, "coverage": CoverageCfg,
    "turn": TurnCfg, "area": AreaCfg, "publish": PublishCfg, "snapshot": SnapshotCfg, "io": IoCfg,
}

ORDERINGS = ("sequential", "paull_pairwise")


def _coerce(cls, name: str, value: Any, where: str) -> Any:
    ftype = {f.name: f for f in fields(cls)}[name].default
    if isinstance(ftype, bool):
        if not isinstance(value, bool):
            raise ConfigError(f"{where}.{name}: expected bool, got {value!r}")
        return value
    if isinstance(ftype, int):
        if isinstance(value, bool) or not isinstance(value, (int, float)) or int(value) != value:
            raise ConfigError(f"{where}.{name}: expected int, got {value!r}")
        return int(value)
    if isinstance(ftype, float):
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise ConfigError(f"{where}.{name}: expected float, got {value!r}")
        return float(value)
    if isinstance(ftype, str):
        if not isinstance(value, str):
            raise ConfigError(f"{where}.{name}: expected str, got {value!r}")
        return value
    if isinstance(ftype, tuple):
        if isinstance(value, str) or not isinstance(value, (list, tuple)):
            raise ConfigError(f"{where}.{name}: expected a list of str, got {value!r}")
        if not all(isinstance(v, str) for v in value):
            raise ConfigError(f"{where}.{name}: every entry must be a str")
        return tuple(value)
    raise ConfigError(f"{where}.{name}: unsupported type")  # pragma: no cover


def _strict(cls, d: Mapping[str, Any], where: str):
    known = {f.name for f in fields(cls)}
    unknown = set(d) - known
    if unknown:
        raise ConfigError(f"Unknown keys in [{where}]: {sorted(unknown)}")
    return cls(**{k: _coerce(cls, k, v, where) for k, v in d.items()})


@dataclass(frozen=True)
class BeliefConfig:
    grid: GridCfg = field(default_factory=GridCfg)
    aspect: AspectCfg = field(default_factory=AspectCfg)
    look: LookCfg = field(default_factory=LookCfg)
    prior: PriorCfg = field(default_factory=PriorCfg)
    detector: DetectorCfg = field(default_factory=DetectorCfg)
    clamp: ClampCfg = field(default_factory=ClampCfg)
    pd: PdCfg = field(default_factory=PdCfg)
    rho: RhoCfg = field(default_factory=RhoCfg)
    association: AssociationCfg = field(default_factory=AssociationCfg)
    contact: ContactCfg = field(default_factory=ContactCfg)
    uncertainty: UncertaintyCfg = field(default_factory=UncertaintyCfg)
    pose: PoseCfg = field(default_factory=PoseCfg)
    ping: PingCfg = field(default_factory=PingCfg)
    alignment: AlignmentCfg = field(default_factory=AlignmentCfg)
    coverage: CoverageCfg = field(default_factory=CoverageCfg)
    turn: TurnCfg = field(default_factory=TurnCfg)
    area: AreaCfg = field(default_factory=AreaCfg)
    publish: PublishCfg = field(default_factory=PublishCfg)
    snapshot: SnapshotCfg = field(default_factory=SnapshotCfg)
    io: IoCfg = field(default_factory=IoCfg)

    def __post_init__(self) -> None:
        validate(self)

    # -- construction --------------------------------------------------------------------
    @classmethod
    def from_dict(cls, d: Mapping[str, Any]) -> "BeliefConfig":
        """Nested ``{section: {key: value}}``; also accepts flat ``section.key`` keys and the
        ROS parameter-file wrapper ``{belief_map: {ros__parameters: {...}}}``."""
        d = unwrap_ros_params(d)
        nested: Dict[str, Dict[str, Any]] = {}
        for k, v in d.items():
            if "." in k:
                sec, key = k.split(".", 1)
                nested.setdefault(sec, {})[key] = v
            elif isinstance(v, Mapping):
                nested.setdefault(k, {}).update(v)
            else:
                raise ConfigError(f"Unknown top-level key: {k!r}")
        unknown = set(nested) - set(_SECTIONS)
        if unknown:
            raise ConfigError(f"Unknown config sections: {sorted(unknown)}")
        kw = {sec: _strict(_SECTIONS[sec], vals, sec) for sec, vals in nested.items()}
        return cls(**kw)

    @classmethod
    def from_flat(cls, flat: Mapping[str, Any]) -> "BeliefConfig":
        return cls.from_dict(dict(flat))

    def to_flat(self) -> Dict[str, Any]:
        """Flat ``section.key`` -> value, in a stable order (ROS parameter declaration)."""
        out: Dict[str, Any] = {}
        for sec in _SECTIONS:
            obj = getattr(self, sec)
            for f in fields(obj):
                v = getattr(obj, f.name)
                out[f"{sec}.{f.name}"] = list(v) if isinstance(v, tuple) else v
        return out

    def with_values(self, **sections: Mapping[str, Any]) -> "BeliefConfig":
        """A copy with some section keys replaced (tests and tools)."""
        kw = {}
        for sec, vals in sections.items():
            if sec not in _SECTIONS:
                raise ConfigError(f"Unknown config section: {sec!r}")
            cur = getattr(self, sec)
            known = {f.name for f in fields(cur)}
            bad = set(vals) - known
            if bad:
                raise ConfigError(f"Unknown keys in [{sec}]: {sorted(bad)}")
            kw[sec] = replace(cur, **{k: _coerce(type(cur), k, v, sec) for k, v in vals.items()})
        return replace(self, **kw)


def unwrap_ros_params(d: Mapping[str, Any]) -> Mapping[str, Any]:
    """``{node: {ros__parameters: {...}}}`` -> ``{...}`` (one node only)."""
    if len(d) == 1:
        (only,) = d.values()
        if isinstance(only, Mapping) and set(only) == {"ros__parameters"}:
            return only["ros__parameters"]
    return d


def validate(c: BeliefConfig) -> None:
    def need(cond: bool, msg: str) -> None:
        if not cond:
            raise ConfigError(msg)

    need(c.grid.resolution_m > 0 and math.isfinite(c.grid.resolution_m), "grid.resolution_m must be > 0")
    need(1 <= c.aspect.sector_count <= 16, "aspect.sector_count must be in 1..16")
    need(math.isfinite(c.aspect.origin_deg), "aspect.origin_deg must be finite")
    need(c.look.timeout_s > 0, "look.timeout_s must be > 0")
    need(c.look.time_reset_s > 0, "look.time_reset_s must be > 0")
    need(c.prior.density_per_ha >= 0, "prior.density_per_ha must be >= 0")
    need(0.0 < c.detector.base_rate < 1.0, "detector.base_rate must be in (0, 1)")
    need(len(c.detector.class_names) >= 1, "detector.class_names must not be empty")
    need(len(set(c.detector.class_names)) == len(c.detector.class_names), "detector.class_names must be unique")
    need(0.0 < c.clamp.p_min < c.clamp.p_max < 1.0, "clamp: need 0 < p_min < p_max < 1")
    need(0.0 <= c.pd.plateau <= 1.0, "pd.plateau must be in [0, 1]")
    need(c.pd.ramp_m > 0, "pd.ramp_m must be > 0")
    need(c.pd.r_lo_m <= c.pd.r_hi_m, "pd.r_lo_m must be <= pd.r_hi_m")
    need(0.0 <= c.pd.pfa < 1.0, "pd.pfa must be in [0, 1)")
    need(c.pd.lever_arm_m >= 0, "pd.lever_arm_m must be >= 0")
    need(c.rho.independence_angle_deg > 0, "rho.independence_angle_deg must be > 0")
    need(c.rho.ordering in ORDERINGS, f"rho.ordering must be one of {ORDERINGS}")
    need(c.association.gate_m > 0 and c.association.chi2 > 0, "association gates must be > 0")
    need(c.contact.position_floor_sigma_m >= 0 and c.contact.common_mode_sigma_m >= 0,
         "contact sigmas must be >= 0")
    need(c.contact.min_indep_sigma_m > 0, "contact.min_indep_sigma_m must be > 0")
    need(0.0 <= c.contact.prune_p < 1.0, "contact.prune_p must be in [0, 1)")
    need(c.contact.max_contacts >= 1, "contact.max_contacts must be >= 1")
    need(c.contact.confirm_looks >= 1, "contact.confirm_looks must be >= 1")
    need(c.contact.dirichlet_alpha0 > 0, "contact.dirichlet_alpha0 must be > 0")
    need(c.pose.stamp_tolerance_s > 0 and c.pose.buffer_s > 0, "pose tolerances must be > 0")
    need(c.pose.frozen_count >= 1, "pose.frozen_count must be >= 1")
    need(c.alignment.window >= 1 and c.alignment.min_pairs >= 1, "alignment window/min_pairs >= 1")
    need(c.alignment.min_pairs <= c.alignment.window, "alignment.min_pairs must be <= window")
    need(c.alignment.queue_max >= 1, "alignment.queue_max must be >= 1")
    need(math.isnan(c.alignment.origin_lat) == math.isnan(c.alignment.origin_lon),
         "alignment.origin_lat and origin_lon are set together")
    need(c.coverage.max_array_gap_s > 0 and c.coverage.wait_s > 0, "coverage times must be > 0")
    need(c.turn.window_s > 0 and c.turn.min_speed_mps >= 0 and c.turn.min_yaw_rate_rad_s > 0,
         "turn: window_s > 0, min_speed_mps >= 0, min_yaw_rate_rad_s > 0")
    need(c.publish.grid_rate_hz > 0 and c.publish.display_rate_hz > 0, "publish rates must be > 0")
    need(c.snapshot.period_s > 0, "snapshot.period_s must be > 0")


def load_yaml(path: str) -> BeliefConfig:
    """Load a YAML config file (PyYAML imported lazily: the core stays numpy + stdlib)."""
    import yaml  # noqa: PLC0415 - lazy by design

    with open(path, "r", encoding="utf-8") as fh:
        data = yaml.safe_load(fh) or {}
    if not isinstance(data, Mapping):
        raise ConfigError(f"{path}: top level must be a mapping")
    return BeliefConfig.from_dict(data)
