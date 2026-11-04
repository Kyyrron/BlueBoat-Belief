# TODO — BlueBoat-Belief (open items only)

Legend:
- **[P0]** blocks a scientifically valid result.
- **[P1]** correctness or cross-module risk.
- **[P2]** realism, project value.

Each item is dated. Done items are removed, not kept.

### [P0] Detector calibration: p_exists is not a probability until it exists (2026-11-04)
- **Situation.** Perception calibration is the identity, and `/sss_ai/detections` has no
  calibration flag.
- **What to do.**
  - Deploy a calibrated detector, calibrated on held-out simulated worlds (synthesis §8.2).
  - Then set `detector.calibrated: true` and `detector.base_rate` to the calibration
    population's prevalence.
  - Request R-1 (calibration flag / meta companion R-2).
- **Until then.** No `p_exists` / `p_target` number may be reported as a probability.

### [P0] Run in a ROS graph (2026-11-04)
- Nothing has run under ROS: `colcon build`, node start, QoS matching, live simulator run.
- Follow `docs/LOCAL_CHECKLIST.md` B-1 … B-6. It is the first job of the in-machine session
  (BELIEF-2-LOCAL).

### [P1] Contract reconciliation with the Replanner (2026-11-04)
- Diff `docs/CONTRACT_PROVIDED.md` against the Replanner's `CONTRACT_CONSUMED.md`, field by field.
- Settle the deviations D-C1 … D-C6.
- Settle the emulator coupling: one array per image, never beside Perception.

### [P1] Coverage-gating heuristic: verify on a live run (2026-11-04)
- Image spans are inferred from array stamps (MATH §8).
- Measure on a built-world run with the dummy and with YOLO (LOCAL_CHECKLIST L-7): the share of
  pings released as covered against geometry-only, and the release latency.
- Replace the heuristic with the meta companion if Perception adds it (R-2).

### [P1] Compass against odom yaw offset (2026-11-04)
- Detections use the GCS compass heading; pings use odom yaw.
- Measure the offset (L-4). If it is > ~2°, model it (covariance) or ask SSS for per-row heading
  provenance.

### [P1] Performance headroom on the laptop (2026-11-04)
- Cloud: ~0.65 ms per swath at 0.5 m against a 1 ms budget.
- Measure on the target (B-5). If it is tight, vectorise `Belief._process_ping` contact
  observation and `PoseBuffer` (lists today).

### [P2] Replace placeholders with fitted models (2026-11-04)
- PD(r) (fitted with confidence intervals; Nguyen & Murdoch 2012 is a precedent, not read here).
- PFA at the 0.10 operating point.
- ρ from E3.
- Keep planner-model and world-model files separate. A simulation-fitted PD evaluated in
  simulation is circular (brief §8.5).

### [P2] Bearing-resolved ρ (contract addition) (2026-11-04)
- ρ uses 30° sector centres, so predicted coverage steps by 1/3 with heading.
- Bourgault 2006 §4.3 (primary): stepped observation models make a planner's objective jagged.
- Proposal: `rho.angle_source: sector_centre | look_bearing` plus an optional ADDED
  `aspect_bearing_deg` (W·H·S float32, empty unless enabled; ~1 MB at 24 000 cells).
- Settle with the Replanner (B-7) before building.

### [P2] PD as an expectation over unknown seabed / sonar parameters (2026-11-04)
- Paull 2013 Eqs. 11–13 (primary): expected confidence over the parameter set Z, uniform when
  unknown, refined in situ (Table I).
- When a fitted PD exists, make it a seabed-class mixture rather than one curve. Keep the
  planner-model file separate (brief §8.5).

### [P1] Turn gate: verify live (2026-11-04)
- In a lawnmower run: `stats.backscan_*` rise only in turns. Check whether the 1 s window lag at
  turn onset matters (LOCAL_CHECKLIST B-3).

### [P2] Orientation-marginalised aspect model (brief §8.1 "later") (2026-11-04)
- Only with E3 data, and behind a model switch. Do not build before the data exist.

### [P2] Non-rectangular survey areas (2026-11-04)
- `polygon_gps` is reserved in `SURVEY_AREA_FORMAT.md` (a new format version). Build it together
  with the configuration tool.
