# TODO — BlueBoat-Belief (open items only)

Legend:
- **[P0]** blocks a scientifically valid result.
- **[P1]** correctness or cross-module risk.
- **[P2]** realism, project value.

Done items are removed, not kept. What has been run is in
`blueboat_belief/docs/VALIDATION_REPORT.md`; the decisions still open are in `docs/DESIGN.md` §7.

## [P0]

### Detector calibration: `p_exists` is not a probability until it exists
- Perception's calibration is the identity; Belief sees it in the image records
  (`detector_calibrated_observed`).
- Needed: a calibrated detector, calibrated on held-out simulated worlds (synthesis §8.2), and the
  base rate of its calibration population (`docs/REQUESTS.md` R-1).
- Then set `detector.calibrated: true` and `detector.base_rate`, and check the reliability of
  `p_exists` on held-out worlds.
- Until then no `p_exists` / `p_target` number may be reported as a probability.

### A run with a real detector
- No trained YOLO weights exist; detections, contacts and negative evidence have run on
  synthetic messages only. With the dummy detector Belief correctly shows zero coverage.
- When a model exists:
  1. set `detector.class_names` to its classes (the 29 World Builder types; anything unlisted
     becomes `other`; DESIGN O-1);
  2. run `test/live_sim_check.py` with it (Gazebo + GCS + Perception + Belief);
  3. compare contacts with the world's objects **offline**: localisation error, ellipse
     coverage, share of pings credited (`stats.pings_covered / pings`);
  4. on two opposite passes over one object, compare each contact with the grid evidence of the
     same pass. Since D-12 the grid is placed from the transducers' point; until Perception
     applies the arm (`docs/REQUESTS.md` R-8) a contact sits 0.345 m along-track from it.

## [P1]

### Real-boat checks
Each needs a real run; L-1, L-2 and L-4 can use the v1.1 run bags already on disk
(`~/ros2_ws/data/Robot_data/2026_10_28-*/…-bag`, with `/blueboat/odom`,
`/mavros/global_position/compass_hdg` and `/mavros/global_position/global`). L-8 needs
`/sss_processor/processed`, which run bags exclude, so it needs a session with Belief's inputs
recorded (`record_inputs:=true`).
- **L-1 Odom yaw is absolute ENU.** Compare `yaw(odom)` with `radians(90 − hdg)` (wrapped) over
  a run, with `rosbags`. Pass: the median |Δ| is a few degrees and does not equal the launch
  heading. It justifies `odom_to_map_yaw = 0` (`core/alignment.py`).
- **L-2 Zero stamps and zero covariance on the real launch.** Record whether `/blueboat/odom` and
  `/mavros/global_position/global` stamps are 0, whether the odom covariance is zero, and the GPS
  `status` values seen. Belief then needs `use_sim_time:=false` (arrival time).
- **L-4 Compass against odom yaw.** Detections are placed with the GCS compass heading, pings
  with odom yaw. Record the offset distribution; above ~2°, model it in the detection covariance
  or ask SSS for per-row heading provenance (DESIGN L9).
- **L-8 Frozen (0, 0) ping poses.** Count `/sss_processor/processed` rows with
  `robot_x == robot_y == 0`; Belief's `frozen_pose` must match and `/diagnostics` must report
  ERROR.
- **L-9 The fore/aft arm on the wire.** On the first real launch with Belief, read
  `/diagnostics` `summary.transducer_x_offset_min_m` / `_max_m`: both must be the processor's
  value (−0.345 m today). 0.0 means a processor installed before 2026-11-15
  (`colcon build --packages-select blueboat_sss`). The value itself is adopted, not measured
  (`DESIGN.md` L21).

### Turn gate on straight lines
- Live in Gazebo the gate acted through the half turn as intended, and also on about one ping in
  ten of the return leg: the yaw rate exceeded `v / range` (0.04 rad/s at 0.8 m/s and 20 m) while
  the boat held the line (`VALIDATION_REPORT.md` §5.2). On a real leg replayed from an `.svlog`
  it removed part of 464 of 1961 pings, a qualitative count (synthetic clock, §4).
- Check on a real run with its own time base (Belief's recorded inputs, or a run bag's odometry)
  whether such yaw rates occur on a line, and whether the 1 s window and
  `turn.min_yaw_rate_rad_s` are right. Do not tune them on simulated motion alone.

### Bearing-resolved ρ (an additive contract field)
- ρ uses 30° sector centres, so predicted coverage steps by 1/3 with heading. On one real leg
  replayed ungated, 12.7 % of the cells seen were credited as more than one look (up to 0.931,
  from up to four sectors), because the bearing crossed sector boundaries
  (`VALIDATION_REPORT.md` §4). Bourgault 2006 §4.3: stepped models make a planner's objective
  jagged.
- Proposal: `rho.angle_source: sector_centre | look_bearing`, plus an optional ADDED
  `aspect_bearing_deg` (W·H·S float32, empty unless enabled; ~1 MB at 24 000 cells).
- Settle with the Replanner before building (DESIGN O-3), then `MATH.md` first, with a
  closed-form test.

### Replanner on its next revision
- The Replanner has implemented the consumer notes (`docs/topics.md` §11) and drives Belief's
  offline core. When it commits, re-run the gate: smoke [22] runs its `validate_grid` on a real
  Belief grid.

## [P2]

### ROS runtime check: endpoint-discovery race in [R1]
- `test/ros_graph_check.py` [R1] waits until the node's five subscriptions are discovered, then
  looks up its publishers and service at once. On 2026-11-15 two of seven runs failed those
  lookups (6 and 1 checks, "None"), while [R2]–[R5] of the same runs passed. The reruns passed
  38/38 (`VALIDATION_REPORT.md` §3.2).
- Fix: wait (same 20 s bound) for the `/belief_map/*` publishers and the service too.

### Replace placeholders with fitted models
- PD(r) with confidence intervals (Nguyen & Murdoch 2012 is a precedent, record only), PFA at
  the 0.10 operating point, ρ from E3.
- Keep planner-model and world-model files separate. A simulation-fitted PD evaluated in
  simulation is circular (brief §8.5).
- When a fitted PD exists, make it an expectation over seabed classes (Paull 2013 Eqs. 11–13,
  Table I) rather than one curve.

### Apply the record's searched range
- The image record carries the searched region per side and `near_range_mask_m`; Belief reports
  a non-zero mask and does not apply it (DESIGN L18). Do it when a mask larger than the altitude
  is used.

### Order-independent multi-look rule
- The look-by-look pairwise rule depends on the order of three or more looks (`MATH.md` §7).
  Adopt a replacement only from a source or from E3 data, with a closed-form test. Until then,
  never compare survey policies on modelled coverage.

### Pairwise contact update
- The contact update is sequential, the grid pairwise (`MATH.md` §9). Revisit once scores are
  calibrated; it needs a rule for mixed detections and misses.

### Orientation-marginalised aspect model (brief §8.1 "later")
- Only with E3 data, and behind a model switch.

### Non-rectangular survey areas
- `polygon_gps` is reserved in `SURVEY_AREA_FORMAT.md` (a new format version). `make-area
  --from-mission` takes the bounding rectangle of the Replanner's polygon for now.
