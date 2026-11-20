# REQUESTS: what BlueBoat-Belief needs from other modules (open items only)

Belief edits nothing outside `BlueBoat-Belief/` (CM-3). Each item states what is needed, why,
what Belief does meanwhile, and who decides. A met request is removed. What Belief offers its
consumer is in `topics.md` §10–§11.

## R-1 [P0] Perception: a calibrated detector, its base rate and its event

- **What.**
  - A calibrator (temperature or Platt scaling, or equivalent) fitted on held-out simulated
    worlds (synthesis §8.2), so that `score` is a probability for a stated event and population.
  - The **base rate π** of that population in the image record's `processing.calibration.params`.
  - A statement of the event the score is a probability of (a target of class k present in the
    box, at what overlap).
- **Why.** The contact update `L = logit q`, `L += ρ(logit q − logit π)` (`MATH.md` §9) is valid
  only for a calibrated posterior of the same event (brief §17.6; Saerens 2002). Today
  `p_exists` is a score-derived index.
- **Already on the wire.** `processing.calibration.calibrated` in every image record; Belief
  publishes it as `detector_calibrated_observed` and warns when its own configuration
  disagrees.
- **Belief meanwhile.** `detector.calibrated: false`, `detector.base_rate: 0.5` (placeholder);
  both are set by hand when a calibrated model is deployed.
- **Decides:** owner / Perception.

## R-2 [P2] Perception: a per-detection shadow field in the image record

- **What.** `shadow`: `none` | `weak` | `resolvable` (unknown allowed) per detection.
- **Why.** A resolvable shadow is the project's second way to confirm a contact (brief §6.5);
  `BeliefContact.shadow_known` is false until an input carries it.
- **Belief meanwhile.** Confirmation by two detected looks only.
- **Decides:** owner / Perception. An added record field is additive (CM-1).

## R-3 [P1] MCS: start Belief with `record_inputs:=true` on real-robot sessions

- **Why.** Control's run bag excludes `/sss_processor/*` and `/sss_ai/*` (`run_bag.py:52`), and
  the GCS "Save as rosbag" writes no `/sss_processor/processed`. Without Belief's own recording
  a real run cannot be replayed into Belief (CM-14).
- **Belief meanwhile.** `belief_launch.py record_inputs:=true` exists; the operator must
  remember it.
- **Decides:** owner (MCS, add-only).

## R-4 [P2] BlueBoat-Control: origin latch and zero stamps

- **Origin latch.** `robot_interface` latches the odom origin's lat/lon without a validity
  check, so `-origin.yaml` can read (0, 0) if no fix had arrived
  (`robot_interface.py:199,614-621`). Belief fits odom → map itself; a non-zero check would make
  the sidecar a safe override (`alignment.origin_lat/lon`).
- **Zero stamps.** `/blueboat/odom` and `/mavros/global_position/global` stamps can be 0 on the
  real launch (mavros `use_sim_time` without `/clock`). Belief uses arrival time and so needs
  `use_sim_time:=false` on the real boat.
- **Decides:** owner (Control, add-only).

## R-5 [P2] BlueBoat-SSS: unknown-altitude flag

- **What.** Without a bottom lock the processor publishes `water_depth = 0 + 0.22` and `*_y` =
  slant range (`sss_processor_node.py:1483-1498`). An explicit lock/quality field in
  `ProcessedSSSPing` (an additive Control type change) would be cleaner.
- **Belief meanwhile.** It detects the sentinel and drops the row (`altitude_unknown`).
- **Decides:** owner (SSS; Control for the message).

## R-6 [P2] Status gate for MCS SimGps fixes (noted, not Belief's)

`sss_processor_node.py:1050` and `bag_replay.py:215` reject `status < 0`, so they drop every MCS
SimGps fix (status −2). The project rule is that only −1 means no fix (root `CLAUDE.md` §4.4).
Belief follows the rule.

## R-7 [P2] BlueBoat-SSS: replay tooling for real recordings

- **`svlog_to_rosbag.py`** has hard-coded input and output paths and stamps on a synthetic
  clock. A command-line form and the recording's own time base would make real-recording
  replays usable for rate-dependent checks (the turn gate, the look timeout;
  `VALIDATION_REPORT.md` §4).
- **`sss_processor_node.py`** ends with a traceback and exit code −2 on the double SIGINT of
  `ros2 launch`, after publishing every row (`VALIDATION_REPORT.md` §4).
- **Decides:** owner (SSS).

## R-8 [done] Perception: place detections from the transducers' point (its CR-PER-2)

- **Done** (Perception `0d55d76`, 2026-11-15; re-checked 2026-11-18 by `~/ros2_ws/sim_runs/perception/handoff_12_cr_per_2.md`: Perception's convention-3 placement matches Belief's `sensor_xy` / `sample_xy` at four headings within 1 mm). Detection positions are computed from the seabed picture's georef with its fore/aft
  arm (`georef.lever_arm_m.x`, convention 3): `pose + R(yaw)·(arm, 0)` before the across-track offset, as
  the GCS and Belief place the pings. Belief needed no change (`DESIGN.md` L22 closed).
