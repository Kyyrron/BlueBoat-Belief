# REQUESTS: what BlueBoat-Belief needs from other modules (addressed to the owner)

Belief edits nothing outside `BlueBoat-Belief/` (CM-3). Each item states what is needed, why, a
proposed specification, and who decides. They are ordered by priority.

## R-1 [P0] Perception: a calibrated detector, and a calibration flag on the wire

- **What.**
  - Fit a calibrator (temperature scaling or equivalent) on held-out simulated worlds (synthesis
    §8.2), so that `confidence` is a probability for a stated event and population.
  - Report the base rate π of that population.
  - Make the fact visible to Belief, which reads `/sss_ai/detections`, where `score` is the only
    confidence and no `calibrated` flag exists (Perception `ros/convert.py:47-75`).
- **Why.** Belief's contact update `L = logit q`, `L += ρ(logit q − logit π)` is valid only for a
  calibrated posterior of the same event (brief §17.6). Today `p_exists` is a score-derived
  index. Owner: **a priority** (2026-11-04).
- **Proposed spec.**
  - (a) The calibration's identity and base rate in R-2's meta companion.
  - (b) Until then, Belief's config `detector.calibrated` / `detector.base_rate` are set by hand
    when a calibrated model is deployed.
- **Decides:** owner / Perception.

## R-2 [P1] Perception: an optional per-image meta companion `/sss_ai/detection_meta`

- **What.** One JSON message per seabed image (std_msgs/String, RELIABLE, depth 50), keyed so
  Belief can join it to the `Detection2DArray` of the same image. Minimal fields:
  - the image key `<stream_id>/<image_id>` and the array stamp;
  - `t_min_s`, `t_max_s` (the image's row span);
  - `processing.detector.status` (`ran` / `no_pixels` / `no_detector_input`);
  - `detector.name`, `dummy`, `calibrated`, `calibration.name`, base rate;
  - per detection `uid`, `side`, `ground_range_m`, `shadow` (`none` | `weak` | `resolvable`;
    unknown allowed);
  - optionally the effective searched region (after `near_range_mask_m` and tile skips).
- **Why.** On the four inputs:
  - an empty array is ambiguous (nothing ≥ 0.10, or no pixels);
  - an array carries only its newest row time;
  - the dummy is recognisable only by class name.

  Belief's coverage gating (`MATH.md` §8) is therefore a stated heuristic. The companion would
  make it exact. It would also give the shadow flag behind the project's confirmation criterion
  (brief §6.5).
- **Belief without it:** works (gating heuristic; `shadow_known = false`).
- **Decides:** owner / Perception. This is a new additive topic (CM-1).

## R-3 [P1] Recording Belief's inputs on real runs

- **What.** Control's run bag excludes `/sss_processor/*` and `/sss_ai/*` (`run_bag.py:52`), and
  the GCS "Save as rosbag" writes no `/sss_processor/processed`. A real run therefore needs its
  own recording of the four inputs, for offline iteration (CM-14).
- **Done here.** `belief_launch.py record_inputs:=true` (owner Q-12).
- **Request.** MCS starts Belief with `record_inputs:=true` for real-robot sessions, once Belief
  is started by MCS. A future addition, add-only.
- **Decides:** owner (MCS).

## R-4 [P2] BlueBoat-Control: origin latch and zero stamps (existing open items, mitigated here)

- **Origin latch.** `robot_interface` latches the odom origin's lat/lon **without a validity
  check**, so `-origin.yaml` can read (0, 0) if no fix had arrived
  (`robot_interface.py:199,614-621`). Belief does not depend on it: it fits odom → map itself.
  A non-zero check would make the sidecar a safe override source (`alignment.origin_lat/lon`).
- **Zero stamps.** `/blueboat/odom` and `/mavros/global_position/global` stamps can be 0 on the
  real launch (mavros `use_sim_time` without `/clock`). Belief uses arrival time, and so needs
  `use_sim_time:=false` on the real boat. The source fix stays Control's.
- **Decides:** owner (Control, add-only).

## R-5 [P2] BlueBoat-SSS: unknown-altitude flag and measured fore/aft offset

- **Unknown altitude.** When the bottom tracker has no altitude the processor publishes
  `water_depth = 0 + 0.22` and `*_y` = slant range (`sss_processor_node.py:1483-1498`). Belief
  detects that sentinel and drops the row (`altitude_unknown`). An explicit lock/quality field
  would be cleaner: `ProcessedSSSPing` is Control's type, so it would be an additive field.
- **Fore/aft offset.** The fore/aft transducer offset is 0.0 (`TRANSDUCER_X_OFFSET_M`). Belief
  inherits the processor's geometry and adds no lever arm.
- **Decides:** owner (SSS, Control for the message).

## R-6 [P2] Status gate for MCS SimGps fixes (noted, not Belief's)

`sss_processor_node.py:1050` and `bag_replay.py:215` reject `status < 0`, so they drop every MCS
SimGps fix (status −2). The project rule is that only −1 means no fix (root `CLAUDE.md` §4.4).
Belief follows the rule.

## R-7 [P1] Replanner / evaluation (D6, D7): what Belief provides

- **Output contract.** `docs/topics.md` and `CONTRACT_PROVIDED.md`.
- **Coupling.**
  - A dev-only detection emulator publishing on `/sss_ai/detections` must not run beside
    Perception.
  - It must publish **one array per image, empty ones included**, or Belief must run with
    `coverage.require_detector: false` (labelled).
  - Its class names must be in Belief's `detector.class_names`.
- **Ground truth.** A ground-truth-driven oracle detector belongs to the Replanner's `eval/` or
  D7, not to Belief (owner Q-4).
- **Snapshots** for D7: `docs/topics.md` §8.
- **Model keys.** A predictor must follow `model_params_json.rho_model.ordering` (default
  `paull_pairwise`) and `turn_gating`.
- **Mission-level coverage.** `summary.area_cumulative_pd` (Bourgault Eq. 7, one target uniform
  in the area) is in `/diagnostics` and manifests. A whole-area `QueryBelief` gives the same
  number as `coverage_mean`.
- **Planner evidence (informational), primary text.**
  - Bourgault §4.4: greedy one-step search spirals and suffers the "scattering effect";
    lookahead gained +29 % / +22 %.
  - Paull Table III: information gain alone was worse than a lawnmower without branch entropy.

## R-8 Superproject integration (owner's commit; proposed text in the Phase 5 report)

- `git submodule add` of BlueBoat-Belief (already a gitlink).
- Root `CLAUDE.md` §1/§4 tables:
  - the new topics;
  - the `blueboat_belief_interfaces` types;
  - D-1, the deviation from CM-9's "image records only".
- A `module-gates` line; `file-interfaces` rows for snapshots, recorded inputs and the
  survey-area file.
- Build order: `colcon build --packages-up-to blueboat_belief` (the interfaces packages first).
