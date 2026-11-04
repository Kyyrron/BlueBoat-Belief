# DESIGN: decisions behind BlueBoat-Belief (why the code is the way it is)

The specification is `MATH.md` and the interfaces are in `topics.md`. The history is
`PHASE1_MEMO.md` (facts) and `PHASE2_DESIGN.md` (the design and the owner's answers). All of
this is model-conditional (CM-13).

> **Priority: detector calibration** (owner, 2026-11-04). Perception's calibration is the identity,
> and no calibration flag reaches Belief. Belief's contact log-odds treat the raw score as `q`, so
> `p_exists` and `p_target` are **not probabilities** until a calibrated detector is deployed and
> `detector.calibrated` is set. This blocks every probabilistic claim (REQUESTS R-1, TODO [P0]).

## 1. Owner decisions (2026-11-04)

| # | Decision | Consequence in the code |
|---|---|---|
| D-1 | The brief wins on design where it disagrees with the root `CLAUDE.md` / synthesis; facts follow the repository | Inputs and outputs follow brief §3 / §4.2 |
| D-2 | Inputs: `/sss_processor/processed`, `/sss_ai/detections`, `/blueboat/odom`, `/mavros/global_position/global` + the survey-area file. **Deviates from CM-9** ("image records only") | `ros/belief_node.py` subscriptions |
| D-3 | odom → map: online translation-only median fit, latched; config override | `core/alignment.py` |
| D-4 | Survey area: world `metadata.yaml`, or a file of the same shape | `core/area.py`, `SURVEY_AREA_FORMAT.md` |
| D-5 | No context pack, no v1 | v1 comparisons are [NOT AVAILABLE]; closed forms replace them |
| D-6 | After Paull 2013 and Bourgault 2006 were read in the primary text: **default `rho.ordering = paull_pairwise`** (Paull's c¹ ≥ c²) | `core/config.py`, `models.combine_miss`; `sequential` stays a switch |
| D-7 | **Turn back-scan gating** (Paull §II-C; r* = v/|ω|) | `core/turn.py`, `Belief._process_ping` |
| D-8 | A minimal survey-area tool now | `cli/make_area.py` (`blueboat-belief-make-area`) |
| D-9 | Bearing-resolved ρ (Bourgault §4.3 smoothness) recorded as a TODO, not built | keeps `belief_grid/0` stable for the Replanner reconciliation |

The answers Q-1 … Q-14 (`PHASE2_DESIGN.md` §12) are applied as recommended, plus the calibration
priority (Q-8).

## 2. Architecture choices

1. **ROS-free deterministic core, thin node.**
   - `core/` is numpy + stdlib: no clock, no randomness, sorted iteration.
   - `ros/belief_node.py` is the only rclpy file. The import sweep and an AST check in the smoke
     test enforce both.
   - Times are explicit floats on the sonar-profile clock. The node replaces zero header stamps
     by its clock at arrival, because the real launch's mavros stamps can be 0.
2. **One code path from state to message (C-7).** `contract.to_grid_dict()` is the only producer
   of output values. `ros/adapters.grid_to_fields()` turns it into a nested field dict. A backend
   then materialises the dict: rclpy `fill()` in the node, rosbags `materialize()` in the tests.
   The CDR round-trip test therefore exercises the same values the node publishes.
3. **Inputs are duck-typed.** The adapters accept rclpy messages, rosbags dataclasses or dicts.
   Never truth-test a sequence field (numpy arrays); `_seq()` exists for that. The real layouts
   caught this bug.
4. **Offline replay = the determinism route.** `cli/replay.py` reads a rosbag2 with rosbags in
   log-time order. Smoke [18] shows a recorded mcap gives a byte-identical snapshot twice, and
   one identical to direct feeding. Live ROS replays are not bitwise reproducible (BEST_EFFORT
   drops, processor bursts), so they are not the criterion.

## 3. Model choices (details in `MATH.md`)

1. **Hybrid grid + contacts** (brief §6.1). The grid takes only misses. Contacts take positive
   evidence and their own looks.
2. **Looks, not pings**, with **same-look replacement computed from the look's opening
   values.**
   - `ln Q = f(ln Q_open, ρ, PD*)` and `ℓ = ℓ_open + Δ ln Q`.
   - Raising PD* within a look is therefore exact, equal to applying the look once at its close,
     and the grid stays live during a pass.
3. **ρ against searched sectors only**, combined pairwise. Paull fuses two looks with the larger
   confidence first (§III-C), and Belief applies that look by look against the accumulated
   coverage (D-6). A stronger repeat from the same aspect therefore lifts coverage to its PD
   (Eq. 22: max). Under the brief's sequential form it would add nothing.
   - A sector counts once a covered look with PD > 0 happened there.
   - A geometry-only look (no detector) carries no information, so it must not make later
     looks dependent (smoke [6]).
   - Folding opposite looks onto the line angle is Paull's α (owner Q-10). The minimum over
     earlier sectors is a heuristic; Paull defines only the pair (brief §17.4).
4. **Per-sector `aspect_log_miss` = ln(1 − max PD in the sector).**
   - It is Paull's α = 0 rule (full dependence ⇒ max), the brief §8.1 minimal fix.
   - It is published and decides nothing.
5. **Coverage gating on detection arrays** (owner Q-7; `MATH.md` §8).
   - Pings are held until the **second-newest** array stamp passes them. Images are 256 rows at
     stride 128, so by then every image a ping belongs to has arrived, together with all of its
     detections.
   - The cost is a latency of about two images (seconds) before the grid shows a pass.
   - It is a stated heuristic, because arrays carry only their newest row time (REQUESTS R-2).
6. **The dummy detector is not a detector.** Its arrays always hold `dummy_center`, so Belief
   marks those arrays non-coverage (geometry-only) and ignores the detections (owner Q-9).
7. **Contacts.**
   - One observation per look (the best-score detection), so the two overlapping images of one
     pass add one look.
   - Perception's σ_nav (1 m) is **common-mode**: it is removed from each observation before
     fusion and added back once. Without that, N looks would claim σ_nav/√N.
   - A miss is applied when the look closes. Closure is evaluated on event times only.
   - Association needs **both** ≤ 2 m and χ² ≤ 9.21. With Perception's σ ≥ 1 m the 2 m gate
     dominates (smoke [7] tests tight and broad covariances).
8. **Survey polygon = `limits.local` exactly;** `corners_gps` is the 0.05 m cross-check. Using the
   GPS-converted corners put the grid origin 1e-9 m off its round value.
9. **Unknown altitude.** `water_depth == 0.22` is the processor's "no altitude" sentinel, and its
   offsets are then slant range. Those rows are dropped, not credited (owner Q-11).

## 4. Known limitations (state them; do not hide them)

| # | Limitation | Where it bites | What would fix it |
|---|---|---|---|
| L1 | PD(r) is a **placeholder** trapezoid; aspect, class and seabed are omitted | every coverage / `p_background` number | a fitted PD (E3/E4); a fit on sim data and evaluation on sim data is circular (brief §8.5) |
| L2 | PFA is **neglected** (0): the Bourgault miss update is the PFA = 0 special case | `p_background` | a measured PFA at the 0.10 operating point (`pd.pfa` exists) |
| L3 | Raw scores used as `q`; base rate π = 0.5 unverified | `p_exists`, `p_target` | **the calibration priority** (R-1) |
| L4 | ρ is Paull's **assumed** linear rule (§III-C "it is assumed"); its look-by-look application and the min over sectors are heuristics beyond two looks | coverage after multi-aspect passes | E3 data |
| L5 | Coverage gating infers image spans from array stamps | which pings count as searched | R-2 meta companion |
| L6 | Nearest-neighbour association, no clutter model (not an RFS filter) | dense or cluttered scenes | PDA / labelled RFS (brief §17.7), if scenes demand it |
| L7 | Independent cells; background and contacts treated as independent when rendering | `p_target`, disc query | inherent to the hybrid approximation (Thrun 2003) |
| L8 | Class update = q-weighted Dirichlet counts, not Bayes | `class_probabilities` | a detector confusion matrix |
| L9 | Detections are georeferenced with the GCS compass heading; pings with odom yaw | a systematic offset between positive and negative evidence if the two disagree | LOCAL_CHECKLIST L-4 |
| L10 | `depth_m` is the boat's water depth assigned to the swath (flat floor); `last_intensity_db` is the device's scale (no gain compensation) | display / analysis only | — |
| L11 | Performance on the cloud machine: ~0.65 ms per swath, near the 1 ms budget | live headroom | measure on the laptop (LOCAL_CHECKLIST) |
| L12 | Nothing has run in a ROS graph | everything live | LOCAL_CHECKLIST |
| L13 | ρ is quantised by 30° sectors (steps of 1/3). Bourgault §4.3 warns that stepped models make a planner's objective jagged | heading optimisation in the Replanner | bearing-resolved ρ, a [P2] contract addition |
| L14 | The turn gate uses Perception's `r*` convention, and its 1 s window lags turn onset | the first ~1 s of each turn | live check (LOCAL_CHECKLIST B-3) |
| L15 | Paull's confidence (`P(m = M)`, 0.5–1) is not PD, so Paull's Eq. 9 is no consistency check here (`MATH.md` §13.1) | interpretation only | — |
