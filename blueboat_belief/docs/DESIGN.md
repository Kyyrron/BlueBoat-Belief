# DESIGN: why BlueBoat-Belief is built the way it is

The specification is `MATH.md`, the interface contract is `topics.md`, the design brief is
`BELIEF_BRIEF.md`. This page holds the decisions in force, the architecture and model choices,
the known limitations, the scientific status of every component and what is still to be
decided. Everything here is model-conditional (CM-13).

> **Priority: detector calibration.** Perception's calibration is the identity. Belief's contact
> log-odds treat the raw score as `q`, so `p_exists` and `p_target` are **not probabilities**
> until a calibrated detector is deployed and `detector.calibrated` is set. This blocks every
> probabilistic claim (`REQUESTS.md` R-1, `TODO.md` [P0]).

## 1. Decisions in force

Owner decisions; the IDs are cited in code and docs.

| # | Decision | Where in the code |
|---|---|---|
| D-1 | The brief wins on design where it disagrees with the root `CLAUDE.md` or `project_synthesis.md`; facts follow the repository | — |
| D-2 | Primary inputs: `/sss_processor/processed`, `/sss_ai/detections`, `/blueboat/odom`, `/mavros/global_position/global`, plus the survey-area file | `ros/belief_node.py` subscriptions |
| D-3 | odom → map: online translation-only median fit, latched once valid (Q-14); a config override exists | `core/alignment.py` |
| D-4 | Survey area: a world `metadata.yaml`, or a `blueboat_survey_area/1` file of the same shape; its `geo_anchor` is the datum (Q-2) | `core/area.py`, `SURVEY_AREA_FORMAT.md` |
| D-6 | `rho.ordering: paull_pairwise` (Paull's c¹ ≥ c²) is the default; `sequential` stays a switch. Opposite looks fold onto the line angle (`fold_opposite`, Q-10) | `core/config.py`, `models.combine_miss` |
| D-7 | Turn back-scan gating (Paull §II-C; r* = v/\|ω\|) | `core/turn.py` |
| D-8 | A minimal survey-area tool (`blueboat-belief-make-area`, also `--from-mission`) | `cli/make_area.py` |
| D-9 | Bearing-resolved ρ is a TODO, not built, so `belief_grid/0` stays stable | — |
| D-10 | Image records are an optional companion input, read for detector coverage only; the array heuristic stays the fallback (`coverage.source: auto`) | `core/coverage.RecordTimeline`, `Belief.ingest_record` |
| D-11 | `rosbags` lives in the project venv; the gate runs in the venv and on the system numpy | `test/gate.sh` |
| D-12 | Ping evidence is placed from the transducers' point, with the fore/aft arm each ping carries (`transducer_x_offset`; CR-BEL-1, 2026-11-15). The turn gate, the frozen-pose detector and the alignment keep the navigation pose (§3 item 10). `ping.transducer_x_offset_source: config` re-places a recording | `core/belief.py`, `core/geometry.sensor_xy` |
| Q-1 | `BeliefGrid` is TRANSIENT_LOCAL depth 1 **and** republished at 1 Hz | `ros/belief_node.py` |
| Q-3 | Per-sector miss accounting (`aspect_log_miss`) is published | `core/grid.py` |
| Q-4 | Synthetic inputs live only in `test/`; no ground-truth reader in Belief | `test/scenarios.py` |
| Q-6 | One gate runner: `test/smoke_test.py` | `test/` |
| Q-7 | Negative evidence is gated on detector coverage; `require_detector: false` is development only | `core/coverage.py` |
| Q-8 | The raw score is used as `q`, with `detector_calibrated: false` published | `core/contract.py` |
| Q-9 | The dummy class `dummy_center` is ignored and its arrays give no coverage | `core/config.py` |
| Q-11 | Unknown-altitude rows (`water_depth == 0.22`) give no evidence (counted) | `core/belief.py` |
| Q-12 | `record_inputs:=true` records Belief's inputs into its data folder | `launch/belief_launch.py` |
| Q-13 | The survey polygon is published (`area_x`, `area_y`) | `core/contract.py` |

Kept by the implementation and reported, each keeping the model as it is: the pairwise rule with
its order dependence beyond two looks stated (`MATH.md` §7); the sequential contact update
(`MATH.md` §9); no field of `belief_grid/0` added since the contract was implemented.

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

1. **Hybrid grid + contacts** (brief §6.1, D-1). The grid takes only misses. Contacts take positive
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
   - It is a stated heuristic, because arrays carry only their newest row time.
   - It is the fallback (D-10). When Perception's image records arrive, they give the exact row
     span and status of each image, and the release rule keeps the same latency (`MATH.md`
     §8.2). A record is applied only after its own detection array, so the belief does not
     depend on which topic is delivered first.
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
10. **Which point each use of a ping pose takes (D-12; `MATH.md` §1 item 7; smoke [23]).** The
    transducers sit `a_x` along the heading from the navigation pose (−0.345 m: aft). Ignored, the
    error flips with the heading: one object seen on two opposite passes lands `2|a_x|` = 0.69 m
    apart, more than a target's size (0.5–0.7 m), which is what association and revisit aiming
    rely on.

    | Use | Point | Why |
    |---|---|---|
    | samples (looks, misses, PD, intensity, depth) | transducers | where the sonar was |
    | a contact look's range on a covered ping (`look_ranges_m`) | transducers | the range at which the sonar saw the cell, from the same point as the samples |
    | the pose of a detection: side, bearing, range, fallback covariance | transducers | side and bearing are exactly unchanged (the shift is parallel to the heading); the range then means the same for detected and missed looks. It differs from the navigation point's by at most `a_x²/2r` (0.024 m at 2.5 m, 0.006 m at 10 m) |
    | odometry fallback of a detection pose | odometry moved by the newest ping's arm | consistent with the ping poses |
    | turn gate (ω, v, r*) | navigation pose | the along-heading velocity of every body point (a_x, y) is `v − ω·y`, whatever a_x, so `r* = v/\|ω\|` is the same; the transducers' chord speed `√(v² + ω²a_x²)` would bias it |
    | frozen-pose detector | navigation pose (the message's x, y) | it detects a published (0, 0); the transducers' point of such a pose is not (0, 0) |
    | odom → map fit | navigation pose | GPS ↔ odometry pairs, no sonar |

    - Detections are taken as Perception places them; Belief does not correct them. Perception
      applies the arm since 2026-11-15 (its CR-PER-2; L22 closed).
    - The arm comes with every ping, and Belief hard-codes none. A recording made before
      2026-11-15 carries 0.0 (the processor of that time) even when the simulator rendered it from
      −0.345 m. It is re-placed by a configuration value, not by a code path:
      `ping: {transducer_x_offset_source: config, transducer_x_offset_m: -0.345}` (and 0.0
      reproduces the old placement).
    - With an arm of 0 every grid array and contact is identical to the placement before D-12
      (`VALIDATION_REPORT.md` §1.1); `model_params_json.transducer_x_offset` states the arm.

## 4. Known limitations (state them; do not hide them)

| # | Limitation | Where it bites | What would fix it |
|---|---|---|---|
| L1 | PD(r) is a **placeholder** trapezoid; aspect, class and seabed are omitted | every coverage / `p_background` number | a fitted PD (E3/E4); a fit on sim data and evaluation on sim data is circular (brief §8.5) |
| L2 | PFA is **neglected** (0): the Bourgault miss update is the PFA = 0 special case | `p_background` | a measured PFA at the 0.10 operating point (`pd.pfa` exists) |
| L3 | Raw scores used as `q`; base rate π = 0.5 unverified | `p_exists`, `p_target` | **the calibration priority** (R-1) |
| L4 | ρ is Paull's **assumed** linear rule (§III-C "it is assumed"); its look-by-look application and the min over sectors are heuristics beyond two looks | coverage after multi-aspect passes | E3 data |
| L5 | Without image records, coverage gating infers image spans from array stamps: the first image after a start is never credited (3.9 % of one 100 m line) and a "no pixels" image is credited | which pings count as searched, for a detections-only publisher | the image records (used when present, `MATH.md` §8.2) |
| L6 | Nearest-neighbour association, no clutter model (not an RFS filter) | dense or cluttered scenes | PDA / labelled RFS (brief §17.7), if scenes demand it |
| L7 | Independent cells; background and contacts treated as independent when rendering | `p_target`, disc query | inherent to the hybrid approximation (Thrun 2003) |
| L8 | Class update = q-weighted Dirichlet counts, not Bayes | `class_probabilities` | a detector confusion matrix |
| L9 | Detections are georeferenced with the GCS compass heading; pings with odom yaw | a systematic offset between positive and negative evidence if the two disagree | real-boat check L-4 (`TODO.md`) |
| L10 | `depth_m` is the boat's water depth assigned to the swath (flat floor); `last_intensity_db` is the device's scale (no gain compensation) | display / analysis only | — |
| L11 | Python core: 0.50 ms per swath offline on the laptop; live in Gazebo 16 % of one core, 110 MB (`VALIDATION_REPORT.md`) | headroom with much larger grids only | measure again if the area or resolution grows |
| L12 | Run under ROS on an isolated domain with synthetic publishers, live in Gazebo with Perception's dummy detector, and on one real recording through the real sonar processor (`VALIDATION_REPORT.md`). **No run with a real detector exists**: none is trained | every claim about detections, contacts and negative evidence on real data | a trained, calibrated detector (Perception) |
| L13 | ρ is quantised by 30° sectors (steps of 1/3). Bourgault §4.3 warns that stepped models make a planner's objective jagged. On one real leg, 12.7 % of the cells seen were credited as more than one look because the heading crossed sector boundaries (`VALIDATION_REPORT.md` §4) | heading optimisation in the Replanner; coverage along a track that is not straight | bearing-resolved ρ, a [P1] additive contract field (`TODO.md`) |
| L14 | The turn gate uses Perception's `r*` convention, and its 1 s window lags turn onset | the first ~1 s of each turn | a real run with its own time base (`TODO.md` turn gate) |
| L15 | Paull formalises confidence as `P(m = M)` (0.5–1), so Paull's Eq. 9 is no consistency check here (`MATH.md` §13.1) | interpretation only | — |
| L16 | **Coverage depends on the order of three or more looks** (0.931 against 0.877 for the same three looks; `MATH.md` §7). Two-pass orthogonal is order-independent | `coverage`, `area_cumulative_pd` must not be used to compare survey policies | ground-truth evaluation (D7); a sourced n-look rule, if one exists |
| L17 | The contact update is sequential while the grid is pairwise: a stronger later detection on the same line does not raise `p_exists` (`MATH.md` §9) | `p_exists` only; `state` and `max_confidence` are unaffected | a pairwise form for contacts, once scores are calibrated |
| L18 | Perception's near-range slant mask (`near_range_mask_m`, 0 by default) is reported, not modelled | near-range coverage would be over-credited if the mask exceeded the altitude | apply the record's searched range per side |
| L19 | `/diagnostics`, the grid and the displays are paced by wall time. In a slowed simulation they are published more often per simulated second | bandwidth only | — |
| L20 | The detector's classes must be listed in `detector.class_names`; anything else becomes `other` (counted, and the node warns once) | a consumer that keys its model on the class | set the list from the trained model |
| L21 | The fore/aft arm (−0.345 m) is the simulator's value, adopted, not measured. On the real boat it counts from the navigation fix, which depends on where the GPS sits (the pool corpus read −0.60 m with the GPS near the bow) | real-boat placement along the heading, ±0.3 m | the owner's measurement (reciprocal passes over a point target, session 06 CR-OWN-1); then the processor's default |
| L22 | **Closed 2026-11-15:** Perception places its detections from the transducers' point (its CR-PER-2, Perception `0d55d76`), so a contact and the grid evidence of the same pass coincide along-track | — | `REQUESTS.md` R-8 (done); Belief needed no change |

## 5. Scientific status of each component

What the literature supports, what the project assumed, and what evidence a thesis claim would
need. Read status of every source: `REFERENCES.md`.

| Component | Formulation | Source and status | Unvalidated | Evidence a claim needs |
|---|---|---|---|---|
| Negative evidence on the grid | `p⁺ = p(1−PD)/(p(1−PD)+1−p)` in log-odds; `Q = ∏(1 − PD_i)` | Bourgault 2006 Eqs. 4–7 (read); Yordanova & Gips 2020 Eq. 2 (read). Bourgault keeps one normalised target PDF; Belief keeps independent cells | PD itself; cell independence | a fitted PD(range) with intervals; predicted against observed misses on labelled runs |
| PD(range) | trapezoid, 0.7 on 3–25 m, 3 m ramps | **placeholder** (brief §10); fitting precedent Nguyen & Murdoch 2012 (record only) | all of it | labelled looks of known targets, split by world |
| PFA | 0 | Bourgault's "perfect discrimination" (assumption) | whether false alarms are negligible at score ≥ 0.10 | false-alarm density on target-free seabed |
| Looks, not pings | a run of pings from one sector, gaps ≤ 20 s | Paull 2013 Eqs. 15–16 (read); timeout and 30° sectors are engineering choices | the timeout | — |
| Dependence between looks ρ | linear in the angle between look lines, pairwise with the stronger look first | Paull 2013 Eqs. 22–24, **stated there as an assumption** (read) | the rule; its extension beyond two looks | E3: the same targets from measured bearings |
| Contact existence | `L = logit q`; `+ρ(logit q − logit π)` per detected look; `+ln(1 − ρ·PD)` per missed look | base-rate shift (Saerens 2002, record); existence recursions with clutter (Mušicki 1994, Ristic 2013, records) | every number: scores are raw, π = 0.5 is a placeholder; no clutter model | a calibrated detector; reliability of `p_exists` on held-out worlds |
| Contact position | information-form fusion, one observation per look, common-mode error removed and added once | Perception's `geometric_v1` sigmas (assumed) | the sigmas | position error and ellipse coverage against known objects |
| Detector coverage | a ping counts only where a detector that ran spans it | owner Q-7 and D-10; brief §17.9 | on a live run with a real detector | share of pings credited, live |
| Turn gate | inner-side samples beyond `r* = v/\|ω\|` add nothing | Paull 2013 §II-C (read); radius as Perception uses it (Cobra 1992, record) | the 1 s estimator lag; behaviour on straight lines | a real run with its own time base |
| odom → map | median of GPS − odom pairs, latched; yaw 0 | real odom yaw is absolute ENU (`robot_interface.py`) | on a real run (L-1) | compass against odom yaw on a run bag |
| Sample placement | from the transducers' point, with each ping's fore/aft arm (D-12) | rigid-body geometry; the arm adopted from the simulator's profile (L21) | the arm on the boat | reciprocal passes over a point target on a real recording with a live clock |

**What Belief can claim today.** Its equations are implemented as specified and reproduce
closed forms; the node publishes what the core computes; a replay is deterministic. **What it
cannot claim:** any probability (scores are raw), any detection performance (PD is a
placeholder), anything about real detections (none exist).

**Assumptions built in.** PD(r), PFA = 0, π = 0.5, the linear ρ and its extension beyond two
looks, the 20 s look timeout, 30° sectors, the prior density 5 /ha, Perception's sigmas,
independent cells.

## 6. Missing evidence, and who provides it

| Missing | Blocks | Who |
|---|---|---|
| A trained detector | every statement about detections, contacts and negative evidence on real data | Perception |
| Its calibration and base rate | every probability | Perception (`REQUESTS.md` R-1) |
| E3 (heading response) | ρ, aspect windows, the claim that aspect matters | field work |
| A fitted PD and PFA | coverage numbers | Evaluation, on labelled runs |
| A Gazebo run of a replanner-owned path in a built world | E4 | SSS-Sim (Control R-3), then the Replanner |
| The Evaluation module | E4 scoring against ground truth | not owned |

**Thesis risks seen from Belief.** Every result is model-conditional (a simulator-trained
detector, an assumed PD, an assumed ρ, evaluated in the same simulator; CM-13). "Aspect-aware"
may show no effect, and a positive effect on Belief's ρ is still a property of Paull's assumed
rule until E3. Modelled coverage is order-dependent, so only ground truth compares policies.
Stating H1 (revisit early) and H2 (choose the bearing) as separate claims keeps a defensible
result either way.

## 7. Decisions still open (owner)

| # | Decision | Blocks |
|---|---|---|
| O-1 | `detector.class_names` once a model is trained, and who maps detector classes to the Replanner's aspect families (Replanner RQ-6) | aspect logic in the Replanner |
| O-2 | The prior density: Belief's 5 /ha is a planner-model placeholder; the Replanner's dev-loop worlds hold 10 /ha. Keep planner and world models separate; do not tune the prior to the world | `p_background`, the Replanner's background value |
| O-3 | Bearing-resolved ρ (`aspect_bearing_deg`, additive): agree with the Replanner | D-9 |
| O-4 | Survey speed 0.5 or 1.0 m/s (`project_synthesis.md` §8.5 says 1.0; the Replanner uses 0.5) | image cadence, latency, budgets |

## 8. How it works, and the science behind it

*Moved verbatim from the README (its §3, subsections 3.1–3.9 renumbered 8.1–8.9) on 2026-11-18; the README
keeps a summary.*
Everything below is specified with equations in `blueboat_belief/docs/MATH.md`, where each one
names the function that implements it and the gate section that checks it. The read status of
each source is in `docs/REFERENCES.md`:
- **read** = the primary text was read in a module session;
- **record** = only the bibliographic record and abstract were checked.

Numbers marked *placeholder* are not fitted to any data.

### 8.1 Search theory: negative evidence

Classical search theory treats every look that does not detect a target as information. Its
lateral-range and cumulative-detection ideas go back to Koopman (1956, record). Bourgault,
Furukawa and Durrant-Whyte (2006, **read**) give the recursion Belief uses:
- the miss likelihood `p(D̄|x) = 1 − PD(x)` (their Eq. 4);
- the probability of no detection so far, `Q_k = ∏ q_i` (Eq. 6), and the cumulative detection
  probability `P_k = 1 − Q_k` (Eq. 7).

Belief applies this **per grid cell**, conditioned on a target being in that cell:
- the grid's `coverage` is `1 − Q`, the modelled probability that a present target would have
  been detected by now;
- `p_background`, the probability of an *undetected* target, falls with every miss:
  `p⁺ = p(1 − PD) / (p(1 − PD) + 1 − p)`, in log-odds.

Yordanova and Gips (2020, **read**) use the same product, `∏(1 − Pd_i)`, as "residual risk" per
grid point.

Assumptions taken from these sources:
- **PFA = 0.** Bourgault's "perfect discrimination", a stated assumption: the false-alarm rate
  is neglected.
- **Independent cells.** Bourgault keeps one normalised target PDF. Belief keeps independent
  cells in the occupancy-grid style of Elfes (1989) and Thrun, Burgard and Fox (2005), with the
  known independent-cell caveat (Thrun 2003).

### 8.2 The detection model PD(range): a placeholder

In side-scan sonar, detection probability is described as a function of lateral range. Paull et
al. (2013, **read**) show such P(y) curves; they depend on seabed, target, sonar and vehicle.

Belief uses a trapezoid:
- PD = 0.7 between 3 m and 25 m ground range from the transducer;
- 3 m ramps at each end, with no steps (Bourgault §4.3 asks for smooth models);
- PD = 0 outside.

This is a **placeholder**. Fitting such a curve with confidence intervals from labelled looks
has precedents (Nguyen & Murdoch 2012, record). A simulation-fitted curve evaluated in
simulation would be circular.

### 8.3 Looks, not pings

At 20 Hz, a 0.5 m cell is hit by many consecutive pings of one pass. Those pings are not
independent trials. Counting them as such would make one pass look like certainty.

Paull et al. count one observation per cell per ping line, "given that there is no overlap"
(Eqs. 15–16). Belief's equivalent for dense pings is the **look**: a run of observations of a
cell from one 30° aspect sector, with gaps of at most 20 s.
- A look contributes **once**, with the best PD it reached.
- Raising that PD inside the look is exact, because the look's terms are recomputed from its
  opening values.
- Sector width and timeout are engineering choices.

### 8.4 Aspect: how much a second look adds

Paull et al. (2013, §III-C, **read**) fuse two looks at angle α between their lines, with the
larger confidence first:
- parallel looks (α = 0) give `max(c₁, c₂)` (Eq. 22);
- perpendicular looks give `1 − (1 − c₁)(1 − c₂)` (Eq. 23), i.e. independent looks;
- in between, `c₁ + ρ·c₂(1 − c₁)` with ρ linear in α (Eq. 24).

The paper states this linear rule **as an assumption** ("it is assumed"). Paull defines the rule
only for a pair of looks. Belief applies it look by look against the accumulated coverage, with
ρ the minimum over the sectors already searched.
- Opposite sides of the same line fold together: Paull's α is a line angle.
- The 12 × 30° sectors follow the 30° aspect increments of Fawcett et al. (2010, record).
- The value of additional views is context from Myers and Williams (2012, record; SAS).

Consequences, stated and pinned by the gate:
- **Order dependence.** Beyond two looks, coverage depends on the order of the looks (0.931 vs
  0.877 for the same three looks). The two-pass orthogonal baseline is order-independent (0.91).
  So modelled coverage is a **planning aid, never a metric for comparing survey policies**.
- **Stepped ρ.** ρ moves in steps of 1/3 because it uses sector centres. On a real harbour leg,
  12.7 % of the cells seen were credited as more than one look where the heading crossed a
  sector boundary. A bearing-resolved ρ is the open fix (`TODO.md`).

### 8.5 Negative evidence only where a detector searched

A ping counts as searched only where a real detector covered it. This was an owner decision.
Belief knows that coverage in one of two ways:
- **Image records** (exact). Perception publishes one record per seabed image: the image's time
  span, whether the detector ran on pixels, and whether it was the dummy. A ping inside a
  searched span is covered.
- **Array stamps** (heuristic fallback). Detection-array stamps bracket the pings between them.
  This fallback never credits the first image after a start, and it credits images that had no
  pixels.

Both ways release a ping only once every image containing it has been reported. Seabed images
are 256 rows at a 128-row stride, so each image overlaps only its two neighbours. As a result
the grid's `header.stamp` is an exact **evidence horizon**: every ping at or before it is in
the grid. The cost is a lag of one to two images (a few seconds).

Pings without detector coverage are kept as geometry only: they count as seen, but are not
evidence.

### 8.6 Turns: the inner swath is scanned backwards

In a turn, the outer side's swath fans out and leaves gaps. The inner side is "completely
distorted" for target recognition (Paull 2013 §II-C, **read**).

Belief estimates the yaw rate ω and the speed v over 1 s of ping poses. Inner-side samples
beyond `r* = v/|ω|` add no look and no aspect. The radius is the one Perception uses (Cobra et
al. 1992, record). Without this gate, the sweeping heading of a turn would hand the same cells
several fake aspects.

### 8.7 Contacts: positive evidence

The grid takes only misses. Detections form **contacts**, a table inspired by Bernoulli objects
(Mahler 2007): an existence probability plus a Gaussian position.

**Association.** A detection joins the nearest contact within 2 m whose Mahalanobis distance
passes χ² = 9.21; otherwise it creates a new contact.

**Existence, in log-odds:**
- created at `logit q`, where q is the look's best score;
- each later detected look adds `ρ·(logit q − logit π)`. This is the prior-shift correction:
  a calibrated posterior carries its training base rate π, which must be removed before looks
  are combined (Saerens et al. 2002, record);
- each covered, missed look adds `ln(1 − ρ·PD)`.

This is **not** a full Bernoulli or IPDA filter: there is no clutter model, and association is
nearest-neighbour. Ristic et al. (2013) and Mušicki et al. (1994), both record, are the
references a reader should compare it with.

**Calibration** is the precondition for all of it. Modern neural networks are poorly calibrated
(Guo et al. 2017, record, for classifiers), and a detector's calibration can depend on box
position and scale (Küppers et al. 2020, record), which for side-scan means range.

**Position.** Fused in information form, one observation per look. Perception's navigation
error (1 m) is shared by every detection, so it is removed before fusion and added back once;
otherwise N looks would claim σ/√N.

**State.** A contact is *confirmed* after two detected looks.

### 8.8 Frames, rendering, queries

**Map frame.** ENU about the survey area's anchor.
- The odom → map translation is the median of GPS − odom pairs, latched once its spread is small.
- The yaw offset is 0, because real odom yaw is absolute ENU.
- Until the fit is valid, observations are queued and `odom_to_map_valid` is false.

**Outputs derived from the grid and contacts:**
- `p_target = 1 − (1 − p_bg)·∏(1 − p_c·w_c)` combines background and contacts, as an
  approximation that treats them as independent.
- `QueryBelief` uses each contact's own Gaussian mass inside the queried disc, never the
  rendered map.
- `summary.area_cumulative_pd` is Bourgault's Eq. 7 for one target placed uniformly in the
  area. It is a planning aid.

### 8.9 Determinism

The core is numpy and stdlib only. It has no wall clock, no randomness, and iterates in sorted
order. A recorded input bag therefore replays to a **byte-identical** snapshot, and the live
node's published grid equals the offline core bit for bit. This is how perception and AI work
is iterated offline rather than in the water (CM-14).
