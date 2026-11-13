# BlueBoat-Belief

The belief map (thesis deliverable D4) of the BlueBoat aspect-aware side-scan sonar survey.
From every processed sonar ping and every detector output, it keeps a recursive estimate of
three things:
- **where targets probably are**;
- **where the seabed has really been searched**;
- **from which aspects** it has been searched.

It publishes that estimate as one message, `BeliefGrid`, which is the only thing the Replanner
reads.

> **Read this first: the scores are not probabilities yet.**
> - Perception's detector is uncalibrated (calibration = identity), and no detector is trained
>   yet.
> - Belief runs with `detector.calibrated: false` and publishes that flag in every grid. Until a
>   calibrated detector is deployed, `p_exists` and `p_target` are score-derived indices, not
>   probabilities (`blueboat_belief/docs/REQUESTS.md` R-1).
> - With Perception's dummy detector, Belief reports **zero coverage**, which is correct: the
>   dummy searches nothing.

---

## 1. Why a belief layer

**The problem.** A side-scan sonar sees a target well only from some directions:
- a cylinder lying along the track and one across it give very different returns;
- a shadow can be hidden by the target's own orientation.

So one pass that finds nothing does not mean "nothing is there". It means "nothing detectable
from that aspect, at that range, by this detector". The thesis asks one question: in a
calibrated simulation, does choosing *where and from which heading to look again* find targets
that a seriously tuned two-pass orthogonal survey misses, and where does it stop paying
(`project_synthesis.md` §1–§4)?

Answering that needs a state that records three things:
- **negative evidence**: how much less likely an undetected target is after a look that found
  nothing;
- **only where a detector truly searched**: a ping the detector never saw is no evidence;
- **the aspect history**: a second look along the same line adds little, and a perpendicular one
  adds a lot.

**Why one module owns it.** The project rule is CM-9: the belief layer is the only object the
planner reads.
- Detector, sensor and planner changes then interact through one documented contract
  (`belief_grid/0`).
- A survey policy is never tuned against detector internals.
- Belief does not plan, and the Replanner does not fuse detections.

## 2. Place in the chain

```
 Cerulean Omniscan 450 SS ×2 ──► sss_node.py ──► sss_processor_node ──► /sss_processor/processed ──┐
   (BlueBoat-SSS, or BlueBoat-SSS-Sim's simulated sonar: same interface, CM-2)                     │
                                 GCS seabed images ──► BlueBoat-Perception ──► /sss_ai/detections ─┤
                                                                         └──► /sss_ai/image_records┤ (optional)
 robot_interface / Gazebo bridge (BlueBoat-Control) ──────────────────────► /blueboat/odom ───────┤
 MAVROS (autopilot GPS) ──────────────────────────────────────────► /mavros/global_position/global ┤
 survey-area file (a world metadata.yaml, or blueboat_survey_area/1) ────────────────────────────────┘
                                                   │
                                     BlueBoat-Belief: belief_map
                                                   │
       /belief_map/belief_grid (BeliefGrid) ──► BlueBoat-Replanner ──► /blueboat/update_path ──► Control
       /belief_map/query_belief, p_target, coverage, contacts, /diagnostics ──► operator, RViz
       <root>/data/Belief_data/<run>/snapshot_*.npz + .json ──► offline evaluation
```

| Input | Owner | What Belief takes from it |
|---|---|---|
| `/sss_processor/processed` (`ProcessedSSSPing`, BEST_EFFORT) | BlueBoat-SSS `sss_processor_node` | per ping: the boat pose, the water depth, signed ground offsets and intensities per side |
| `/sss_ai/detections` (`Detection2DArray`, RELIABLE) | BlueBoat-Perception | one array per seabed image: detections with score ≥ 0.10, positioned in odom |
| `/sss_ai/image_records` (JSON, optional) | BlueBoat-Perception | per image: its row time span, whether a real detector ran on pixels, and the calibration flag |
| `/blueboat/odom` | BlueBoat-Control (real) / SSS-Sim bridge (Gazebo) | poses for the odom → map fit |
| `/mavros/global_position/global` | MAVROS / SSS-Sim shim | GPS fixes for the same fit |
| survey-area file | SSS-Sim World Builder (`metadata.yaml`), or `blueboat-belief-make-area` | the grid extent and the map datum (`geo_anchor`) |

Geodesy is a bit-identical copy of BlueBoat-MCS's (`core/geo.py`, checked in the gate). Belief
never reads simulation ground truth. Full contract: `blueboat_belief/docs/topics.md`.

## 3. How it works, and the science behind it

Everything below is specified with equations in `blueboat_belief/docs/MATH.md`, where each one
names the function that implements it and the gate section that checks it. The read status of
each source is in `docs/REFERENCES.md`:
- **read** = the primary text was read in a module session;
- **record** = only the bibliographic record and abstract were checked.

Numbers marked *placeholder* are not fitted to any data.

### 3.1 Search theory: negative evidence

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

### 3.2 The detection model PD(range): a placeholder

In side-scan sonar, detection probability is described as a function of lateral range. Paull et
al. (2013, **read**) show such P(y) curves; they depend on seabed, target, sonar and vehicle.

Belief uses a trapezoid:
- PD = 0.7 between 3 m and 25 m ground range from the transducer;
- 3 m ramps at each end, with no steps (Bourgault §4.3 asks for smooth models);
- PD = 0 outside.

This is a **placeholder**. Fitting such a curve with confidence intervals from labelled looks
has precedents (Nguyen & Murdoch 2012, record). A simulation-fitted curve evaluated in
simulation would be circular.

### 3.3 Looks, not pings

At 20 Hz, a 0.5 m cell is hit by many consecutive pings of one pass. Those pings are not
independent trials. Counting them as such would make one pass look like certainty.

Paull et al. count one observation per cell per ping line, "given that there is no overlap"
(Eqs. 15–16). Belief's equivalent for dense pings is the **look**: a run of observations of a
cell from one 30° aspect sector, with gaps of at most 20 s.
- A look contributes **once**, with the best PD it reached.
- Raising that PD inside the look is exact, because the look's terms are recomputed from its
  opening values.
- Sector width and timeout are engineering choices.

### 3.4 Aspect: how much a second look adds

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

### 3.5 Negative evidence only where a detector searched

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

### 3.6 Turns: the inner swath is scanned backwards

In a turn, the outer side's swath fans out and leaves gaps. The inner side is "completely
distorted" for target recognition (Paull 2013 §II-C, **read**).

Belief estimates the yaw rate ω and the speed v over 1 s of ping poses. Inner-side samples
beyond `r* = v/|ω|` add no look and no aspect. The radius is the one Perception uses (Cobra et
al. 1992, record). Without this gate, the sweeping heading of a turn would hand the same cells
several fake aspects.

### 3.7 Contacts: positive evidence

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

### 3.8 Frames, rendering, queries

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

### 3.9 Determinism

The core is numpy and stdlib only. It has no wall clock, no randomness, and iterates in sorted
order. A recorded input bag therefore replays to a **byte-identical** snapshot, and the live
node's published grid equals the offline core bit for bit. This is how perception and AI work
is iterated offline rather than in the water (CM-14).

## 4. What Belief does not claim

- Every result is **model-conditional** (CM-13). PD(range), PFA = 0, ρ, the 20 s look timeout,
  the 5 /ha prior and the base rate π = 0.5 are all assumptions or placeholders.
- Scores are raw, so no output is a probability yet.
- Modelled coverage depends on the order of looks, so only ground truth compares survey
  policies.
- Nothing has been shown with a real detector.

The full list of limitations, and the evidence each claim would need, is in
`docs/DESIGN.md` §4–§6.

## 5. Status

Evidence levels are kept apart (`docs/VALIDATION_REPORT.md`):

| Level | Result |
|---|---|
| Offline gate (`test/gate.sh`) | green in both Python environments of the laptop, 0 skipped |
| ROS 2 Jazzy, isolated domain | built into `~/ros2_ws/install`; the published grid equals the offline core bit for bit; QoS, latch, service, exit codes and the final snapshot checked |
| Real recording | one real harbour leg through the real sonar processor: every ping applied, alignment from real GPS (4 mm spread) |
| Live Gazebo, dummy detector | every ping applied beside the real processor, GCS and Perception; image records used; zero coverage, correctly; the fault-injection profile changed nothing |
| Consumer | the Replanner drives Belief's offline core in its dev loop, and its validator accepts a real Belief grid (gate [22]); the Replanner's node ran beside `belief_map` on an isolated ROS graph with made-up sonar and Control (2026-11-12) |
| **A real detector, a real boat session with Belief** | **not yet**: no detector is trained |

Open work: `.claude/TODO.md`.

## 6. Use

```bash
# build (from ~/ros2_ws, in a shell that has not sourced the venv)
colcon build --packages-select blueboat_belief_interfaces blueboat_belief

# run (use_sim_time:=true in Gazebo, false on the boat)
ros2 launch blueboat_belief belief_launch.py area_file:=<world folder | survey-area yaml> \
     [use_sim_time:=true] [record_inputs:=true] [config:=<yaml>]

# a survey-area file for the real boat
ros2 run blueboat_belief blueboat-belief-make-area --out area.yaml --anchor LAT LON --local XMIN YMIN XMAX YMAX
ros2 run blueboat_belief blueboat-belief-make-area --out area.yaml --from-mission <replanner mission.yaml>

# read the latched grid
ros2 topic echo --once /belief_map/belief_grid --qos-durability transient_local --field grid_seq

# offline replay of recorded inputs (deterministic); rosbags is in the project venv
cd ~/ros2_ws && source env.sh && python3 -m blueboat_belief.cli.replay <bag folder> --area <file>

# gates
cd blueboat_belief && test/gate.sh                  # offline, no ROS: venv and system python
/usr/bin/python3 -B -m test.ros_graph_check         # ROS runtime, isolated domain only (see the file)
```

- A parameter override goes in a YAML file passed as `config:=`. An unknown parameter name stops
  the node.
- To drive the core from Python without ROS (for an evaluation loop, or the Replanner's dev
  loop), see `docs/OFFLINE_CORE_API.md`.

## 7. Layout and documents

| Package | Type | Content |
|---|---|---|
| `blueboat_belief_interfaces/` | ament_cmake (rosidl) | `msg/BeliefGrid`, `msg/BeliefContact`, `srv/QueryBelief` |
| `blueboat_belief/` | ament_python | `core/` (ROS-free deterministic model), `io/`, `ros/` (adapters; `belief_node.py` is the only rclpy file), `cli/` (replay, make-area), `launch/`, `config/`, `test/` |

| Document (`blueboat_belief/docs/`) | For |
|---|---|
| `MATH.md` | the model specification, equation by equation, with the primary-source reading (§13) |
| `topics.md` | the interface contract: topics, QoS, frames, every field, which field answers which question, conformance, notes for a consumer |
| `DESIGN.md` | decisions in force, architecture and model choices, limitations, scientific status, open decisions |
| `OFFLINE_CORE_API.md` | driving the core without ROS; run-health counters |
| `VALIDATION_REPORT.md` | what was run, with commands and results, and what was not |
| `REFERENCES.md` | every source, what it supports, what it does not, and whether it was read |
| `REQUESTS.md` | what Belief needs from other modules |
| `SURVEY_AREA_FORMAT.md` | the `blueboat_survey_area/1` file |
| `BELIEF_BRIEF.md` | the design brief the module was built from |

## 8. Main references

Full entries, DOIs and read status: `docs/REFERENCES.md`.
- Bourgault, Furukawa, Durrant-Whyte (2006). *Optimal Search for a Lost Target in a Bayesian
  World.* FSR, STAR 24, 209–222. **Read.**
- Paull, Saeedi, Seto, Li (2013). *Sensor-Driven Online Coverage Planning for Autonomous
  Underwater Vehicles.* IEEE/ASME T-Mech 18(6), 1827–1838. **Read.**
- Yordanova, Gips (2020). *Coverage Path Planning With Track Spacing Adaptation for Autonomous
  Underwater Vehicles.* IEEE RA-L 5(3). **Read** (arXiv preprint).
- Fawcett et al. (2010), IEEE JOE 35(4); Myers, Williams (2012), IEEE JOE 37(1); Nguyen,
  Murdoch (2012), NRL 59(8); Cobra, Oppenheim, Jaffe (1992), IEEE JOE 17(3); Koopman (1956),
  Oper. Res. 4(5); Elfes (1989); Thrun, Burgard, Fox (2005); Thrun (2003); Mahler (2007);
  Ristic et al. (2013), IEEE TSP 61(13); Mušicki, Evans, Stankovic (1994), IEEE TAC 39(6);
  Saerens et al. (2002), Neural Comput. 14(1); Guo et al. (2017), ICML; Küppers et al. (2020),
  CVPRW. Cited for context only, as `REFERENCES.md` states for each.
