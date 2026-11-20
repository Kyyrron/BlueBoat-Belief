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
| `/sss_processor/processed` (`ProcessedSSSPing`, BEST_EFFORT) | BlueBoat-SSS `sss_processor_node` | per ping: the boat pose, the transducers' fore/aft arm (`transducer_x_offset`: the samples are placed from the transducers' point, `docs/MATH.md` §1), the water depth, signed ground offsets and intensities per side |
| `/sss_ai/detections` (`Detection2DArray`, RELIABLE) | BlueBoat-Perception | one array per seabed image: detections with score ≥ 0.10, positioned in odom |
| `/sss_ai/image_records` (JSON, optional) | BlueBoat-Perception | per image: its row time span, whether a real detector ran on pixels, and the calibration flag |
| `/blueboat/odom` | BlueBoat-Control (real) / SSS-Sim bridge (Gazebo) | poses for the odom → map fit |
| `/mavros/global_position/global` | MAVROS / SSS-Sim shim | GPS fixes for the same fit |
| survey-area file | SSS-Sim World Builder (`metadata.yaml`), or `blueboat-belief-make-area` | the grid extent and the map datum (`geo_anchor`) |

Geodesy is a bit-identical copy of BlueBoat-MCS's (`core/geo.py`, checked in the gate). Belief
never reads simulation ground truth. Full contract: `blueboat_belief/docs/topics.md`.

## 3. How it works (the full explanation with the science: `blueboat_belief/docs/DESIGN.md` §8)

- **Search theory, negative evidence** (8.1): every look that does not detect a target is information
  (Koopman; the recursion of Bourgault et al. 2006); the detection model PD(range) is a placeholder until a
  calibrated detector exists (8.2).
- **Looks, not pings** (8.3): the many pings of one pass over a cell are not independent trials.
  **Aspect** (8.4): two looks are fused by the angle between their lines (Paull et al. 2013: parallel looks
  give `max(c₁, c₂)`).
- **Negative evidence only where a detector searched** (8.5), from Perception's image records.
- **Turns** (8.6): the inner swath is scanned backwards and distorted (Paull 2013 §II-C), so it is gated (D-7).
- **Contacts** (8.7): positive evidence, Bernoulli contacts; **frames, rendering, queries** (8.8);
  **determinism** (8.9): a replay of the same inputs is byte-identical.

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
| ROS 2 Jazzy, isolated domain | built into `~/ros2_ws/install`; the published grid equals the offline core bit for bit; QoS, latch, service, exit codes and the final snapshot checked. The fore/aft arm (2026-11-15) was checked against a scratch build; the shared install holds it (checked 2026-11-18) |
| Fore/aft arm | with an arm of 0 nothing moves; on a re-placed simulated run the four tyres' two opposite passes went from 0.68–0.70 m apart to ≤ 0.023 m (`docs/VALIDATION_REPORT.md` §5.4); the arm itself is adopted, not measured |
| Real recording | one real harbour leg through the real sonar processor: every ping applied, alignment from real GPS (4 mm spread) |
| Live Gazebo, dummy detector | every ping applied beside the real processor, GCS and Perception; image records used; zero coverage, correctly; the fault-injection profile changed nothing |
| Consumer | the Replanner drives Belief's offline core in its dev loop, and its validator accepts a real Belief grid (gate [22]); the Replanner's node ran beside `belief_map` on an isolated ROS graph with made-up sonar and Control (2026-11-12) |
| **A real detector, a real boat session with Belief** | **not yet**: no detector is trained |
| Simulator | frozen 2026-11-18 (`sim-freeze-v1.0-2026-11-18`, literal acceptance verdict NOT READY, used as a model-conditional tool); simulated results follow the superproject's `SIM_VALIDITY_CONDITIONS.md` |

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

Full entries, DOIs and read status of every source: `docs/REFERENCES.md`. The three read in full and
used for the model: Bourgault, Furukawa, Durrant-Whyte (2006), Paull, Saeedi, Seto, Li (2013), Yordanova,
Gips (2020); the others are cited for context only.
