# CLAUDE.md — BlueBoat-Belief

The belief map (D4) of the BlueBoat aspect-aware side-scan survey: processed pings + detections in,
`BeliefGrid` out. It runs on the operator laptop like every node of the project.

```
/sss_processor/processed ─┐                                   ┌─► /belief_map/belief_grid (BeliefGrid) ─► replanner
/sss_ai/detections ───────┤  belief_map: adapters → ROS-free  ├─► /belief_map/query_belief, p_target, coverage, contacts
/blueboat/odom ───────────┤  core (alignment, coverage gate,  ├─► /diagnostics (rejection counters)
/mavros/.../global ───────┤  grid looks, Bernoulli contacts)  └─► <root>/data/Belief_data/<run>/snapshot_*.npz + .json
survey-area file ─────────┘
```

**PRIORITY: detector calibration.** Scores are uncalibrated today (Perception identity
calibration; no flag on the wire). `detector.calibrated: false`, so `p_exists` / `p_target` are
not probabilities. See `docs/REQUESTS.md` R-1 and `TODO.md` [P0].

**Authority.**
- The brief (`BELIEF_BRIEF.md`) wins on design where it disagrees with the root `CLAUDE.md` or
  `project_synthesis.md` (owner, 2026-11-04, D-1). The repository wins on facts.
- `docs/MATH.md` is the specification. `docs/topics.md` is the interface contract.
- Where this file and the code disagree, the code wins.

## 1. Layout
- `blueboat_belief_interfaces/` (ament_cmake): `msg/BeliefGrid.msg`, `msg/BeliefContact.msg`,
  `srv/QueryBelief.srv`. Depends only on `std_msgs` / `builtin_interfaces`. Never add types to
  `blueboat_interfaces`.
- `blueboat_belief/` (ament_python, one level down):
  - `blueboat_belief/core/`: numpy + stdlib only, deterministic:
    - `config` (strict dataclasses, flat `section.key`);
    - `geo` (MCS pair, bit-identical copy);
    - `area` (survey area, grid geometry);
    - `alignment` (odom→map median fit, latch, override);
    - `pose` (buffer, frozen-pose detector);
    - `models` (prior, PD placeholder, ρ table, miss combination);
    - `geometry` (look bearing, sector, sample positions);
    - `grid` (looks, misses, per-sector maxima);
    - `contacts` (association, existence, fusion, class);
    - `coverage` (detector-array timeline);
    - `turn` (turn back-scan gate, r* = v/|ω|);
    - `belief` (facade, event ordering, render, disc query);
    - `contract` (`to_grid_dict`, field tables).
  - `blueboat_belief/io/`: `missions` (area file), `snapshot` (deterministic npz + manifest),
    `dataroot`.
  - `blueboat_belief/ros/`:
    - `adapters.py`: ROS-free, duck-typed in; field dicts out;
    - `belief_node.py`: **the only rclpy file**.
  - `blueboat_belief/cli/replay.py`: `blueboat-belief-replay`, rosbags-based, deterministic.
  - `blueboat_belief/cli/make_area.py`: `blueboat-belief-make-area`, writes a validated survey-area file.
  - `launch/belief_launch.py` (optional ROS imports), `config/belief.yaml`, `config/survey_area.example.yaml`.
  - `test/smoke_test.py` (the gate), `test/scenarios.py` (dev-only synthetic inputs),
    `test/msg_reference/` (verbatim `ProcessedSSSPing.msg`, `vision_msgs`).
  - `docs/`: MATH, topics, DESIGN, SURVEY_AREA_FORMAT, REQUESTS, CONTRACT_PROVIDED,
    CONTRACT_DEVIATIONS, REFERENCES (read status of every source), LOCAL_CHECKLIST,
    HANDOVER_CLOUD, PHASE1_MEMO, PHASE2_DESIGN.

## 2. Interfaces (fields: `docs/topics.md`)
| Topic | Type | Dir | QoS |
|---|---|---|---|
| `/sss_processor/processed` | `blueboat_interfaces/ProcessedSSSPing` | in | BEST_EFFORT 50 |
| `/sss_ai/detections` | `vision_msgs/Detection2DArray` | in | RELIABLE 50 |
| `/blueboat/odom` | `nav_msgs/Odometry` | in | BEST_EFFORT 50 |
| `/mavros/global_position/global` | `sensor_msgs/NavSatFix` | in | BEST_EFFORT 10 |
| `/belief_map/belief_grid` | `blueboat_belief_interfaces/BeliefGrid` | out | RELIABLE, TRANSIENT_LOCAL 1, 1 Hz |
| `/belief_map/query_belief` | `QueryBelief` (srv) | out | — |
| `/belief_map/p_target`, `/belief_map/coverage` | `nav_msgs/OccupancyGrid` (display) | out | RELIABLE, TRANSIENT_LOCAL 1 |
| `/belief_map/contacts` | `visualization_msgs/MarkerArray` | out | default |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | out | default |

- **Clock.** Times are on the sonar-profile clock. Zero header stamps (real launch) become the
  node's arrival time. Use `use_sim_time:=false` on the real boat and `true` in Gazebo.
- **Map frame.** ENU about the survey area's `geo_anchor`.
  - `odom_to_map` = median GPS↔odom translation (yaw 0: odom yaw is absolute ENU), latched.
  - `odom_to_map_valid` is false before that.
  - Observations are queued, bounded, until valid.
- **Negative evidence is gated.** A ping counts as searched only where `/sss_ai/detections` arrays
  bracket it. Otherwise it is geometry-only.
- **Aspect fusion.** Paull 2013 Eqs. 22–24, pairwise with the larger confidence first
  (`rho.ordering: paull_pairwise`, the default since 2026-11-04), on the folded line angle.
- **Turn gate.** Inner-side samples beyond `r* = v/|ω|` count for nothing (Paull §II-C).
- **Mission-level coverage.** `summary.area_cumulative_pd` = Bourgault Eq. 7 for one target
  uniform in the area (`/diagnostics`, manifests).

## 3. Non-negotiables
1. No edits outside `BlueBoat-Belief/` (CM-3). Needs go to `docs/REQUESTS.md`.
2. Custom types only in `blueboat_belief_interfaces`. Never retype or rename an existing topic.
   QoS matches the other end (CM-1, CM-4).
3. Online code never reads simulation ground truth: no `ground_truth/*`, no `/sim/*`, no world
   `objects` (brief §13.4).
4. `core/` imports no ROS and no Qt; the adapters are ROS-free; rclpy lives only in
   `ros/belief_node.py`. Enforced by smoke [0].
5. Deterministic core: no wall clock, no randomness, sorted iteration. Snapshots are
   byte-identical on replay (smoke [12], [18]). Wall clock appears only in the manifest
   `provenance`.
6. Looks, not pings.
   - Misses use the effective PD `ρ·PD`, combined Paull-pairwise.
   - Back-scanned turn samples add no look.
   - Same-look replacement is computed from the look's opening values.
   - A contact miss is applied once, at look close.
7. Cite nothing from memory; `docs/REFERENCES.md` says what was read in the primary text.
   PD, PFA and ρ are **placeholders** until fitted. Never tune a number to improve a result
   (CM-13). Planner-model and world-model parameters stay separate.
8. Outputs are claimed with O_EXCL, never overwritten. Inputs and recordings are write-once (CM-7).
9. `BeliefGrid` carries everything a consumer needs (CM-9 consumer side). `docs/MATH.md` changes
   come before code changes, each with a closed-form test.
10. Never `git commit` or `git push` unless the owner explicitly asks (owner pushed to `main` on 2026-11-04).

## 4. Commands
```bash
colcon build --packages-up-to blueboat_belief                         # from ~/ros2_ws
ros2 launch blueboat_belief belief_launch.py area_file:=<world|area.yaml> [use_sim_time:=true] [record_inputs:=true]
blueboat-belief-replay <bag> --area <world|area.yaml> [--config cfg.yaml] [--msg-dir test/msg_reference]
blueboat-belief-make-area --out area.yaml --anchor LAT LON --local XMIN YMIN XMAX YMAX   # or --from-world <world>
ros2 topic echo --once /belief_map/belief_grid --qos-durability transient_local --field grid_seq
```

## 5. Gate
- `python3 -m test.smoke_test` from `blueboat_belief/`, no ROS. **188 checks, 0 skipped,
  ~29 s** (cloud session 2026-11-04, second pass).
  - `check()` fails fast. `SKIP` is printed when `rosbags` or the MCS checkout is absent;
    `pip install rosbags` gives sections [10] and [18].
- Shell pitfalls: do not source `~/ros2_ws/install/setup.bash` first (the venv `python3` may lack
  numpy/rosbags). Use `/usr/bin/python3` if in doubt.
- There is no lint or type-checker.

## 6. Status (update every session)
- **2026-11-04, cloud session BELIEF-1:** built and gated offline.
  - Interfaces package, core, adapters, node, launch and replay CLI exist; docs are complete.
  - **Not run in a ROS graph:** no `colcon build`, no live simulator, no fault-injection, no real
    replay. All of it is listed with commands in `docs/LOCAL_CHECKLIST.md`.
  - Performance is about 0.65 ms per swath on the cloud machine (indicative).
- **2026-11-04, same session, second pass** (the owner supplied the Paull 2013 and Bourgault 2006
  PDFs):
  - both were read in the primary text;
  - the default ordering is now `paull_pairwise`;
  - the turn back-scan gate is added;
  - `summary.area_cumulative_pd` is published;
  - `blueboat-belief-make-area` exists;
  - docs were corrected (`MATH.md` §13, `REFERENCES.md`).
  - Gate: 188/188.

## 7. Where things are documented
- Open work: `.claude/TODO.md`.
- In-machine steps: `blueboat_belief/docs/LOCAL_CHECKLIST.md`.
- Contract status: `docs/CONTRACT_PROVIDED.md`, `docs/CONTRACT_DEVIATIONS.md`.
- Superproject: the root `.claude/CLAUDE.md`; skills `module-gates`, `file-interfaces`
  (integration lines proposed in `docs/HANDOVER_CLOUD.md`).
