# CLAUDE.md — BlueBoat-Belief

The belief map (D4) of the BlueBoat aspect-aware side-scan survey: processed pings + detections in,
`BeliefGrid` out. It runs on the operator laptop like every node of the project.

```
/sss_processor/processed ─┐                                   ┌─► /belief_map/belief_grid (BeliefGrid) ─► replanner
/sss_ai/detections ───────┤  belief_map: adapters → ROS-free  ├─► /belief_map/query_belief, p_target, coverage, contacts
/blueboat/odom ───────────┤  core (alignment, coverage gate,  ├─► /diagnostics (rejection counters, latency)
/mavros/.../global ───────┤  grid looks, Bernoulli contacts)  └─► <root>/data/Belief_data/<run>/snapshot_*.npz + .json
/sss_ai/image_records ····┤  (optional: exact detector coverage)
survey-area file ─────────┘
```

**PRIORITY: detector calibration.** Scores are uncalibrated today (Perception identity
calibration). `detector.calibrated: false`, so `p_exists` / `p_target` are not probabilities.
See `docs/REQUESTS.md` R-1 and `TODO.md` [P0]. No trained detector exists yet either: with the
dummy detector Belief correctly reports zero coverage.

**Authority.**
- The brief (`blueboat_belief/docs/BELIEF_BRIEF.md`, frozen, never edited) wins on design where
  it disagrees with the root `CLAUDE.md` or `project_synthesis.md` (D-1). The repository wins on
  facts.
- `docs/MATH.md` is the specification. `docs/topics.md` is the interface contract.
  `docs/DESIGN.md` holds the decisions in force (D-/Q- IDs cited in code).
- Where this file and the code disagree, the code wins.
- **Docs state the current status only** (owner, 2026-11-12): no session logs, handovers or
  decision histories. Update a document in place; delete what is no longer true.

**Simulated results** follow `../SIM_VALIDITY_CONDITIONS.md` (superproject root; simulator frozen 2026-11-18,
literal verdict NOT READY, a model-conditional tool). For Belief and Replanner results: a time difference
inside **±3 % plus 0.6 s per turn** is not a result; simulated position error is up to about **1.1 m right
after a turn** (0.25 m on straight legs); compare strategies paired (same world, seeds and sea schedule).

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
    - `geometry` (look bearing, sector, the transducers' point, sample positions, the processor's
      side offsets);
    - `grid` (looks, misses, per-sector maxima);
    - `contacts` (association, existence, fusion, class);
    - `coverage` (detector coverage: array stamps, or image-record spans);
    - `turn` (turn back-scan gate, r* = v/|ω|);
    - `belief` (facade, event ordering, render, disc query);
    - `contract` (`to_grid_dict`, field tables).
  - `blueboat_belief/io/`: `missions` (area file; a Replanner mission file for `make-area`),
    `snapshot` (deterministic npz + manifest), `dataroot`.
  - `blueboat_belief/ros/`:
    - `adapters.py`: ROS-free, duck-typed in; field dicts out;
    - `belief_node.py`: **the only rclpy file**.
  - `blueboat_belief/cli/replay.py`: offline replay of a recorded bag (needs `rosbags`).
  - `blueboat_belief/cli/make_area.py`: writes a validated survey-area file.
  - `launch/belief_launch.py` (optional ROS imports), `config/belief.yaml`, `config/survey_area.example.yaml`.
  - `test/smoke_test.py` (the offline gate), `test/gate.sh` (runs it in both Python
    environments), `test/ros_graph_check.py` (the ROS runtime check, isolated domain only),
    `test/scenarios.py` (dev-only synthetic inputs), `test/msg_reference/` (verbatim
    `ProcessedSSSPing.msg`, `vision_msgs`), `test/data/` (one real Perception record).
  - `docs/` (nine files): `MATH` (spec), `topics` (contract, conformance, consumer notes),
    `DESIGN` (decisions, limitations, scientific status, open decisions), `OFFLINE_CORE_API`
    (driving the core without ROS; run health), `SURVEY_AREA_FORMAT`, `REQUESTS` (open needs
    from other modules), `REFERENCES` (read status of every source), `VALIDATION_REPORT` (what
    ran), `BELIEF_BRIEF` (the design brief).

## 2. Interfaces (fields: `docs/topics.md`)
| Topic | Type | Dir | QoS |
|---|---|---|---|
| `/sss_processor/processed` | `blueboat_interfaces/ProcessedSSSPing` | in | BEST_EFFORT 50 |
| `/sss_ai/detections` | `vision_msgs/Detection2DArray` | in | RELIABLE 50 |
| `/blueboat/odom` | `nav_msgs/Odometry` | in | BEST_EFFORT 50 |
| `/mavros/global_position/global` | `sensor_msgs/NavSatFix` | in | BEST_EFFORT 10 |
| `/sss_ai/image_records` (optional) | `std_msgs/String` JSON | in | RELIABLE 100, VOLATILE |
| `/belief_map/belief_grid` | `blueboat_belief_interfaces/BeliefGrid` | out | RELIABLE, TRANSIENT_LOCAL 1, 1 Hz wall |
| `/belief_map/query_belief` | `QueryBelief` (srv) | out | — |
| `/belief_map/p_target`, `/belief_map/coverage` | `nav_msgs/OccupancyGrid` (display) | out | RELIABLE, TRANSIENT_LOCAL 1 |
| `/belief_map/contacts` | `visualization_msgs/MarkerArray` | out | default |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | out | default |

- **Clock.** Times are on the sonar-profile clock. Zero header stamps (real launch) become the
  node's arrival time. Use `use_sim_time:=false` on the real boat and `true` in Gazebo. Outputs
  are paced by a steady clock, so they are published also when `/clock` is absent or paused.
- **Evidence horizon.** `BeliefGrid.header.stamp`: every ping at or before it is in the grid.
  It lags the newest ping by one to two image strides (`/diagnostics` `latency.event_age_s`).
- **Map frame.** ENU about the survey area's `geo_anchor`.
  - `odom_to_map` = median GPS↔odom translation (yaw 0: odom yaw is absolute ENU), latched.
  - `odom_to_map_valid` is false before that.
  - Observations are queued, bounded, until valid.
- **Placement from the transducers' point** (D-12, since 2026-11-15). Samples, contact-look
  ranges and detection poses start at `pose + a·(cos ψ, sin ψ)`, `a` = the ping's
  `transducer_x_offset` (−0.345 m from the processor; recordings made before 2026-11-15 carry
  0.0). The turn gate, the frozen-pose detector and the alignment keep the navigation pose.
  `ping.transducer_x_offset_source: config` + `ping.transducer_x_offset_m` re-place a recording.
  `model_params_json.transducer_x_offset` and `summary.transducer_x_offset_*` say which arm was
  applied. Belief hard-codes no arm and does not correct Perception's detections (R-8).
- **Negative evidence is gated.** A ping counts as searched only where the detector covered it:
  from the image records when they arrive (exact), else from the stamps of `/sss_ai/detections`
  arrays (a heuristic). `coverage.source: auto | arrays | records`.
- **Aspect fusion.** Paull 2013 Eqs. 22–24, pairwise with the larger confidence first
  (`rho.ordering: paull_pairwise`), on the folded line angle. Order-dependent beyond two looks.
- **Turn gate.** Inner-side samples beyond `r* = v/|ω|` count for nothing (Paull §II-C).
- **Mission-level coverage.** `summary.area_cumulative_pd` = Bourgault Eq. 7 for one target
  uniform in the area (`/diagnostics`, manifests). A planning aid, not a policy metric.

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
   - A look with ρ = 0 is an exact no-op, and Q never rises (smoke [6]).
   - Back-scanned turn samples add no look.
   - Same-look replacement is computed from the look's opening values.
   - A contact miss is applied once, at look close.
7. Cite nothing from memory; `docs/REFERENCES.md` says what was read in the primary text.
   PD, PFA and ρ are **placeholders** until fitted. Never tune a number to improve a result
   (CM-13). Planner-model and world-model parameters stay separate.
8. Outputs are claimed with O_EXCL, never overwritten. Inputs and recordings are write-once (CM-7).
9. `BeliefGrid` carries everything a consumer needs (CM-9 consumer side). `docs/MATH.md` changes
   come before code changes, each with a closed-form test.
10. Never `git commit` or `git push` unless the owner explicitly asks.
11. `test/ros_graph_check.py` runs only on a ROS domain other than 0 and beside no other node. It
    must never join a live graph.
12. **The Replanner reads this repository.** Its dev loop imports `core/` (via
    `OFFLINE_CORE_API.md`), records the sha256 of `core/*.py` and parses
    `ALL CHECKS PASSED (N checks, M skipped)` from `docs/VALIDATION_REPORT.md` into its manifests.
    Keep that line's format, and do not edit `core/*.py` while
    `pgrep -f blueboat_replanner.eval.runner` shows a run.

## 4. Commands
```bash
# build (from ~/ros2_ws, in a shell that has NOT sourced the venv)
colcon build --packages-select blueboat_belief_interfaces blueboat_belief
ros2 launch blueboat_belief belief_launch.py area_file:=<world|area.yaml> [use_sim_time:=true] [record_inputs:=true] [config:=<yaml>]
ros2 run blueboat_belief blueboat-belief-make-area --out area.yaml --anchor LAT LON --local XMIN YMIN XMAX YMAX
ros2 run blueboat_belief blueboat-belief-make-area --out area.yaml --from-world <world>     # or --from-mission <replanner mission.yaml>
ros2 topic echo --once /belief_map/belief_grid --qos-durability transient_local --field grid_seq
# offline replay of recorded inputs (rosbags is in the venv): from ~/ros2_ws after `source env.sh`
python3 -m blueboat_belief.cli.replay <bag folder> --area <world|area.yaml> [--config cfg.yaml]
```
- The console scripts live under `lib/blueboat_belief`: use `ros2 run`, they are not on `PATH`.
- A parameter override goes in a YAML file passed as `config:=`; an unknown parameter name
  stops the node (exit 2).

## 5. Gates
- **Offline:** `test/gate.sh` from `blueboat_belief/`: `python3 -m test.smoke_test` in the venv
  (numpy 2.4.4) and on the system python (numpy 1.26.4, the one ROS nodes import). Both must end
  with ALL CHECKS PASSED and 0 skipped. `check()` fails fast. Sections [10] and [18] need
  `rosbags` (in `~/ros2_ws/.venv`); [1] and [22] use the MCS and Replanner
  checkouts when they are beside this repository.
- **ROS runtime:** `test/ros_graph_check.py` on an isolated domain (header of the file).
- Do not source `~/ros2_ws/install/setup.bash` before the offline gate.
- There is no lint or type-checker.

## 6. Status (keep current; replace, do not append)
- **Offline gate** green in both Python environments (`VALIDATION_REPORT.md` §1).
- **Fore/aft arm (D-12, 2026-11-15)** applied in the source tree: with an arm of 0 every grid array
  and contact is identical to before (only the added `model_params_json` key differs). On the
  re-placed `b3a_sea_validation` recording, the four tyres' East and West passes went from
  0.68–0.70 m apart to ≤ 0.023 m (`VALIDATION_REPORT.md` §5.4). `~/ros2_ws/install` holds it
  (checked 2026-11-18: the installed package equals the source tree, built 2026-11-15).
- **ROS 2 Jazzy:** built into `~/ros2_ws/install`; on an isolated domain the published grid
  equals the offline core bit for bit; a recorded bag replays to a byte-identical snapshot.
- **Live in Gazebo** beside the real sonar processor, GCS and Perception (dummy detector): every
  ping applied, alignment at the world anchor, image records used, zero coverage as it must be;
  the fault-injection profile changed nothing.
- **One real recording** through the real `sss_processor_node`: every ping applied, alignment
  from real GPS; 12.7 % of the cells seen got more than single-look credit (sector steps); the
  turn gate acted on a quarter of the pings (qualitative, synthetic clock).
- **Consumer:** the Replanner drives the offline core in its dev loop and has implemented the
  consumer notes (`topics.md` §11); run together once under ROS, on an isolated graph with
  made-up sonar and Control (Replanner `.claude/tools/ros_check.sh`, 2026-11-12); never in
  Gazebo or with a real detector.
- **Not shown:** anything with a real detector (none is trained); any real-boat session with
  Belief running.

## 7. Where things are documented
- Open work: `.claude/TODO.md`. Decisions still open: `docs/DESIGN.md` §7.
- What ran and what did not: `blueboat_belief/docs/VALIDATION_REPORT.md`.
- Contract conformance and consumer notes: `docs/topics.md` §10–§11.
- Scientific status of each component and missing evidence: `docs/DESIGN.md` §5–§6.
- The README explains the module with its scientific background.
- Superproject: the root `.claude/CLAUDE.md`; skills `module-gates`, `file-interfaces`.
