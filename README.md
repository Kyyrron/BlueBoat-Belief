# BlueBoat-Belief

The belief map of the BlueBoat side-scan sonar survey. It takes every processed sonar ping and
every detector output, and keeps a running estimate of three things: **where targets probably
are**, **where the seabed has really been searched**, and **from which directions** it has been
searched. It publishes that estimate as one message, `BeliefGrid`.

> **Caveat: the scores are not probabilities yet.** No detector is trained, and Perception's
> calibration is identity. Belief runs with `detector.calibrated: false` and says so in every
> grid. Until a calibrated detector exists, `p_exists` and `p_target` are score-derived indices.
> With Perception's dummy detector, Belief reports zero coverage. That is correct: the dummy
> searches nothing.

## Where it fits

```
sonar ──► Perception ──► Belief ──BeliefGrid──► Replanner ──/blueboat/update_path──► Control
```

- `BeliefGrid` is the **only** target evidence the Replanner reads.
- Belief does not plan. The Replanner does not fuse detections.
- Like all project software, Belief runs on the operator laptop, not on the boat.

## How it works, in short

- **A miss is evidence.** A look that finds nothing makes a target there less likely.
- **Only where a detector searched.** A ping the detector never processed counts for nothing.
  Belief learns what was searched from Perception's image records (exact), or else from the
  timing of the detection arrays (an approximation).
- **Looks, not pings.** The many pings of one pass over a cell form one look.
- **Aspect.** Two looks are combined by the angle between them: two parallel looks add little,
  a perpendicular one adds a lot (Paull et al. 2013).
- **Turns.** The inner side of a turn is scanned backwards and distorted, so it is ignored.
- **Contacts.** Detections become contacts, each with an existence score and a class.
- **Deterministic.** Replaying the same inputs gives a byte-identical result.

The model, equation by equation: `blueboat_belief/docs/MATH.md`.

## Prerequisites

- ROS 2 Jazzy, workspace `~/ros2_ws`.
- `blueboat_interfaces` (in BlueBoat-Control), plus `vision_msgs`, `diagnostic_msgs`,
  `visualization_msgs`, `nav_msgs`, `sensor_msgs`.
- Python: `numpy`, `PyYAML`. The offline replay also needs `rosbags`, which is in the project
  venv `~/ros2_ws/.venv`.

## Build

From `~/ros2_ws`, in a shell that has **not** sourced the venv:

```bash
cd ~/ros2_ws
colcon build --packages-select blueboat_interfaces          # first, if not built yet
colcon build --packages-select blueboat_belief_interfaces blueboat_belief
source install/setup.bash
```

`blueboat_belief_interfaces` must be built before `blueboat_belief`.

## Run

```bash
# the node (use_sim_time:=true in Gazebo, false on the boat)
ros2 launch blueboat_belief belief_launch.py area_file:=<world folder | survey-area yaml>
```

Launch arguments:

| Argument | Default | Meaning |
|---|---|---|
| `area_file` | required | a simulator world folder, its `metadata.yaml`, or a survey-area file |
| `use_sim_time` | `false` | `true` in Gazebo |
| `config` | installed `config/belief.yaml` | parameter file |
| `record_inputs` | `false` | `true` also records Belief's inputs with `ros2 bag record`, for later replay |
| `data_dir` | `""` | data root for outputs (see below) |

The node is named `belief_map`.

### Make a survey-area file (for a real-boat run)

```bash
ros2 run blueboat_belief blueboat-belief-make-area --out area.yaml --anchor LAT LON --local XMIN YMIN XMAX YMAX
ros2 run blueboat_belief blueboat-belief-make-area --out area.yaml --from-world <world folder>
ros2 run blueboat_belief blueboat-belief-make-area --out area.yaml --from-mission <replanner mission.yaml>
```

The file is never overwritten. Format: `blueboat_belief/docs/SURVEY_AREA_FORMAT.md`; example:
`blueboat_belief/config/survey_area.example.yaml`.

### Read the grid

```bash
ros2 topic echo --once /belief_map/belief_grid --qos-durability transient_local --field grid_seq
```

### Replay a recorded bag offline

```bash
cd ~/ros2_ws && source env.sh       # venv (for rosbags) + workspace
python3 -m blueboat_belief.cli.replay <bag folder> --area <world | area.yaml> [--config cfg.yaml] [--out <folder>]
```

To drive the core from Python without ROS (e.g. an evaluation loop), see
`blueboat_belief/docs/OFFLINE_CORE_API.md`.

## Topics

Inputs:

| Topic | Type | From |
|---|---|---|
| `/sss_processor/processed` | `blueboat_interfaces/ProcessedSSSPing` | BlueBoat-SSS sonar processor |
| `/sss_ai/detections` | `vision_msgs/Detection2DArray` | BlueBoat-Perception |
| `/sss_ai/image_records` (optional) | `std_msgs/String` (JSON) | BlueBoat-Perception: which pings the detector searched |
| `/blueboat/odom` | `nav_msgs/Odometry` | BlueBoat-Control / Gazebo bridge |
| `/mavros/global_position/global` | `sensor_msgs/NavSatFix` | MAVROS / simulator shim |

Outputs:

| Name | Type | For |
|---|---|---|
| `/belief_map/belief_grid` | `blueboat_belief_interfaces/BeliefGrid` | the Replanner; latched (TRANSIENT_LOCAL), 1 Hz |
| `/belief_map/query_belief` | service `blueboat_belief_interfaces/QueryBelief` | summary of a disc around a point (convenience only) |
| `/belief_map/p_target`, `/belief_map/coverage` | `nav_msgs/OccupancyGrid` | display (RViz) |
| `/belief_map/contacts` | `visualization_msgs/MarkerArray` | display |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | counters, latency |

Every field, QoS and frame: `blueboat_belief/docs/topics.md`.

Good to know:
- **Map frame**: ENU about the survey area's `geo_anchor`. The grid says
  `odom_to_map_valid: false` until the GPS/odometry alignment has converged; do not plan before.
- **Evidence horizon**: every ping up to `BeliefGrid.header.stamp` is in the grid.
- Belief never reads simulation ground truth.

## Configuration

- Parameters: `blueboat_belief/config/belief.yaml` (flat `section.key` names, e.g.
  `grid.resolution_m: 0.5`, `aspect.sector_count: 12`).
- Override by passing your own YAML with `config:=`. An unknown parameter name stops the node.
- Detection model values (PD, PFA, ρ, prior density) are **placeholders** until fitted on a real
  detector.

## Outputs on disk

Snapshots (`snapshot_*.npz` + `.json`) go to `<root>/data/Belief_data/<run>/`. The root is
`data_dir`, else `$BLUEBOAT_DATA_DIR`, else the workspace (normally `~/ros2_ws`). Nothing is
ever overwritten.

## Tests

```bash
cd BlueBoat-Belief/blueboat_belief
test/gate.sh                                   # offline gate, no ROS; venv and system python
/usr/bin/python3 -B -m test.ros_graph_check    # ROS runtime check
```

- Do not source `~/ros2_ws/install/setup.bash` before `test/gate.sh`.
- `test.ros_graph_check` needs Belief sourced and `ROS_DOMAIN_ID` set to something other
  than 0. It refuses to run beside any other node. Never run it on a live graph.

## Layout

```
blueboat_belief_interfaces/   ament_cmake: msg/BeliefGrid, msg/BeliefContact, srv/QueryBelief
blueboat_belief/              ament_python package
  blueboat_belief/core/       the model: numpy only, no ROS, deterministic
  blueboat_belief/io/         survey-area and mission files, snapshots, data root
  blueboat_belief/ros/        adapters + belief_node.py (the only rclpy file)
  blueboat_belief/cli/        replay, make_area
  launch/belief_launch.py
  config/                     belief.yaml, survey_area.example.yaml
  test/                       smoke_test.py, gate.sh, ros_graph_check.py, scenarios, reference data
  docs/                       see below
```

## Status

- Offline gate green; built and checked under ROS 2 Jazzy (the published grid equals the
  offline core bit for bit).
- Run live in Gazebo beside the sonar processor, GCS and Perception (dummy detector), and on one
  real harbour recording.
- Run together with the Replanner on an isolated ROS graph with made-up inputs.
- **Not yet:** a real detector, or a real-boat session with Belief running.

Open work: `.claude/TODO.md`. Simulated results follow the superproject's
`SIM_VALIDITY_CONDITIONS.md`.

## Documents (`blueboat_belief/docs/`)

| File | Content |
|---|---|
| `MATH.md` | the model specification |
| `topics.md` | interface contract: topics, QoS, frames, every field, notes for consumers |
| `DESIGN.md` | design decisions, limitations, scientific status, open decisions |
| `OFFLINE_CORE_API.md` | using the core without ROS |
| `SURVEY_AREA_FORMAT.md` | the survey-area file |
| `VALIDATION_REPORT.md` | what was run, with commands and results |
| `REFERENCES.md` | sources and what each supports |
| `REQUESTS.md` | what Belief needs from other modules |
| `BELIEF_BRIEF.md` | the original design brief |

Main references: Bourgault, Furukawa & Durrant-Whyte (2006); Paull, Saeedi, Seto & Li (2013);
Yordanova & Gips (2020). Full entries: `blueboat_belief/docs/REFERENCES.md`.
