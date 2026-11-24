# BlueBoat-Belief — user guide

How to run and use the belief map, in detail. For installation, see the [README](../../README.md).
Every command assumes `cd ~/ros2_ws && source env.sh` was run first.

## 1. Run the node

```bash
ros2 launch blueboat_belief belief_launch.py area_file:=<world folder | survey-area yaml>
```

| Launch argument | Default | Meaning |
|---|---|---|
| `area_file` | required | A simulator world folder, its `metadata.yaml`, or a survey-area file |
| `use_sim_time` | `false` | `true` in Gazebo |
| `config` | installed `config/belief.yaml` | Parameter file |
| `record_inputs` | `false` | `true` also records Belief's inputs with `ros2 bag record`, for a later replay |
| `data_dir` | `""` | Data root for the outputs (see section 6) |

The node is named `belief_map`. Read the grid's sequence number while it runs:

```bash
ros2 topic echo --once /belief_map/belief_grid --qos-durability transient_local --field grid_seq
```

## 2. Make a survey-area file (for a real-boat run)

In simulation the world folder is the area. On the real boat, write an area file:

```bash
ros2 run blueboat_belief blueboat-belief-make-area --out area.yaml --anchor LAT LON --local XMIN YMIN XMAX YMAX
ros2 run blueboat_belief blueboat-belief-make-area --out area.yaml --from-world <world folder>
ros2 run blueboat_belief blueboat-belief-make-area --out area.yaml --from-mission <replanner mission.yaml>
```

The file is never overwritten. Format: [SURVEY_AREA_FORMAT.md](SURVEY_AREA_FORMAT.md); example:
`blueboat_belief/config/survey_area.example.yaml`.

## 3. Replay a recorded run offline

```bash
cd ~/ros2_ws && source env.sh       # the venv has rosbags
python3 -m blueboat_belief.cli.replay <bag folder> --area <world | area.yaml> [--config cfg.yaml] [--out <folder>]
```

The replay of the same inputs gives a byte-identical result. To drive the core from Python without
ROS (an evaluation loop, for example), see [OFFLINE_CORE_API.md](OFFLINE_CORE_API.md).

## 4. Topics

Inputs:

| Topic | Type | From |
|---|---|---|
| `/sss_processor/processed` | `blueboat_interfaces/ProcessedSSSPing` | BlueBoat-SSS sonar processor |
| `/sss_ai/detections` | `vision_msgs/Detection2DArray` | BlueBoat-Perception |
| `/sss_ai/image_records` (optional) | `std_msgs/String` (JSON) | BlueBoat-Perception: which pings the detector searched |
| `/blueboat/odom` | `nav_msgs/Odometry` | BlueBoat-Control or the Gazebo bridge |
| `/mavros/global_position/global` | `sensor_msgs/NavSatFix` | the autopilot link, or the simulator's stand-in |

Outputs:

| Name | Type | For |
|---|---|---|
| `/belief_map/belief_grid` | `blueboat_belief_interfaces/BeliefGrid` | The Replanner; kept for late subscribers, once per second |
| `/belief_map/query_belief` | service `blueboat_belief_interfaces/QueryBelief` | A summary of a disc around a point (convenience only) |
| `/belief_map/p_target`, `/belief_map/coverage` | `nav_msgs/OccupancyGrid` | Display, for example in RViz |
| `/belief_map/contacts` | `visualization_msgs/MarkerArray` | Display |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | Counters, delays |

Every field, delivery setting and frame: [topics.md](topics.md).

Good to know:

- **Map frame**: east-north-up metres about the survey area's GPS anchor. The grid says
  `odom_to_map_valid: false` until the GPS and odometry alignment has converged; do not plan before.
- **Evidence horizon**: every ping up to `BeliefGrid.header.stamp` is in the grid. It lags the newest
  ping by one to two image strides.
- **What was searched** comes from Perception's image records when they arrive (exact), or else from
  the timing of the detection arrays (an approximation).

## 5. Settings

- Parameters: `blueboat_belief/config/belief.yaml`, with flat `section.key` names (for example
  `grid.resolution_m: 0.5`, `aspect.sector_count: 12`).
- Override them by passing your own YAML file with `config:=`. An unknown parameter name stops the
  node, on purpose.
- The detection model values (detection probability, false-alarm rate, the correlation between looks,
  prior density) are **placeholders** until they are fitted on a real detector.

## 6. Outputs on disk

Snapshots (`snapshot_*.npz` and `.json`) go to `<root>/data/Belief_data/<run>/`. The root is
`data_dir`, else `$BLUEBOAT_DATA_DIR`, else the workspace (normally `~/ros2_ws`). Nothing is ever
overwritten.

## 7. Where it stands

- The offline test gate passes. Built and checked under ROS 2 Jazzy: the published grid equals the
  offline core bit for bit.
- Run live in Gazebo beside the sonar processor, the Ground Control Station and Perception (dummy
  detector), and on one real harbour recording.
- Run with the Replanner, on an isolated ROS graph with made-up inputs, and once in Gazebo.
- **Not yet**: a real detector, or a real-boat session with Belief running.

## 8. Folder layout

```
blueboat_belief_interfaces/   the message types: BeliefGrid, BeliefContact, QueryBelief
blueboat_belief/              the package
  blueboat_belief/core/       the model: numpy only, no ROS, deterministic
  blueboat_belief/io/         survey-area and mission files, snapshots, data root
  blueboat_belief/ros/        adapters and belief_node.py (the only file that uses ROS)
  blueboat_belief/cli/        replay, make_area
  launch/belief_launch.py
  config/                     belief.yaml, survey_area.example.yaml
  test/                       the test gate, the ROS check, scenarios, reference data
  docs/                       this guide and the design documents
```
