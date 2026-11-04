# LOCAL_CHECKLIST: what the cloud session could not run

This file is the input of the in-machine session `BELIEF-2-LOCAL`. Each item names:
- what to check;
- why it matters to Belief;
- the exact commands (from `~/ros2_ws` with `source env.sh`, unless stated);
- what counts as a pass.

The file grows phase by phase. Phase 1 (rev. b) holds the runtime facts behind the design. Later
phases add the build, node, live-simulator, fault-injection and real-replay steps.

Status: every item is **[NOT RUN — needs in-machine session]**.

## Phase 1: runtime facts

### L-1 Real `/blueboat/odom` yaw is absolute ENU
- **Why.** Absolute yaw makes odom → map translation-only (`odom_to_map_yaw = 0`). The code says
  absolute (`BlueBoat-Control/.../robot_interface.py:627-639`).
- **How.** Use a v1.1 real-run bag; it holds `/blueboat/odom` and `/mavros/global_position/compass_hdg`.
  ```bash
  ros2 bag info ~/ros2_ws/data/Robot_data/<run>/<run>-bag
  ```
  Then compare `yaw(odom)` with `radians(90 − hdg)` (wrapped) over the run, using `rosbags`.
- **Pass.** The median |Δ| is a few degrees and does **not** equal the launch heading.

### L-2 On the real launch: zero stamps and zero covariance
- **Why.**
  - Zero stamps: Belief must key these messages on arrival time, so the node runs on wall time on
    the real graph (mavros runs `use_sim_time` with no `/clock`; `BlueBoat_launch.py:3`).
  - Zero covariance means falling back to the configured sigmas.
- **How.** During a real session, or on a v1.1 bag:
  ```bash
  ros2 topic echo --once /blueboat/odom --field header.stamp
  ros2 topic echo --once /blueboat/odom --field pose.covariance
  ros2 topic echo --once /mavros/global_position/global --qos-reliability best_effort --field header.stamp
  ros2 topic echo --once /mavros/global_position/global --qos-reliability best_effort --field status
  ```
- **Pass.** The facts are recorded: stamp 0 or not, covariance zero or not, and the `status` values seen (0 / −1).

### L-3 QoS of Belief's four inputs and its output on the live graph
- **Why.** CM-4: QoS must match at both ends. A wrong reliability or durability setting receives
  nothing, and gives no error.
- **How.** During a built-world run (`full_mission_launch.py`) with Perception and the GCS processor
  running, and again with Belief running:
  ```bash
  for t in /sss_processor/processed /sss_ai/detections /blueboat/odom /mavros/global_position/global; do ros2 topic info -v $t; done
  ```
  And, once Belief runs: `ros2 topic info -v /belief_map/belief_grid`.
- **Pass.** Expected publishers:
  - `/sss_processor/processed`: BEST_EFFORT / VOLATILE.
  - `/sss_ai/detections`: RELIABLE / VOLATILE.
  - `/blueboat/odom`: RELIABLE / VOLATILE.
  - `/mavros/global_position/global`: BEST_EFFORT.

  Belief's subscriptions must be compatible with each, and Belief's own subscription count must
  be 1 on each.

### L-4 Compass against odom yaw offset (detections against pings)
- **Why.** Detection positions use the GCS compass heading; Belief's ping geometry uses odom yaw.
  An offset between the two is a systematic shift between positive and negative evidence (memo S-14).
- **How.** On a real run, record `/blueboat/odom` and `/mavros/global_position/compass_hdg` and
  compute the yaw difference. On a built world it should be about 0.
- **Pass.** The offset distribution is recorded. If it exceeds ~2°, it is added to Belief's
  covariance or handled in Phase 3.

### L-5 Built world: odom (0, 0) = the world `geo_anchor`, and the GPS has no noise
- **Why.** In simulation it is the exact cross-check of the online odom → map fit (decision D-3).
- **How.** During `ros2 launch blueboat_sss_sim full_mission_launch.py world_dir:=<world> with_control:=true trajectory_file:=<deployed yaml>`:
  ```bash
  ros2 topic echo --once /mavros/global_position/global --qos-reliability best_effort
  ros2 topic echo --once /blueboat/odom --field pose.pose.position
  grep -A3 geo_anchor <world>/metadata.yaml
  ```
- **Pass.** The fix at the spawn equals `geo_anchor` to float precision.

### L-6 Real-recording replay chain for Belief
- **Why.**
  - Control's run bag excludes `/sss_processor/*` and `/sss_ai/*` (`run_bag.py:52`).
  - The GCS "Save as rosbag" writes no `/sss_processor/processed`.

  Belief's four topics therefore have to be produced and recorded explicitly.
- **How (corpus `.svlog`).**
  1. In the GCS replay window, use "Save as rosbag" (mcap).
  2. Play the bag: `ros2 bag play <bag>`.
  3. Regenerate the processed pings: `ros2 launch blueboat_sss SSS_processing_launch.py will_use_rosbag:=True`.
  4. Record: `ros2 bag record -s mcap /sss_processor/processed /blueboat/odom /mavros/global_position/global -o <out>`.

  Real runs: record the same topics plus `/sss_ai/detections` beside the session.
- **Pass.** A bag with `/sss_processor/processed` at about the ping rate, which Belief's offline
  replay tool reads with `rosbags`.

### L-7 Perception's detection-array cadence and empty arrays
- **Why.** Empty arrays and `no_pixels` arrays look the same on the wire, and an array carries only
  its newest row time. Belief may use the arrays as a detector-coverage signal (memo S-3).
- **How.** In a built-world run with the dummy detector:
  ```bash
  ros2 topic hz /sss_ai/detections
  ros2 topic echo /sss_ai/detections --field header.stamp
  ```
  Compare the stamps with the record log `~/ros2_ws/data/Perception_data/live/<stream_id>.jsonl`
  (`image_id`, `time.t_min_s/t_max_s`, `processing.detector.status`).
- **Pass.** The interval between arrays and the overlap with record spans are measured, and the
  stamp-0 arrays are identified.

### L-8 Frozen (0, 0) ping poses on the real boat
- **Why.** The brief §2 and §7 frozen-pose rule: Belief must report ERROR and not map onto (0, 0).
- **How.** On any real session's `/sss_processor/processed`, count rows with `robot_x == robot_y == 0`:
  ```bash
  ros2 topic echo /sss_processor/processed --field robot_x
  ```
  Also look for `ALIGNMENT: ProcessedSSSPing poses are frozen` in the GCS console.
- **Pass.** The count is recorded. Belief's rejection counter `frozen_pose` matches it in Phase 4.

## Phases 3–4: build, run and live validation (written 2026-11-04, none run)

Order: B-1 → B-2 → B-3 → B-4 → B-5 → B-6 → B-7. Run every command from `~/ros2_ws` after
`source /opt/ros/jazzy/setup.bash`, unless stated.

### B-1 Build and interface check
```bash
sudo apt install ros-jazzy-vision-msgs        # if absent (Perception needs it too)
colcon build --packages-up-to blueboat_belief
source install/setup.bash
ros2 interface show blueboat_belief_interfaces/msg/BeliefGrid
ros2 interface show blueboat_belief_interfaces/srv/QueryBelief
cd src/BlueBoat-SideScanSonar/BlueBoat-Belief/blueboat_belief && /usr/bin/python3 -m test.smoke_test   # needs: pip install rosbags
```
- **Pass:** the build is clean; the fields shown equal `docs/topics.md` §3; smoke reports
  ALL CHECKS PASSED (188 here).

### B-2 Node start, parameters, QoS
```bash
ros2 run blueboat_belief belief_node          # without area.file: expect a FATAL naming the missing file, exit 2
ros2 launch blueboat_belief belief_launch.py area_file:=$HOME/worlds/<path>/<world> use_sim_time:=true
ros2 node info /belief_map
for t in /belief_map/belief_grid /belief_map/p_target /belief_map/coverage; do ros2 topic info -v $t; done
ros2 param get /belief_map detector.calibrated
```
- **Pass:**
  - four subscriptions and the outputs of `topics.md` §1 are listed;
  - `belief_grid` is offered RELIABLE + TRANSIENT_LOCAL;
  - `detector.calibrated` is False;
  - the WARN line about calibration is printed;
  - the message fill path works: rclpy accepts every array, which is v1 bug 4 checked live.

### B-3 First live simulator run (built world)
Start each line in its own terminal:
```bash
ros2 launch blueboat_sss_sim full_mission_launch.py world_dir:=$HOME/worlds/<path>/<world> with_control:=true controller_type:=PID trajectory_file:=$HOME/.config/blueboat_mcs/trajectories/.deployed/<name>.yaml
ros2 launch blueboat_perception perception_launch.py world_dir:=$HOME/worlds/<path>/<world>
cd ~/ros2_ws/src/BlueBoat-SideScanSonar/BlueBoat-SSS/blueboat_sss && python -m blueboat_gcs.main   # Sonar: Simulation, START
ros2 launch blueboat_belief belief_launch.py area_file:=$HOME/worlds/<path>/<world> use_sim_time:=true record_inputs:=true
```
Then check:
```bash
ros2 topic echo --once /belief_map/belief_grid --qos-durability transient_local --qos-reliability reliable --field odom_to_map_valid
ros2 topic echo --once /diagnostics
```
- **Run a: default config (the dummy detector):**
  - `odom_to_map_valid` is true within ~10 s of the first fix;
  - no sim cross-check WARN (|t| < 0.5 m);
  - `rejected.ignored_class` grows by about one per image;
  - `stats.pings_geometry_only` grows;
  - `coverage` stays 0. **This is correct**: the dummy is not a detector (owner Q-9).
- **Turn gate (both runs):**
  - `stats.backscan_pings` / `backscan_samples` rise only while the boat turns, i.e. lawnmower
    ends;
  - in RViz no `/belief_map/coverage` looks appear beyond the turn centre on the inner side;
  - note how much of each turn's first ~1 s (estimator window) escapes the gate.
- **Run b: `coverage.require_detector:=false` (labelled development run):**
  - `coverage` reaches 0.7 along the swath (placeholder plateau) with the nadir and > 25 m at 0;
  - `n_looks` / `aspect_counts` match the track in RViz (`/belief_map/coverage` Map display).
- **Then, offline (determinism on a real recording):**
  ```bash
  blueboat-belief-replay ~/ros2_ws/data/Belief_data/<stamp>-inputs --area $HOME/worlds/<path>/<world>
  ```
  Run it twice: `snapshot_final.npz` must be byte-identical (`sha256sum`).
- **With a trained YOLO model** (`detector:=yolo detector_model:=…` in Perception), additionally:
  - contacts appear at the world's objects; compare offline with `metadata.yaml` `objects` (never
    online);
  - localisation ≤ 2 m from one pass (T12).

### B-4 CM-2 check against the fault-injection profile
```bash
ros2 launch blueboat_sss_sim full_mission_launch.py world_dir:=<world> with_control:=true trajectory_file:=<yaml> sonar_config:=$(ros2 pkg prefix blueboat_sss_sim)/share/blueboat_sss_sim/config/fault_injection_sonar.yaml
```
Turn on the knobs as documented in that file.
- **Pass:** Belief keeps running; one-sided rows are handled; `rejected.*` counters move and no
  exception occurs.

### B-5 Performance on the laptop
```bash
cd ~/ros2_ws/src/BlueBoat-SideScanSonar/BlueBoat-Belief/blueboat_belief && /usr/bin/python3 -m test.smoke_test | grep -E "us/swath|snapshot encode"
ros2 topic hz /belief_map/belief_grid
```
- **Pass:** < 1 ms per swath; `belief_grid` at about 1 Hz during a run. Record the numbers in
  `.claude/CLAUDE.md` §6.

### B-6 Real-recording replay (negative evidence, coverage, performance; no real detections exist)
1. Build the bag: L-6 chain (GCS "Save as rosbag" → `ros2 bag play` + `SSS_processing_launch.py
   will_use_rosbag:=True` → `ros2 bag record` of the four inputs).
2. Write a survey-area file around the recording:
   `blueboat-belief-make-area --out <area.yaml> --anchor <lat> <lon> --local <xmin> <ymin> <xmax> <ymax>`.
3. Replay:
   ```bash
   blueboat-belief-replay <recorded bag> --area <survey_area.yaml> --no-require-detector
   ```
- **Pass:**
  - `odom_to_map_valid` true;
  - coverage follows the track;
  - `rejected.altitude_unknown` and `frozen_pose` match the recording's known pathologies (L-8);
  - runtime is recorded.

### B-7 Reconcile with the Replanner
- Diff `docs/CONTRACT_PROVIDED.md` against BlueBoat-Replanner's `docs/CONTRACT_CONSUMED.md`.
- Settle `CONTRACT_DEVIATIONS.md` D-C1 … D-C6.
- If the Replanner's emulator feeds Belief, check that it publishes one array per image, empty
  ones included, and never runs beside Perception.
