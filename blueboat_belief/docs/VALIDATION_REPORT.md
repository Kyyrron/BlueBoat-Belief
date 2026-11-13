# VALIDATION_REPORT: what was run on the operator laptop, and what it shows

Evidence gathered on 2026-11-12, re-run where stated. Machine: the operator laptop (16 cores, Ubuntu 24.04,
ROS 2 Jazzy, `rmw_fastrtps_cpp`), `/usr/bin/python3` 3.12.3 with numpy 1.26.4, and the project
venv `~/ros2_ws/.venv` with numpy 2.4.4.

Four levels of evidence are kept apart. A statement belongs to the weakest level any of its
parts needs.

| Level | Meaning | What exists |
|---|---|---|
| **A. Offline** | the ROS-free core and adapters, on synthetic inputs and closed forms | §1 |
| **B. ROS, isolated** | the real node under real `rclpy` and DDS, on a ROS domain of its own, fed synthetic messages with the upstream QoS | §2, §3 |
| **C. Real recording** | real field sonar data through the real `sss_processor_node` into Belief | §4 |
| **D. Live simulator** | the full chain in Gazebo: simulated sonar, GCS, Perception, Belief | §5 |

Not at any level: **a real detector** (none is trained; Perception's dummy searches nothing), a
real boat, the Replanner.

---

## 1. Level A — the offline gate

```bash
cd ~/ros2_ws/src/BlueBoat-SideScanSonar/BlueBoat-Belief/blueboat_belief
test/gate.sh            # runs python3 -m test.smoke_test in both environments
```

| Environment | Result |
|---|---|
| A: `~/ros2_ws/.venv/bin/python3`, numpy 2.4.4, rosbags 0.11.9 | **ALL CHECKS PASSED (241 checks, 0 skipped)**, 39.8 s |
| B: `/usr/bin/python3`, numpy 1.26.4 (what ROS nodes import), rosbags taken from the venv, appended last on the path | **ALL CHECKS PASSED (241 checks, 0 skipped)**, 45.8 s |

Last run: 2026-11-12, after the run-health check of `OFFLINE_CORE_API.md` §5 was added to [22],
with the Replanner's working tree of that evening beside it.

**What the gate establishes** (sections of `test/smoke_test.py`):

| Sections | Establishes |
|---|---|
| [0] | the core, io, adapters and CLIs import with ROS, Qt, cv2, rosbags and yaml blocked; rclpy only in `ros/belief_node.py` |
| [1]–[5] | configuration strictness; geodesy bit-identical to MCS; area files; alignment; pose buffer; look geometry |
| [6] | every closed form of `MATH.md` §2–§7, the exact no-op, monotonicity, the order-dependence table |
| [7], [8] | contacts, rendering, disc query |
| [9], [10] | every contract field with its dtype; `.msg` files parsed and CDR round trips |
| [11]–[14] | adapters; determinism; localisation of synthetic targets within 2 m; every rejection reason; fuzzed inputs never raise |
| [15] | 0.50 ms per swath at 0.5 m on this laptop under load (budget 1 ms); `to_grid_dict` 8.5 ms for 240 × 160 cells |
| [16]–[18] | snapshot files; packaging and the node file; an mcap bag replays byte-identically and equals direct feeding |
| [19], [20] | turn gate; survey-area tool including `--from-mission`; run summary |
| [21] | image records: first image credited, `null` images not, dummy flag, auto switch, order of array and record irrelevant, release bound, evidence horizon, a verbatim real record |
| [22] | searched sectors ⟺ `aspect_log_miss < 0`; unknown class counted; a detection released late keeps its pose; the documented offline example runs; the documented run-health keys equal the code's; **the Replanner's own `validate_grid` accepts a real Belief grid** |

**Limits of level A.** Synthetic inputs. No statement about detector performance, calibration
or real sonar data follows from it.

---

## 2. Level B — build

Built into a scratch workspace, with `~/ros2_ws/install` as a read-only underlay for
`blueboat_interfaces`, in a shell that had not sourced the venv:

```bash
ISO=<scratch>; BEL=~/ros2_ws/src/BlueBoat-SideScanSonar/BlueBoat-Belief
env -i HOME=$HOME LANG=C.UTF-8 PATH=/usr/bin:/bin bash --noprofile --norc -c "
  source /opt/ros/jazzy/setup.bash && source ~/ros2_ws/install/setup.bash && cd $ISO &&
  MAKEFLAGS='-j2' nice -n 19 colcon --log-base $ISO/log build --base-paths $BEL \
    --packages-select blueboat_belief_interfaces blueboat_belief \
    --build-base $ISO/build --install-base $ISO/install --executor sequential \
    --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3 -DBUILD_TESTING=OFF"
```

| Check | Result |
|---|---|
| `blueboat_belief_interfaces` (rosidl) | finished, 14.9 s |
| `blueboat_belief` (ament_python) | finished, 1.3 s |
| console scripts | `belief_node`, `blueboat-belief-make-area`, `blueboat-belief-replay`, shebang `#!/usr/bin/python3` |
| files newer than the build under `~/ros2_ws/{build,install,log}` | none |
| build artefacts in the source tree | none |
| `ros2 interface show …/BeliefGrid`, `…/QueryBelief` | every field of `topics.md` §3 and §7, with the contract types |
| `ros2 pkg executables blueboat_belief` | the three executables |
| `ros2 launch blueboat_belief belief_launch.py --show-args` | `area_file`, `config`, `use_sim_time`, `record_inputs`, `data_dir` |

## 3. Level B — the node under real rclpy, isolated

Environment of every shell: `ROS_DOMAIN_ID=42`, `FASTDDS_BUILTIN_TRANSPORTS=UDPv4`,
`ROS_PYTHON_CHECK_FIELDS=1`, own log, tmp and data directories. A canary node on that domain saw
only itself; the number of `fastrtps` shared-memory segments in `/dev/shm` did not change (188
before, during and after); its sockets were on the domain-42 ports.

### 3.1 Refusals

| Start | Result |
|---|---|
| `ros2 run blueboat_belief belief_node` (no area file) | FATAL "survey area file not found…", **exit code 2** |
| `-p coverage.requir_detector:=false` (misspelt) | FATAL "unknown parameters: ['coverage.requir_detector']", exit code 2 |
| `-p pd.plateau:=high` (wrong type) | FATAL "pd.plateau: expected float, got 'high'", exit code 2 |

### 3.2 The runtime check

```bash
source <scratch>/env_iso.sh
cd ~/ros2_ws/src/BlueBoat-SideScanSonar/BlueBoat-Belief/blueboat_belief
/usr/bin/python3 -B -m test.ros_graph_check --scenario straight      # also: lawn; --no-records
```

It starts the installed `belief_node`, publishes a synthetic survey as real messages on the five
input topics with the upstream publishers' QoS, and feeds the same messages, after a
serialisation round trip, to an offline core.

| Run | Messages | Result |
|---|---|---|
| straight pass, with image records | 2299 (1001 pings, 23 arrays, 23 records) | **ALL ROS CHECKS PASSED (36 checks, 0 failed)** |
| lawnmower, with image records | 4603 (2002 pings, 49 arrays, 49 records) | 36 / 36 |
| straight pass, detections only | 2276 | 36 / 36, `coverage.source` = `arrays` |

What the 36 checks cover:

| Group | Checks |
|---|---|
| [R1] | the node runs; one subscription per input topic with the documented reliability; each of our publishers is matched (QoS compatible, CM-4); `belief_grid` offered RELIABLE + TRANSIENT_LOCAL; every output topic and the service with their types; **diagnostics published before any input** (steady-clock timer) |
| [R2] | every `stats.*` and `rejected.*` value on `/diagnostics` equals the offline core's; no zero-stamp fallback was needed; level OK; coverage source and observed calibration as fed |
| [R3] | a subscriber created after the run received the last grid **35 ms** after subscribing; `grid_seq` advanced by exactly 1; **all 33 non-contact fields and all 21 fields of every contact equal the offline core's, bit for bit**; `model_params_json`; both display grids; the markers |
| [R4] | `QueryBelief` around a target equals the core's answer field by field; an invalid radius is answered `ok = false` |
| [R5] | SIGINT → exit code 0, no traceback; `snapshot_final.npz` **byte-identical** to the offline core's; manifest counters equal |

### 3.3 Launch file, input recording, replay

```bash
ros2 launch blueboat_belief belief_launch.py area_file:=<W>/area.yaml config:=<W>/params.yaml \
     use_sim_time:=false record_inputs:=true data_dir:=<W>
/usr/bin/python3 -B -m test.ros_graph_check --attach --scenario lawn --work <W>      # second shell
# SIGINT to the launch, then twice:
~/ros2_ws/.venv/bin/python3 -B -m blueboat_belief.cli.replay <W>/data/Belief_data/<stamp>-inputs \
     --area <W>/area.yaml --config <W>/params.yaml --out <W>/replayN
```

| Check | Result |
|---|---|
| checker against the launched node | 33 / 33 (the shutdown group is skipped with `--attach`) |
| launch on SIGINT | exit 0; `belief_node` "finished cleanly"; `snapshot_final.npz` and `.json` written |
| recorded bag | `<stamp>-inputs_0.mcap` + `metadata.yaml`; the replay read **4603 messages**, all that were published |
| replay 1 against replay 2 | **sha256 identical** |
| the live node's final snapshot against the replay | **33 of 33 members identical** (all but `grid_seq`, which the replay sets to 1) |

### 3.4 Shutdown and parameter behaviour under a real launch

Checked by [R5], §3.1 and §3.3: SIGINT and SIGTERM give exit code 0 and write
`snapshot_final.*`; a second SIGINT during shutdown (as `ros2 launch` sends it) is ignored; an
integer given for a float parameter is accepted; a misspelt parameter or a refused area file
stops the node with code 2; the grid, displays and `/diagnostics` are published without
`/clock`; `grid_seq` advances only when the grid is published.

**Observed, not a Belief defect.** Under `ros2 launch`, `ros2 bag record` exits with code 2 on a
double SIGINT ("process has died"). The bag is complete: it has its `metadata.yaml` and replays.

**Limits of level B.** The publishers are synthetic. It shows that the node, the generated
messages, QoS, parameters, timers, the service, shutdown and recording behave as specified. It
shows nothing about the upstream nodes' real output.

---

## 4. Level C — a real recording

Run on an isolated ROS domain.

**Recording.** `~/ros2_ws/data/SSS_data/sessions/harbourval_real_leg1/toOptimize-harbourScan_leg1_18p4m.svlog`:
one leg of a real harbour survey, 1961 pings per side, range 18.4 m. Read through a symbolic
link; nothing was written into the corpus.

**Chain** (every tool unmodified):
1. `BlueBoat-SSS/blueboat_sss/blueboat_gcs/tools/svlog_to_rosbag.py` → a raw bag: both sonar
   profiles, the mavros topics, `/mavros/global_position/global` (2973) and a synthesised
   `/blueboat/odom` (5945). Its input and output paths are hard-coded, so it was run in a
   scratch directory that mirrors them.
2. `ros2 launch blueboat_sss SSS_processing_launch.py will_use_rosbag:=True` (the real
   `sss_processor_node`), `ros2 bag play` of the raw bag at rate 1, and
   `ros2 bag record -s mcap --topics /sss_processor/processed /blueboat/odom /mavros/global_position/global`.
3. `blueboat-belief-make-area --from-world ~/worlds/Shiraishijima_real_toOptimize/harbour_wl1p50`
   (the world built on that harbour: its anchor and limits), then the replay, with the venv python.

| Check | Result |
|---|---|
| processed rows recorded | **1961**: one per ping pair of the recording, none dropped |
| bag | 10 879 messages over 149.8 s (1961 pings, 5945 odom, 2973 fixes) |
| bottom tracker | locked at 4.93 m; no row with the "altitude unknown" sentinel |
| replay, gated (default) | 1961 pings applied, all geometry-only (no detector ran on this recording); `coverage_expired` 1175, the rest released at the end |
| replay, **ungated** (`--no-require-detector`, labelled) | 1961 pings applied and credited |
| alignment from the **real GPS** against the synthesised odometry | valid; `odom_to_map` = (−12.147, 53.860) m; spread **0.004 m** |
| `rejected.*` | `alignment_pending` 7 (queued until the fit was valid, then applied), `no_pose_at_stamp` 1 (one fix before any odometry); no `frozen_pose`, no `altitude_unknown`, no `nan_input` |
| grid | 320 × 460 cells; 9081 cells seen |
| ungated replay twice | sha256 identical; 0.9 s for the whole bag |

**What the ungated replay shows about the model on real motion** (not a search result: no
detector, placeholder PD):

| Seen cells with coverage | Share |
|---|---|
| below 0.70 (the range ramps) | 14.4 % |
| exactly 0.70 (one look) | 73.0 % |
| above 0.70 (**credited as more than one look, from one leg**) | **12.7 %**, up to 0.931 |

1. **One real leg is credited as several looks where the heading changed.** 962 cells were seen
   from two sectors, 253 from three, 20 from four. A look bearing that crosses a 30° sector
   boundary opens a new look with ρ ≥ 1/3, however small the actual change of bearing. This is
   limitation L13 (`DESIGN.md`) seen on real data: with sector-centre ρ, a few degrees across a
   boundary count as 30°. The bearing-resolved ρ of `TODO.md` would remove that step.
2. **The turn gate removed part of 464 of the 1961 pings** (354 644 samples). See the caveat.

**Caveats, stated because they limit what this shows.**
- The converter stamps messages on a synthetic clock, and the bag lasts 149.8 s where the
  session's metadata gives 98 s. Positions and bearings are unaffected. Everything that depends
  on rates is: the yaw rate and speed the turn gate estimates, its 1 s window, the 20 s look
  timeout. The turn-gate count above is therefore **qualitative**. `r* = v/|ω|` itself is a
  ratio and does not depend on the time scale, so the gate did act on real motion, but how
  often it should is not settled by this replay.
- `/blueboat/odom` is synthesised by the converter from the autopilot's attitude and local
  position, not recorded from `robot_interface`. L-1, L-2 and L-8 (yaw convention, zero stamps,
  frozen poses on a real launch) are **not** covered.
- No detections exist for this recording.

**Observed, not a Belief matter.** On SIGINT under `ros2 launch`, `sss_processor_node.py` ended
with a traceback and exit code −2, after it had published every row. It looks like the same
double-interrupt pattern that Belief's node now ignores (§3.4). For the owner / SSS.

## 5. Level D — the live simulator

Run when no other simulation was running.

### 5.1 Shared build

```bash
env -i HOME=$HOME LANG=C.UTF-8 PATH=/usr/bin:/bin bash --noprofile --norc -c "
  source /opt/ros/jazzy/setup.bash && cd ~/ros2_ws &&
  colcon build --packages-select blueboat_belief_interfaces blueboat_belief \
      --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3"
```

Both packages finished (8.5 s). `~/ros2_ws/install` now holds `blueboat_belief` and
`blueboat_belief_interfaces`; the node's shebang is `#!/usr/bin/python3`; the chain in
`install/setup.bash` is unchanged (`/opt/ros/jazzy` only).

### 5.2 Run a: default configuration, Perception's dummy detector

```bash
cd ~/ros2_ws/src/BlueBoat-SideScanSonar/BlueBoat-Belief/blueboat_belief
/usr/bin/python3 -B test/live_sim_check.py --world ~/worlds/sea_validation/flat_5p0m \
    --trajectory sea_validation --out ~/ros2_ws/sim_runs/belief_runs/b3a_sea_validation --sensor geometric
```

The script starts `belief_launch.py` (`use_sim_time:=true record_inputs:=true`), then the
superproject's `tools/sim_record/record_run.py` unmodified (MCS launcher, Gazebo, the simulated
sonar, `sss_processor`, the GCS offscreen, Perception). World: 115 × 64 m, flat, 5 m deep; path:
two 70 m legs 12 m apart joined by a half turn, 0.8 m/s. Everything is in the run folder.

| Check | Result |
|---|---|
| `record_run.py` | exit 0, `status: completed`, 250 s wall, `left_running: []` |
| GCS | 4365 pings, 73 seabed images |
| **pings applied by Belief** | **4365**: every ping the GCS recorded |
| image records / detection arrays received | 73 / 73, `records_without_array` 0, `records_overlap` 0 |
| `coverage_source` | `records` (the optional input worked against the real Perception node) |
| records searched | 0 of 73: all from `dummy_center`, `detector_calibrated_observed: false` |
| pings covered / geometry-only | 0 / 4365; `area_cumulative_pd` 0.0. **Correct**: no detector searched anything |
| `rejected.*` | `ignored_class` 73 (the dummy's one detection per image); every other counter 0, `frozen_pose` included |
| alignment | valid after 10 pairs, source `fit`; `odom_to_map` = (3e-10, 3e-21) m, σ = 7e-10 m: odom (0, 0) is the world anchor |
| zero stamps | 0 (sim time everywhere) |
| `/diagnostics` during the run | level OK ("running") in all 8 samples |
| latency (`latency.event_age_s`) | 3.2 to 5.9 s while pings flowed (one to two image strides at 0.8 m/s is 2.7 to 5.3 s, plus the pipeline) |
| node load | 16 % of one core, 110 MB resident |
| turn gate | `backscan_pings` 0 on the first leg, 84 → 549 through the half turn, 754 at the end, 463 302 samples dropped. See the observation below |
| inputs bag | 10 277 messages over 240 s: 4365 pings, 4805 odom, 961 fixes, 73 arrays, 73 records |
| replay of that bag, twice | sha256 identical |
| live final snapshot against the replay | all members identical (except `grid_seq`) |

**QoS on the live graph**, from `ros2 topic info -v` 60 s into the run:

| Topic | Publisher | Belief's subscription |
|---|---|---|
| `/sss_processor/processed` | `sss_processor`, BEST_EFFORT | one, BEST_EFFORT |
| `/sss_ai/detections` | `perception`, RELIABLE | one, RELIABLE |
| `/sss_ai/image_records` | `perception`, RELIABLE | one, RELIABLE |
| `/blueboat/odom` | `gz_bridge`, RELIABLE | one, BEST_EFFORT |
| `/mavros/global_position/global` | `mavros_shim`, BEST_EFFORT | one, BEST_EFFORT |
| `/belief_map/belief_grid` | `belief_map`, RELIABLE + TRANSIENT_LOCAL | — |

**Ungated replay of the same bag** (`--no-require-detector`, a labelled development mode: every
ping counted as searched). `area_cumulative_pd` = 0.452 with the PD placeholder; no contact
(the only detections are the dummy's). This shows the swath geometry, not a search result.

**Observations.**
1. **The turn gate also acts after the turn.** `backscan_pings` was 549 at ping 2433 and kept
   rising on the return leg: 617, 664, 703, 741 at pings 2877, 3321, 3820, 4209, and 754 at the
   end. So about 200 of the last 1900 pings (one in ten) had a back-scanned part: the yaw rate
   exceeded `v / range` = 0.04 rad/s while the boat held the line. That is the model as
   specified (Perception flags the same rows), but it means a share of straight-line swath is
   not credited. Whether that share is right needs real yaw-rate data (`TODO.md`).
2. `record_run.py` counted `behind_sim_clock: 4` in its launch log. Its own reading rule wants 0
   for a sonar-comparison session. This run was a Belief integration run with an extra node and
   a recorder attached; it must not be used as a sonar dataset.
3. Live, the node received 4794 odometry messages where the bag holds 4805 (a BEST_EFFORT
   subscription to a RELIABLE publisher). The belief is identical: odometry is used only for the
   alignment fit, which had latched.
4. The last 156 pings were released at shutdown: no image came after them. This is the tail
   described in `MATH.md` §8.

**Limits of this run.** The detector is the dummy. The run shows that Belief's plumbing holds
against the real upstream nodes: stamps, QoS, alignment, records, turn gate, recording, replay.
It shows nothing about detections, contacts or negative evidence.

### 5.3 Run b: the simulator's fault-injection profile

```bash
/usr/bin/python3 -B test/live_sim_check.py --world ~/worlds/sea_validation/flat_5p0m \
    --trajectory sea_validation --out ~/ros2_ws/sim_runs/belief_runs/b4_fault_injection --sensor geometric \
    --extra sonar_config:=$HOME/ros2_ws/install/blueboat_sss_sim/share/blueboat_sss_sim/config/fault_injection_sonar.yaml
```

The profile puts on the wire what a clean simulation never does (root `CLAUDE.md` CM-2):
`dropped_ping_prob: 0.02` per side, a gain step every 250 pings, a counter restart every 4000
pings, a counter offset of 60, `channel_number: 255`.

| Check | Run a (no faults) | Run b (faults) |
|---|---|---|
| `record_run.py` | completed | completed, exit 0, 250 s |
| rows on `/sss_processor/processed` | 4365 | 4347 |
| one-sided rows among them | 0 | **172** (86 port only, 86 starboard only) |
| ping-number restarts | 0 | **6** |
| **pings applied by Belief** | 4365 | **4347**: every row, one-sided ones included |
| `rejected.*` | `ignored_class` 73 | `ignored_class` 119; nothing else |
| exceptions, node exit | none, 0 | none, 0 |
| alignment | valid, 3e-10 m | valid, 9e-11 m |
| seabed images / records | 73 | 119 |
| replay twice identical; live = replay | yes; yes | yes; yes |

Belief does not key anything on the ping counter except "0 = this side is absent", so restarts
and offsets do not reach it; a one-sided row updates the side that is present (CM-6: no ping is
dropped). The gain walk reaches only `last_intensity_db`, which is display-only.

**Limit.** The dummy detector again: the faults were exercised on the ping path only.

---

## 6. What remains unvalidated

| Item | Why | Needs |
|---|---|---|
| Detections, contacts and negative evidence on real or simulated sonar images | no trained detector exists | Perception: weights, then calibration |
| `p_exists` as a probability | scores are raw | calibration and its base rate |
| PD, PFA, ρ | placeholders and an assumed rule | labelled runs; E3 |
| Belief on the real boat | no field session with Belief running | a field day with `record_inputs:=true`; real-boat checks L-1, L-2, L-4, L-8 (`TODO.md`) |
| Belief with the Replanner under ROS | the Replanner drives Belief's offline core in its dev loop (`OFFLINE_CORE_API.md`); its node ran beside `belief_map` once, on an isolated ROS graph with made-up sonar and Control (2026-11-12), never in Gazebo | a Gazebo mission with the Replanner; SSS-Sim `path_source` (Control R-3) for a built world |
