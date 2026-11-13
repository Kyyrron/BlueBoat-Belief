# BlueBoat-Belief: interface contract (`belief_grid/0`, producer side)

This page is the contract a consumer relies on: topics, QoS, frames, every field, what each
field answers, and where Belief departs from the `belief_grid/0` text (§10). Changes are
additive only (CM-1). The model behind the numbers is `MATH.md`; driving the core without ROS
is `OFFLINE_CORE_API.md`; what has been run is `VALIDATION_REPORT.md`.

> **Priority: detector calibration.**
> - Perception's calibration is the identity today, and `/sss_ai/detections` carries no
>   calibration flag.
> - Belief uses the raw score `q` as configured (`detector.calibrated: false`) and publishes
>   that flag in `model_params_json`.
> - **Until a calibrated detector exists, `p_exists`, `p_target` and every contact log-odds are
>   score-derived indices, not probabilities.**

Someone who has never seen a consumer must be able to reproduce any prediction from this page.

## 1. Node and topics

The node is `belief_map` (package `blueboat_belief`, executable `belief_node`). Launch it with
`ros2 launch blueboat_belief belief_launch.py area_file:=… [use_sim_time:=true] [record_inputs:=true]`.

### Inputs

The first four are the primary inputs (owner decision D-2). The fifth is an optional companion
(D-10): Belief works without it and uses it only to know which pings the detector searched. A
subscription QoS that does not match
the publisher's receives nothing (CM-4); the subscription QoS below is compatible with every
publisher.

| Topic | Type | Subscription QoS | Publishers |
|---|---|---|---|
| `/sss_processor/processed` | `blueboat_interfaces/ProcessedSSSPing` | BEST_EFFORT, KEEP_LAST 50 | `sss_processor_node` (BEST_EFFORT 10) |
| `/sss_ai/detections` | `vision_msgs/Detection2DArray` | RELIABLE, KEEP_LAST 50 | Perception (RELIABLE 10) |
| `/blueboat/odom` | `nav_msgs/Odometry` | BEST_EFFORT, KEEP_LAST 50 | `robot_interface` / Gazebo bridge / `bag_replay` (RELIABLE, VOLATILE) |
| `/mavros/global_position/global` | `sensor_msgs/NavSatFix` | BEST_EFFORT, KEEP_LAST 10 | mavros, SSS-Sim shim, MCS SimGps (BEST_EFFORT), `bag_replay` (RELIABLE) |
| `/sss_ai/image_records` *(optional)* | `std_msgs/String`, JSON `blueboat_perception/image_record/1` | RELIABLE, KEEP_LAST 100, VOLATILE | Perception (same QoS) |

From an image record Belief reads only the image's row span, whether the detector searched it,
the dummy flag and the calibration flag (`MATH.md` §8.2). Detections are never taken from it.

**Mission geo file**, read once at start (`area.file`):
- a simulator world folder or its `metadata.yaml` (`blueboat_world_meta/1`);
- or a `blueboat_survey_area/1` file (`SURVEY_AREA_FORMAT.md`).

Belief never reads simulation ground truth: not `ground_truth/*`, not `/sim/*`, not the world's
`objects`.

### Outputs

| Name | Type | QoS | Rate |
|---|---|---|---|
| `/belief_map/belief_grid` | `blueboat_belief_interfaces/BeliefGrid` | RELIABLE, **TRANSIENT_LOCAL**, KEEP_LAST 1 | `publish.grid_rate_hz` (1 Hz, **wall clock**); every publish is `grid_seq + 1` |
| `/belief_map/query_belief` | `blueboat_belief_interfaces/QueryBelief` (srv) | — | on request |
| `/belief_map/p_target` | `nav_msgs/OccupancyGrid` (display) | RELIABLE, TRANSIENT_LOCAL, 1 | `publish.display_rate_hz` (0.5 Hz) |
| `/belief_map/coverage` | `nav_msgs/OccupancyGrid` (display) | same | same |
| `/belief_map/contacts` | `visualization_msgs/MarkerArray` | default, 10 | same |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | default, 10 | 1 Hz |

**Subscribing to `belief_grid`.** Use RELIABLE + TRANSIENT_LOCAL + depth 1 to get the last
snapshot on joining; RELIABLE VOLATILE also connects.

**Pacing.** The grid, the displays and `/diagnostics` are paced by a steady (wall) clock, also
under `use_sim_time`. A simulator that is not started, or is paused while a ping renders, has no
`/clock`; node-clock timers would then publish nothing. Periodic snapshot files stay on the node
clock: one per `snapshot.period_s` of mission time.

**`grid_seq`** starts at 1 when the node starts and advances by exactly 1 per published grid.
A restarted node starts again at 1: a consumer that drops "old" sequence numbers must treat a
decrease as a restart, not as a stale message.

**Exit codes of `belief_node`.** 0 after SIGINT or SIGTERM (the final snapshot is written; a
second interrupt during shutdown is ignored). 2 when the configuration or the survey-area file
is refused: a missing area file, an unknown parameter name, a wrong parameter type.

**Display encoding.**
- OccupancyGrid `coverage` = round(100·coverage).
- OccupancyGrid `p_target` = round(100·(log10 p − log10 1e-5)/5), i.e. 0 at 1e-5 and 100 at 1.
  A log scale, because `p_target` is ≈ 1e-4.
- −1 outside the area or for a cell with no look.

## 2. Clocks and frames

- **Time.** Every time is the **ROS time stamped on the sonar profiles**: wall time on the real
  boat, sim time under Gazebo.
  - `header.stamp` of `BeliefGrid` is the newest processed event time. It is the **evidence
    horizon**: every ping at or before it has been applied to the grid, and none after it.
  - **Latency.** The horizon lags the newest ping by one to two seabed-image strides plus the
    detector's processing time, because a ping is credited only once every image containing it
    has been reported (`MATH.md` §8). An image stride is 128 rows of 16.7 mm, i.e. 2.1 m of
    track: 4 to 9 s at 0.5 m/s, 2 to 4 s at 1 m/s. `/diagnostics` `latency.event_age_s` shows
    the lag live. A consumer should use the stamp, not a fixed delay.
  - A **zero header stamp** (the real launch: mavros on `use_sim_time` without `/clock`, so the
    odom and fixes) is replaced by the node clock at arrival. Run with `use_sim_time:=false` on
    the real boat and `true` under Gazebo.
  - Offline replays use the bag's log time instead.
- **Map frame.** ENU metres about the **datum** = the survey area's `geo_anchor`.
  - `x_map = R(odom_to_map_yaw)·x_odom + (odom_to_map_x, odom_to_map_y)`.
  - `odom_to_map_yaw` is 0: real odom yaw is absolute ENU.
  - The translation comes from an online median fit of GPS-fix ↔ odom pairs, latched once
    valid, or from a config override (`alignment.origin_lat/lon`).
  - `odom_to_map_valid` is **false until then: do not plan on it**.
- **Geodesy.** `east = radians(lon − lon0)·R·cos(radians(lat0))`,
  `north = radians(lat − lat0)·R`, `R = 6378137.0`. This is bit-identical to
  `BlueBoat-MCS/mcs/core/geo.py`, and `test/smoke_test.py [1]` checks it.

## 3. `BeliefGrid` layout

- `width`, `height` and `resolution` (m) give the grid; `origin_x`, `origin_y` are the map-frame
  lower-left corner of cell (0, 0).
- Cell index `i = row·width + col`. Row 0 is the south edge and column 0 the west edge. The centre
  of cell i is `(origin_x + (col + ½)·resolution, origin_y + (row + ½)·resolution)`.
- Per-sector arrays (`aspect_counts`, `aspect_log_miss`) have `width·height·S` entries, index
  `i·S + s`, with S = `aspect_sector_count`.
- The survey polygon is `area_x`, `area_y` (map frame, counter-clockwise, not closed). It is an
  addition to `belief_grid/0`. A cell is *inside* when its centre is inside the polygon.

| Channel | dtype | Meaning | Outside the area | Inside, never observed |
|---|---|---|---|---|
| `p_background` | float32 | P(an **undetected** target in the cell); lowered only by misses | NaN | prior `p0` |
| `p_target` | float32 | `1 − (1 − p_bg)·∏_c(1 − p_c·w_c)` (MATH §11) | NaN | `p0` |
| `coverage` | float32 | `1 − Q`, the modelled cumulative detection probability (not geometric swath coverage) | NaN | 0 |
| `n_looks` | uint16 | independent looks (MATH §5); alias of synthesis `n_observations` | 0 | 0 |
| `n_pings` | uint32 | raw pings | 0 | 0 |
| `n_aspects` | uint8 | sectors with ≥ 1 look | 0 | 0 |
| `depth_m` | float32 | `water_depth` of the last ping observing the cell (flat floor) | NaN | NaN |
| `last_intensity_db` | float32 | mean sample dB in the cell of that ping (device scale, no gain compensation) | NaN | NaN |
| `max_confidence` | float32 | best associated detection score | NaN | 0 |
| `aspect_counts` | uint8 | looks per sector, saturating at 255 (synthesis `aspect_coverage`) | 0 | 0 |
| `aspect_log_miss` | float32 | `ln(1 − max covered PD in sector s)` (Paull α = 0 rule) | NaN | 0 |

A consumer treats NaN as "no valid belief data", never as 0.

### 3a. Which field answers which question (checked in smoke [22])

| A consumer asks | Read | Not |
|---|---|---|
| Is this cell inside the survey area? | `coverage` is not NaN | — |
| Which sectors of this cell has the **detector searched**? (the set ρ is taken against) | `aspect_log_miss[i·S + s] < 0` | `aspect_counts > 0`, which also counts looks that had no detector coverage. With the dummy detector, or none, every look is counted and no sector is searched |
| How well has this cell been searched overall? | `coverage` (= 1 − Q) | `n_looks`, `n_pings` (geometry only) |
| How much is a new look from sector k worth here? | ρ from §4 item 5 against the searched sectors, then §4 item 6 with `coverage` | a count of distinct sectors |
| Up to which time is the grid complete? | `header.stamp` (§2, evidence horizon) | the arrival time, or a fixed delay |
| Are the scores probabilities? | `model_params_json.detector_calibrated`; what Perception itself reports is in `detector_calibrated_observed` | assuming so. Today both are false |
| Is the map usable for planning? | `odom_to_map_valid` | `odom_to_map_sigma_m` alone |
| May coverage be compared between two survey patterns? | No: it depends on the order of three or more looks (`MATH.md` §7). Two-pass orthogonal is order-independent | — |
| What class is a contact? | `class_probabilities`, aligned with `class_names` | assuming the detector's classes are listed. A class outside `detector.class_names` is kept as `other`, and `stats.detections_class_other` counts it |

## 4. Look geometry: how to reproduce a prediction

Notation: ψ is the boat yaw (`ProcessedSSSPing.robot_orientation`, ENU, rad); s = +1 is port and
−1 is starboard; `y_i` are the ping's signed ground offsets (`port_y` > 0, `starboard_y` < 0).

1. **Look bearing**: `b = ψ + s·π/2`.
2. **Sample position**: `p_i = (x, y) + |y_i|·(cos b, sin b) + t`, which is the processor's
   `(x − y_i sin ψ, y + y_i cos ψ)` + t.
   - `|y_i|` is measured from base_link and already includes the processor's 0.285 m lateral
     lever arm. Add none.
   - The PD argument is the ground range from the transducer, `|y_i| − 0.285`.
3. **Sector**: `k = floor(mod(round(deg(b) − aspect_sector_origin_deg, 9), 360) / (360/S))`, on
   the **directed** bearing (0–360°). A bearing on a sector edge goes to the upper sector.
   - Heading East: port looks North (sector 3 of 12), starboard South (sector 9).
4. **A look** of a cell is a run of observations from one sector with gaps ≤ `look_timeout_s`
   (`model_params_json`). A sector change, a timeout or a time reset (> 5 s backwards) opens a
   new look.
5. **ρ** for a new look in sector k is the minimum over the cell's **searched** sectors k' of
   `clip(Δ(k, k')/90°, 0, 1)`.
   - Δ is between sector centres `o + (k + ½)·360/S`.
   - It is **folded onto the line angle** (opposite sides count as parallel) when
     `model_params_json.rho_model.fold_opposite` is true, the default.
6. **Per look**, with PD* = the best covered PD of the look and `c_acc = 1 − Q_open`. Read
   `model_params_json.rho_model.ordering`:
   - `"paull_pairwise"` (the default):
     `Q = (1 − max(c_acc, PD*))·(1 − ρ·min(c_acc, PD*))`. This is Paull Eqs. 22–24 with the
     larger confidence first.
   - `"sequential"`: `ln Q = ln Q_open + ln(1 − ρ·PD*)`.
6a. **Turn back-scan** (`model_params_json.turn_gating`).
   - From the ping poses over `window_s`: `ω = Δψ/Δt`, `v = |Δp|/Δt`.
   - If `|ω| ≥ min_yaw_rate_rad_s`: `r* = v/|ω|` (0 when `v < min_speed_mps`), and the inner side
     is port for ω > 0.
   - Inner-side samples with `|y| > r*` are not counted at all (no look, aspect or coverage).
   - A ping gap longer than `window_s` restarts the estimate.
7. The PD model and every parameter are in `model_params_json`. `pd_model` is
   `trapezoid_placeholder`, **a placeholder**.

## 5. Negative evidence gating

`model_params_json.negative_evidence = "gated"` (the default) means a ping counts as searched
(it updates `coverage` and `p_background`) only where the detector covered it.
`model_params_json.coverage_source` says how that is known (`MATH.md` §8):

- `"records"`: from Perception's image records. A ping is covered iff a record whose detector
  ran on pixels, and is not a dummy, spans it in time. This is exact, and it is used from the
  first valid record on (`coverage.source: auto`, the default).
- `"arrays"`: from detection-array stamps alone, when no record has been seen. A ping is covered
  where consecutive `/sss_ai/detections` stamps `t_{m−1} < τ ≤ t_m` bracket it, with a gap
  ≤ 30 s and neither array from an ignored class (`dummy_center`). This is a heuristic: it never
  credits the first image after a start, and it credits an image that had no pixels.

Pings without coverage are counted **geometry-only**: they update `n_pings`, `n_looks` and
`aspect_counts`. `"ungated"` is a labelled development mode.

A detections-only publisher (the Replanner's emulator) must therefore publish **one array per
image, empty ones included**, as Perception does, or Belief must run ungated.

## 6. Contacts (`BeliefContact`)

- `uid` is never reused, including after pruning.
- `x`, `y` and `covariance` (row-major 2×2, m²) are in the map frame.
- `p_exists` = σ(L). See the priority box above.
- `state` is 0 candidate or 1 confirmed (≥ 2 detected looks).
- `class_probabilities` are aligned with `class_names`.
- `sectors_detected` and `sectors_missed` are unique and sorted; missed = a covered look with
  PD > 0 that closed without a detection.
- `first_confidence` and `first_sector` belong to the creating detection.
- `shadow_known` is false: there is no shadow input yet.
- Per look: `look_bearings_deg`, `look_ranges_m`, `look_detected`.
- `detection_ids` = `"<array stamp ns>:<Detection2D.id>"`, because Perception's ids restart at
  every GCS START.

## 7. `QueryBelief`

Request: `x`, `y`, `radius_m` (map frame). Response fields:

| Field | Meaning |
|---|---|
| `ok`, `error`, `grid_seq` | status |
| `p_any` | `1 − ∏_{cells in disc}(1 − p_bg)·∏_c(1 − p_c·m_c)`, with `m_c` = **the contact's Gaussian mass inside the disc** (never the rendered map) |
| `expected_count` | `Σ p_bg + Σ p_c m_c` |
| `coverage_mean`, `coverage_min` | over the disc's in-area cells |
| `observed_cells` (`n_looks > 0`), `total_cells` | cell counts |
| `min_aspects`, `max_aspects` | over the disc |
| `class_names`, `class_distribution` | mass-weighted |
| `contact_uids` | contacts whose mean lies in the disc |

## 8. Files

Each run gets `<root>/data/Belief_data/<YYYY_MM_DD-HH_MM_SS>/`, claimed exclusively. In it:
- `snapshot_NNNN.npz` + `.json` every `snapshot.period_s`;
- `snapshot_final.npz` + `.json` at shutdown.

`<root>` resolves as `io.data_dir`, then `$BLUEBOAT_DATA_DIR`, then the parent of the first
`$COLCON_PREFIX_PATH` entry, then the cwd.

**`.npz` contents.** Every `BeliefGrid` array and scalar, `header_stamp`, and `contacts_json`
(canonical JSON). The file is byte-deterministic: fixed zip timestamps, sorted members.

**Manifest** (`blueboat_belief/snapshot/1`) keys:
- `contract_version`, `grid_seq`, `stamp`;
- `model_params`, `config` (flat);
- `counters` (every rejection reason), `stats`;
- `input_recording`, `npz_sha256`;
- `summary`:
  - `area_cumulative_pd`: Bourgault 2006 Eq. 7 for one target uniform in the area = the mean
    in-area coverage;
  - `contacts`, `contacts_confirmed`, `contacts_retired`, `alignment_valid`;
- `provenance`: wall clock, excluded from determinism checks.

**Survey-area files.** `blueboat-belief-make-area --out <file> --anchor LAT LON --local XMIN
YMIN XMAX YMAX`, or `--from-world <world>`, writes a validated `blueboat_survey_area/1` file
(never overwrites).

**Recorded inputs.** `record_inputs:=true` writes `<root>/data/Belief_data/<stamp>-inputs/`
(mcap): the four inputs and the image records. The replay tool reads that folder (the one
holding `metadata.yaml`) and is deterministic: the same bag gives a byte-identical `.npz`. It
needs the Python package `rosbags`, which is in the project venv, so run it with that python:

```bash
cd ~/ros2_ws && source env.sh
python3 -m blueboat_belief.cli.replay <bag folder> --area <world | area.yaml> [--config cfg.yaml] [--out dir]
```

The console scripts are installed under `lib/blueboat_belief`, so they are reached with
`ros2 run blueboat_belief <name>`, not from the `PATH`.

## 9. Rejection counters (`/diagnostics` `rejected.<reason>`, manifest `counters`)

`nan_input`, `frozen_pose` (≥ 20 poses at exactly (0, 0) → ERROR), `altitude_unknown`
(`water_depth == 0.22`: the processor had no altitude), `time_reset`, `no_pose_at_stamp`,
`outside_area`, `bad_covariance` (falls back to propagation), `alignment_pending`,
`alignment_queue_overflow`, `no_fix` (`status == −1` or `lat == lon == 0`; MCS's −2 is a fix),
`ignored_class`, `unknown_class`, `duplicate_detection`, `stale_detection`,
`future_detection`, `zero_stamp_array`, `coverage_expired`, `empty_ping`, `bad_record` (an image
record that is not readable JSON, has another schema, or has no time span).

**Stats** (`/diagnostics` `stats.<name>`, manifest `stats`):
- `pings`, `pings_covered`, `pings_geometry_only`: pings **applied** so far (a ping waiting for
  its images is not counted yet);
- `arrays`, `detections`, `contacts_created`, `fixes`, `odom`;
- `detections_class_other`: detections kept under `other` because their class is not configured;
- `records`, `records_searched`, `records_unsearched` (no pixels, no detector input, or a
  dummy), `records_without_array`, `records_overlap` (`MATH.md` §8.2);
- `backscan_pings`, `backscan_samples`: the turn gate.

`/diagnostics` also carries:
- `summary.*` (as in the manifest) and `alignment.*`;
- `detector.calibrated` (configured), `detector.calibrated_observed` and `detector.observed`
  (from the newest image record);
- `coverage.source`: `records`, `arrays` or `none`;
- `latency.event_age_s`: the node clock minus the grid's header stamp;
- `zero_stamps_arrival`: messages whose zero stamp was replaced by the arrival time.

## 10. Conformance to `belief_grid/0`

Every field of the contract text is provided under its contract name and dtype
(`core/contract.py`: `GRID_FIELDS`, `CONTACT_FIELDS`, `to_grid_dict`). The message is built
**only** from `to_grid_dict` (`ros/adapters.grid_to_fields`), so the published grid equals the
offline core bit for bit (checked under ROS by `test/ros_graph_check.py`).

| Check | Where |
|---|---|
| names, dtypes, sizes, NaN rules of every field | smoke [9] |
| `.msg` parse against the field tables, CDR round-trip | smoke [10] |
| consumer semantics (§3a); the Replanner's own `validate_grid` on a real Belief grid | smoke [22] |
| endpoints and QoS on a live graph, every published field against the core, query, latch, exit code, final snapshot | `test/ros_graph_check.py` [R1]–[R5] |

**Where Belief departs from the contract text** (all accepted by the owner; consumers need no
change):

| # | Item | What Belief does | Why |
|---|---|---|---|
| D-C1 | `datum_lat/lon` | the **survey area's** `geo_anchor` (a world `metadata.yaml` in simulation, a `blueboat_survey_area/1` file on the boat) | no mission file carries an area; in a built world odom (0, 0) is the world anchor |
| D-C2 | `class_names` | the configured `detector.class_names` plus `other`; an unlisted class becomes `other` (`stats.detections_class_other`) | YOLO classes are the training worlds' object types. **Open:** set the list from the trained model |
| D-C3 | `first_confidence` | the creating detection's score, **raw while `detector_calibrated` is false** | Perception's calibration is the identity |
| D-C4 | (added) `area_x`, `area_y` | the survey polygon in the map frame | the Replanner checks its mission polygon against it |
| D-C5 | (added) `look_detected`; `detection_ids` format | per-look detected flag; ids `"<array stamp ns>:<id>"` | Perception ids restart at every GCS START |
| D-C6 | `shadow_known`, `has_shadow` | always `false` (degraded, as C-6 allows) | no input carries shadow information |

**`model_params_json` keys** (schema 1): `pd_model`, `pfa`, `rho_model` (`ordering`,
`fold_opposite`, `independence_angle_deg`), `base_rate`, `look_timeout_s`, `turn_gating`,
`association`, `alignment_source`, `area`, `negative_evidence` (`gated` / `ungated`),
`coverage_source` (`records` / `arrays` / `none`), `detector_calibrated` (configured) and
`detector_calibrated_observed` (newest image record; `null` before one).

## 11. Notes for a consumer

Statements about Belief that a planner must take into account (each checked in smoke [22]):
1. **Searched sectors** are `aspect_log_miss < 0`, not `aspect_counts > 0` (§3a). With the dummy
   detector, or none, every look is counted and no sector is searched.
2. **Scores are raw** while `detector_calibrated` is false: `first_confidence`,
   `max_confidence` and `p_exists` are then not probabilities.
3. **Classes.** Mapping detector classes to aspect families is a planner-model matter; Belief
   passes the configured classes through.
4. **Latency is not a constant.** Use `header.stamp`, the evidence horizon (§2).
5. **`grid_seq` restarts at 1** with the node.
6. **Coverage is order-dependent beyond two looks** (`MATH.md` §7): a planning aid, never a
   metric for comparing survey policies.
7. **Model keys a predictor must read:** `rho_model.ordering` and `turn_gating`.
8. **The survey area** comes from Belief's own file; `blueboat-belief-make-area --from-mission`
   derives it from a Replanner mission polygon (its bounding rectangle), and a consumer checks
   its polygon against `area_x`, `area_y`.
9. **A detections-only publisher** (the Replanner's dev emulator) never runs beside Perception,
   publishes one array per image (empty ones included) stamped with the newest covered ping
   time, and uses class names listed in `detector.class_names`; otherwise Belief runs with
   `coverage.require_detector: false`, labelled `ungated`.
