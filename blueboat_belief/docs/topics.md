# BlueBoat-Belief: interface contract (`belief_grid/0`, producer side)

**Status:** implemented 2026-11-04 (cloud session BELIEF-1). It is not yet run in a ROS graph
(`LOCAL_CHECKLIST.md`). Changes from here on are additive (CM-1). Field-by-field conformance:
`CONTRACT_PROVIDED.md`. Deviations: `CONTRACT_DEVIATIONS.md`. Model: `MATH.md`.

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

These are the brief's four topics, by owner decision D-2 (2026-11-04). This deviates from
CM-9's "image records only"; see `CONTRACT_DEVIATIONS.md`. A subscription QoS that does not match
the publisher's receives nothing (CM-4); the subscription QoS below is compatible with every
publisher.

| Topic | Type | Subscription QoS | Publishers |
|---|---|---|---|
| `/sss_processor/processed` | `blueboat_interfaces/ProcessedSSSPing` | BEST_EFFORT, KEEP_LAST 50 | `sss_processor_node` (BEST_EFFORT 10) |
| `/sss_ai/detections` | `vision_msgs/Detection2DArray` | RELIABLE, KEEP_LAST 50 | Perception (RELIABLE 10) |
| `/blueboat/odom` | `nav_msgs/Odometry` | BEST_EFFORT, KEEP_LAST 50 | `robot_interface` / Gazebo bridge / `bag_replay` (RELIABLE, VOLATILE) |
| `/mavros/global_position/global` | `sensor_msgs/NavSatFix` | BEST_EFFORT, KEEP_LAST 10 | mavros, SSS-Sim shim, MCS SimGps (BEST_EFFORT), `bag_replay` (RELIABLE) |

**Mission geo file**, read once at start (`area.file`):
- a simulator world folder or its `metadata.yaml` (`blueboat_world_meta/1`);
- or a `blueboat_survey_area/1` file (`SURVEY_AREA_FORMAT.md`).

Belief never reads simulation ground truth: not `ground_truth/*`, not `/sim/*`, not the world's
`objects`.

### Outputs

| Name | Type | QoS | Rate |
|---|---|---|---|
| `/belief_map/belief_grid` | `blueboat_belief_interfaces/BeliefGrid` | RELIABLE, **TRANSIENT_LOCAL**, KEEP_LAST 1 | `publish.grid_rate_hz` (1 Hz); every publish is a new `grid_seq` |
| `/belief_map/query_belief` | `blueboat_belief_interfaces/QueryBelief` (srv) | — | on request |
| `/belief_map/p_target` | `nav_msgs/OccupancyGrid` (display) | RELIABLE, TRANSIENT_LOCAL, 1 | `publish.display_rate_hz` (0.5 Hz) |
| `/belief_map/coverage` | `nav_msgs/OccupancyGrid` (display) | same | same |
| `/belief_map/contacts` | `visualization_msgs/MarkerArray` | default, 10 | same |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | default, 10 | 1 Hz |

**Subscribing to `belief_grid`.** Use RELIABLE + TRANSIENT_LOCAL + depth 1 to get the last
snapshot on joining; RELIABLE VOLATILE also connects.

**Display encoding.**
- OccupancyGrid `coverage` = round(100·coverage).
- OccupancyGrid `p_target` = round(100·(log10 p − log10 1e-5)/5), i.e. 0 at 1e-5 and 100 at 1.
  A log scale, because `p_target` is ≈ 1e-4.
- −1 outside the area or for a cell with no look.

## 2. Clocks and frames

- **Time.** Every time is the **ROS time stamped on the sonar profiles**: wall time on the real
  boat, sim time under Gazebo.
  - `header.stamp` of `BeliefGrid` is the newest processed event time.
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
   - `"paull_pairwise"` (the default since 2026-11-04):
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
(it updates `coverage` and `p_background`) only where detection arrays bracket it:
- consecutive `/sss_ai/detections` stamps `t_{m−1} < τ ≤ t_m`;
- a gap ≤ 30 s;
- neither array from an ignored class (the dummy detector `dummy_center`).

Pings without that are counted **geometry-only**: they update `n_pings`, `n_looks` and
`aspect_counts`. `"ungated"` is a labelled development mode. Belief therefore expects the detector
to publish **one array per image, empty ones included**, as Perception does.

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
(mcap) of the four inputs. `blueboat-belief-replay <bag> --area <file>` replays it, deterministic.

## 9. Rejection counters (`/diagnostics` `rejected.<reason>`, manifest `counters`)

`nan_input`, `frozen_pose` (≥ 20 poses at exactly (0, 0) → ERROR), `altitude_unknown`
(`water_depth == 0.22`: the processor had no altitude), `time_reset`, `no_pose_at_stamp`,
`outside_area`, `bad_covariance` (falls back to propagation), `alignment_pending`,
`alignment_queue_overflow`, `no_fix` (`status == −1` or `lat == lon == 0`; MCS's −2 is a fix),
`ignored_class`, `unknown_class`, `duplicate_detection`, `stale_detection`,
`future_detection`, `zero_stamp_array`, `coverage_expired`, `empty_ping`.

**Stats** (`/diagnostics` `stats.<name>`, manifest `stats`):
- `pings`, `pings_covered`, `pings_geometry_only`;
- `arrays`, `detections`, `contacts_created`, `fixes`, `odom`;
- `backscan_pings`, `backscan_samples`: the turn gate.

`/diagnostics` also carries `summary.*` (as in the manifest), `alignment.*` and `detector.calibrated`.
