# BlueBoat-Belief: Phase 1 memo (revision b)

Cloud session BELIEF-1, 2026-11-04. This revision supersedes revision a, which was written for an
image-records-only input set. Revision a's still-useful findings are summarised in Appendix A.
Phase 1 wrote no code.

Labels:
- **[REPO]**: repository fact, with `file:line` read in this session.
- **[LIT]**: literature-supported, with whether the primary source was read here.
- **[DECISION]**: owner decision.
- **[ASSUMPTION]**: modelling assumption.
- **[UNKNOWN]**: not known.
- **[external]**: upstream ROS / mavros / Gazebo behaviour not checkable here.
- **[NOT RUN]**: listed in `LOCAL_CHECKLIST.md`.
- **[NOT AVAILABLE]**: depends on the v1 oracle, which does not exist.

---

## 0. Workspace, revisions, environment

| Repo | Path here | HEAD | Note |
|---|---|---|---|
| superproject | `/home/user/BlueBoat-SideScanSonar` | `bc18142` | Its submodule directories are empty; the modules are separate clones under `/home/user/` |
| BlueBoat-Belief | `/home/user/BlueBoat-Belief` | `3ba9281` | `README.md` + these docs |
| BlueBoat-Control | `/home/user/BlueBoat-Control` | `1fadbb2` | = gitlink |
| BlueBoat-MCS | `/home/user/BlueBoat-MCS` | `1d5cb36` | = gitlink |
| BlueBoat-Perception | `/home/user/BlueBoat-Perception` | `8c28025` | = gitlink |
| BlueBoat-SSS | `/home/user/BlueBoat-SSS` | `f85876b` | **Newer than the superproject gitlink `ed759e0`** |
| BlueBoat-SSS-Sim | `/home/user/BlueBoat-SSS-Sim` | `86f8e29` | = gitlink |
| Aug-Studio, Replanner | not cloned | — | Not needed; the Replanner is not read by design |

Environment:
- Python 3.13.18.
- **No `rclpy`**: the node is written, not run.
- numpy 2.4.4, pytest 9.2.0, PyYAML 6.0.3 and scipy 1.17.1 are present.
- `rosbags` is absent; Phase 3 installs it with pip.

The context pack (files 00–03) and `BlueBoat-Belief-v1/` **do not exist** (owner, 2026-11-04). Every
v1-dependent comparison is [NOT AVAILABLE]; the brief's citations of them are re-sourced in §3.

---

## 1. Owner decisions (2026-11-04)

| # | Decision | Consequence |
|---|---|---|
| D-1 | **The brief wins on design** wherever it disagrees with the root `.claude/CLAUDE.md` or `project_synthesis.md`. The repository governs facts about what exists | Deviates from root `CLAUDE.md:53-55` (§1), CM-9 (`:476-479`) and synthesis `:216` ("reads **only** Perception's per-image records … sends **only** to the Replanner"). Recorded in `DESIGN.md` / `CONTRACT_DEVIATIONS.md`. Root-document wording proposed at Phase 5, never edited by this session |
| D-2 | Inputs = the brief's four topics: `/sss_processor/processed`, `/sss_ai/detections`, `/blueboat/odom`, `/mavros/global_position/global`, plus mission geo files (read-only). `/sss_ai/image_records` is **not** an input | §2 |
| D-3 | odom → map: an **online, translation-only, median fit of GPS-fix ↔ odom pairs** (the MCS `GeoFit` model, `BlueBoat-MCS/mcs/core/geo.py:79-175`). `odom_to_map_valid = false` until converged. Observations that arrive earlier are queued (bounded) and replayed in order (brief §7 bug 1). An explicit origin in config overrides it. A sim world's `metadata.yaml` is a cross-check | Phase 2 design |
| D-4 | Survey area: in simulation, the world `metadata.yaml` (`geo_anchor` + `limits {local, corners_gps}`). On the real boat, a config file **of the same shape** (a subset of `blueboat_world_meta/1`), documented as a file format so a configuration tool can produce it later | Phase 2 defines the schema |
| D-5 | No context pack, no v1. Rules come from the root `CLAUDE.md` + synthesis, under D-1. Layout and gates come from the sibling modules. Unsourced facts are marked [OPEN] and asked | §3 |

---

## 2. The four inputs: repository facts

### 2.1 `/sss_processor/processed`: `blueboat_interfaces/ProcessedSSSPing`

Fields (`BlueBoat-Control/blueboat_interfaces/msg/ProcessedSSSPing.msg:29-67`):
`port_stamp`, `starboard_stamp` (Time), `port_ping_number`, `starboard_ping_number` (uint32),
`robot_x`, `robot_y` (float32), `robot_orientation` (Quaternion), `water_depth`,
`transducer_x_offset` (float32), and `port_intensity_db`, `port_y`, `starboard_intensity_db`,
`starboard_y` (float32[]).

| Fact | Ref |
|---|---|
| Publisher QoS: **BEST_EFFORT, KEEP_LAST 10**. Belief subscribes BEST_EFFORT (depth ≥ 50, brief §3) | `BlueBoat-SSS/blueboat_sss/src/sss_processor_node.py:596-600,638` |
| About 20 Hz (`msec_per_ping` 50), one row per offset-normalised ping pair. **Rows come in bursts and not strictly in order**: a startup pre-roll of up to 24 pings, and stale one-sided groups emitted after the complete row that triggered them | `:1291-1359` |
| `*_stamp` = the profile header stamp: wall time on the real boat (`sss_node`), sim time under Gazebo. An absent side gets a zero stamp | `:1543-1544` |
| `robot_x/y` is the **interpolated** odom position (linear, bracketing samples ≤ 1 s each, else nearest ≤ 1 s, else the row is dropped) at the port stamp, else the starboard stamp. The brief's "nearest odom sample" and the msg comment `ProcessedSSSPing.msg:22-23` are stale | `:357-391`, `:1451`, `:1527-1538` |
| `robot_orientation` is **yaw-only on the real boat** (`robot_interface` publishes `yaw_to_quaternion`), and full attitude in sim and in bag replay | `robot_interface.py:639` |
| `water_depth = altitude + 0.22` (measured 2026-11-02). **Unknown altitude → `altitude = 0.0`**, so `water_depth == 0.22` exactly and `*_y` is slant range + lever arm, not ground range. No lock flag is published | `:142`, `:1483-1484`, `:1498` |
| `transducer_x_offset` is always 0.0 | `:139` |
| `*_intensity_db` = `min_pwr_db + raw/65535·(max_pwr_db − min_pwr_db)`, the device's normalised power scale. **No gain or TVG compensation** | `_custom_libraries/sss_helper.py:66-77` |
| `*_y = ±(0.285 + sqrt(s² − h²))`, + for port, near → far. The water column (`slant ≤ altitude`) is removed; the processor applies **no nadir blank**. **The lever arm is already included: Belief adds none** | `sss_helper.py:224-232`; `sss_processor_node.py:1508-1520` |
| Up to **1199 samples per side** at 20 m / 1200 bins (1080 at h = 2 m). The ground step is ~0.1 m just past nadir, shrinking to 16.7 mm far out. The brief's v1 timing figure assumed 600 per side | agent count on `sss_helper.py`; to measure in Phase 4 |
| One-sided rows: **presence = `*_ping_number != 0`**, never array length. The only drop is a missing pose | `ProcessedSSSPing.msg:7-8`; `:1458-1469` |

### 2.2 `/blueboat/odom`: `nav_msgs/Odometry`

| Mode | Facts | Ref |
|---|---|---|
| Real | RELIABLE / VOLATILE / depth 10. **Yaw absolute ENU** and position translated to the launch point (`x − x0`), no rotation. The header is copied from mavros, and mavros runs `use_sim_time` (`SimpleLauncher(use_sim_time = True)`) with no `/clock`, so **the stamp can be 0**. Covariance is copied from mavros; expected all-zero [external, L-2]. Rate 4–13 Hz, unsteady (root `CLAUDE.md` §4.4). The origin is latched on the second mavros message | `robot_interface.py:117`, `:604-644`; `BlueBoat_launch.py:3` |
| Sim | Gazebo `OdometryPublisher`, 20 Hz in sim time, frame `world`, full attitude. Covariance zero (gz Odometry has none) [external] | `blueboat_description/.../blueboat.xacro:106-113`; `upload_rov_launch.py:51-53` |
| Bag replay | RELIABLE, full recorded attitude, covariance as recorded, stamps on replay (sim) time | `bag_replay.py:538,671-684` |

All three publishers are VOLATILE. A **TRANSIENT_LOCAL subscriber would receive nothing**.
`sss_processor_node` keys zero-stamped odom on its own arrival time, on wall time, because the SSS
launch has no `use_sim_time` (`sss_processor_node.py:279-312`, `:703`). Belief must do the same:
**the node runs on wall time on the real graph and on sim time under Gazebo.**

### 2.3 `/mavros/global_position/global`: `sensor_msgs/NavSatFix`

Subscribe **BEST_EFFORT**: it is the only QoS that hears all four publishers.

| Publisher | When | QoS | Status | Stamp | Position / noise | Ref |
|---|---|---|---|---|---|---|
| mavros | real | SensorData, BEST_EFFORT [external] | 0, or −1 = no fix [external] | can be 0 | real GPS | — |
| SSS-Sim `mavros_shim_node` | built worlds | BEST_EFFORT, depth 5 | 0 | sim time | **exact**, about the world anchor | `mavros_shim_node.py:71-73,120-133`; `sss_sim_launch.py:181-195` |
| MCS SimGps (`bridge_node`) | empty-world, GPS-anchored missions | BEST_EFFORT, depth 10 | **−2** (unset) | wall time | σ = 0.4 m seeded; origin 10 m north of the first point | `mcs/core/sim_gps.py:43-49`; `mcs/config/settings.py:357-365` |
| `bag_replay` | replay | **RELIABLE**, depth 10 | as recorded | replay time | as recorded | `bag_replay.py:534,604,661` |

No-fix rule (root `CLAUDE.md` §4.4): **`status == −1` or `lat == lon == 0`**, never `status < 0`.
`sss_processor_node.py:1050` and `bag_replay.py:215` gate `status < 0` and so drop every MCS fix;
that is their issue, noted for the owner.

### 2.4 `/sss_ai/detections`: `vision_msgs/Detection2DArray`

Published by BlueBoat-Perception (`ros/convert.py:47-75`, `ros/perception_node.py:121,257-258`).

| Fact | Consequence for Belief |
|---|---|
| **One array per seabed image, even when empty, even for `no_pixels`**. Then the stamp is 0 if the image time is null (`t_max_s or 0.0`) | An empty array is **ambiguous**: nothing ≥ 0.10, or no pixels |
| Only detections with `confidence >= map_min_confidence` (0.10) are included (`core/pipeline.py:62-66`) | The operating point is fixed upstream at 0.10 |
| Array `header.stamp` = the image's newest row time. `frame_id = "odom"` | The image's row span is not on the wire |
| `detections[k].header.stamp` = **the detection's row time** (`t_row_s`) | The acquisition time the brief §3.2 asks for: provided |
| `id` = `str(uid)` = `2000000 + 16·image_id + k`. **`image_id` restarts at every GCS START** | **Ids repeat across GCS streams**. "Same id ⇒ update" (brief §3.2) is safe only within one stream |
| `bbox.center.position` = odom-frame x, y; `bbox.center.theta` = boat heading at the row (GCS compass); `size_x/size_y` = extent across/along | Side and range must be derived (brief §3.2) |
| `results[0].hypothesis.class_id` = class name (dummy: `dummy_center`); `score` = `confidence`, **raw while uncalibrated** | The dummy can be recognised by class name |
| `results[0].pose.covariance[0,1,6,7]` = the 2×2 block (`geometric_v1`). It includes **σ_nav = 1 m isotropic per detection** and can be NaN if the row's altitude or yaw is NaN (`core/uncertainty.py:46-65`) | Validate the covariance (brief §7); treat σ_nav as common-mode (S-8) |
| **Not on the wire:** the `calibrated` flag, the `dummy` flag, image key, image span, side, range | See S-2 and S-3 |
| Publisher QoS: default RELIABLE depth 10. The GCS subscribes depth 10 reliable (`gcs/ros/detections_listener.py:62`) | Belief subscribes RELIABLE, depth ≥ 10 |
| **When there are no arrays at all.** Perception is started by MCS only for real-robot and built-world launches, never for the empty world or a bag replay (Perception `.claude/CLAUDE.md` §2). The GCS produces images only between START and STOP, and in `square` geometry only while moving (`gcs/core/seabed_imager.py:557-574`). There are therefore **no arrays at all** in empty-world runs, bag replays, and while the GCS is stopped | Pings flow without a detector (S-3) |
| Consecutive images overlap ~50 % on the same pings (`seabed_imager.py:34`). Perception does not dedupe across images (Perception `docs/DESIGN.md:308`) | Each object is detected about twice per pass with different ids, centimetres apart |
| The dummy detector emits one detection per image at the nadir pixel, score 0.5 (≥ 0.10, so on the wire), σ ≈ 12 m (`core/detectors/dummy.py:27-32`) | Fed naively, it makes a chain of contacts along the track |
| Detection positions use the GCS's live heading = **the compass at GCS processing time** (fallback: odom yaw after 2 s) (`gcs/gui/main_window.py:479-493`) | A different heading source from the pings' odom yaw (S-14) |

### 2.5 Mission geo files (read-only)

- **Designer trajectory** `~/.config/blueboat_mcs/trajectories/<name>.yaml`. It carries
  `geo_anchor {lat0, lon0, theta_deg}` and points, and **no area**
  (`mcs/designer/io_yaml.py:101-120`, `BlueBoat-MCS/docs/05_trajectory_format.md:18-60`).
- **Deployed and session files** (`.deployed/*.yaml`, `.session/run.yaml`). `deploy_mission`
  **pops `geo_anchor`** and keeps only `deployed_from` (a path) and `deployed_fit_rms_m`; the fit
  translation is not written (`io_yaml.py:231-235`).
- **World `metadata.yaml`** (`blueboat_world_meta/1`, `worldgen/world_folder.py:398-417`;
  `docs/configuration_guide.md:631-651`). It has `geo_anchor`, `spawn`,
  `limits {local {x_min, y_min, x_max, y_max}, corners_gps [SW, SE, NE, NW]}`, `walls`,
  `objects` and `source_path.design_offset` (`world = design − offset`).
  - In a built world, **odom (0,0) = the world anchor**: the shim publishes GPS about it.
  - Belief must not read `objects` online. That is ground truth, and the brief §1/§13.4 forbids it.
- **Survey area.** No mission file has one (D-4).
- **Geodesy.** Copied, never imported; asserted bit-identical to
  `BlueBoat-MCS/mcs/core/geo.py:38-52`:
  - `EARTH_RADIUS_M = 6378137.0`;
  - `east = radians(lon − lon0)·R·cos(radians(lat0))`;
  - `north = radians(lat − lat0)·R`;
  - and the inverse.
- **Control's per-run `-origin.yaml`** holds the odom origin's lat/lon. It is latched **without a
  validity check**, so it reads `[0, 0]` if no fix had arrived (`robot_interface.py:199,614-621,838-860`).
  It is usable only as an explicit config override, with a zero check.

---

## 3. Brief citations of the non-existent context pack, re-sourced (D-5)

| Brief cites | Re-sourced to | Status |
|---|---|---|
| file 01 §3.1 (real odom yaw absolute, "VERIFIED") | root `CLAUDE.md` §4.1 `/blueboat/odom` row; `robot_interface.py:627` | found |
| file 02 §5.3 (no simulation ground truth online) | brief §1 and §13.4 (wins, D-1), with CM-2 as the rationale; Perception NC #4 is the module-local analogue | found (brief rule) |
| file 02 §5.3 / brief §12.6 (CM-1 wording) | root `CLAUDE.md:422` and synthesis `:50-52` already say new modules may own their interfaces package | **already done upstream**; drop from requests |
| file 02 §5.4 (channel names) | synthesis §5.2: `p_target`, `n_observations`, `aspect_coverage`, `last_intensity`, `max_confidence` | found |
| file 02 §5.2 (package template) | Perception (`blueboat_perception/` one level down: `package.xml`, `setup.py`, `setup.cfg`, `resource/`); SSS-Sim (same layout); Control `blueboat_interfaces` (ament_cmake rosidl) | re-sourced |
| file 02 §5.7 (`CLAUDE.md` skeleton) | Perception `.claude/CLAUDE.md` (193 lines: Layout · Interfaces · Non-negotiables · module specifics · Commands · Gates · Where documented); SSS-Sim `TODO.md` `[P0]/[P1]/[P2]` legend; no `.claude/rules/` exists in any module | re-sourced |
| file 02 §4–5 (gate) | skill `module-gates`. SSS-Sim `test/smoke_test.py`: `check()` prints `[PASS]/[FAIL]`, fails fast with exit 1, `[N]` section prints, `ALL CHECKS PASSED`, manual count in `CLAUDE.md`. Perception pytest: `find_spec` + `skipif`, import sweep that blocks ROS on `sys.meta_path` (`tests/conftest.py:37-82`) | re-sourced |
| file 02 §5.6 (integration) | root `CLAUDE.md` §2, §3, §7; skills `module-gates`, `file-interfaces` | found |
| file 03 B4 (first live simulator run) | SSS-Sim `CLAUDE.md` §3.5 and §7 (`full_mission_launch.py world_dir:=… with_control:=true trajectory_file:=…`, ping enable); MCS `CLAUDE.md` §7; SSS `CLAUDE.md` Commands (`SSS_processing_launch.py`) | found |

Nothing is [OPEN] so far.

---

## 4. The brief's [VERIFY] items

1. **Real odom yaw absolute ENU, translation only.** Verified in code
   (`robot_interface.py:614-639`: "Absolute ENU; yaw0 is provenance for the sidecar only").
   Runtime check: L-1. v1's "rotated by −yaw₀" does not apply to today's code. So odom → map is a
   translation, and `odom_to_map_yaw` stays 0 unless L-1 fails.
2. **What a GCS "Save as rosbag" recording contains** (`gcs/tools/svlog_to_rosbag.py:283-299`,
   `:112-117`, `:334-343`, `:542-567`):
   - 14 topics: both sides' `profile` and `raw`, ten `/mavros/*` topics (NavSatFix included), and
     a `/blueboat/odom` synthesised from ATTITUDE + LOCAL_POSITION_NED;
   - **no `/sss_processor/processed` and no `/sss_ai/*`**;
   - stamps are synthetic (2024-01-01 plus 20 ms ticks);
   - sonar is routed by `src`, but the processor re-resolves sides from the packet.

   A real-recording replay for Belief is therefore: play the bag → run `sss_processor_node` → the
   four topics (no detections exist for real data). **Control's run bag excludes
   `/sss_processor/*` and `/sss_ai/*`** (`run_bag.py:52`), so a real run needs its own
   `ros2 bag record` of Belief's four topics (REQUESTS item, L-6).
3. **The `Detection2DArray` fields the GCS reads** (`gcs/ros/detections_listener.py:72-94`):
   - read: the array and per-detection stamps, `id`, `results[0].hypothesis.class_id`/`score`,
     `bbox.center.position.x/y`, `bbox.size_x` (≥ 0.5);
   - not read: `size_y`, `theta`, `frame_id`.

   Perception fills more than that (§2.4).

---

## 5. Brief statements that are stale (facts follow the repository, D-1)

| Brief says | Repository says |
|---|---|
| §2: D2 beach dataset critical path; D3 not implemented, unowned | Perception exists. Its dummy is the default; YOLO is built but untrained; calibration is identity. Training is in simulation (synthesis §9 D2/D3, CM-11) |
| §2, §3.1: lever arm "sim 0.20 m, processor 0.0" | Both are **0.285 m** (root `CLAUDE.md` §4.3; `sss_processor_node.py` constants) |
| §3.1: pose = nearest odom sample | Interpolated since 2026-10-27 (§2.1) |
| §3.1: reserved 30 m / 15 m ranges | **20 m per side for every condition** (synthesis §8.5) |
| §2: Control/MCS v1.1 imminent | Both are add-only after v1.1 (root `CLAUDE.md` §3) |
| §12.6: update the CM-1 wording | Already updated (root `CLAUDE.md:422`) |
| §3.2 GCS contract `bbox.size_x` = extent | Perception fills `size_x` across and `size_y` along |

---

## 6. Contract `belief_grid/0`: status against the facts

**Providable as specified:**
- C-1: Belief subscribes `/sss_ai/detections` with `Detection2DArray`.
- C-2: topic and QoS.
- C-3: every channel.
  - `datum_*` and `odom_to_map_*` come from the D-3 fit; `odom_to_map_yaw = 0` (§4.1).
  - `odom_to_map_sigma_m` comes from the fit residual.
  - `last_intensity_db` and `depth_m` come from the pings; `depth_m` is the flat-floor depth of
    the last ping observing the cell, labelled.
  - `n_pings` counts distinct pings.
- C-4: every contact field, including `first_confidence` and `first_sector`.
- C-5: look geometry. `b = ψ + s·π/2`, port s = +1, matches Perception's
  `look_port = wrap(yaw + π/2)` (`core/geometry.py:87-90`).

**Deviations to record in Phase 2:**

| # | Item | Fact | Handling |
|---|---|---|---|
| X-1 | C-4 `first_confidence` "calibrated q" | No `calibrated` flag on the wire; `score` is raw today | Publish the score. `model_params_json` states `detector_calibrated` from Belief config (default `false`) |
| X-2 | C-3 `class_names` default list (5 classes) | YOLO class names = the training worlds' object types; the dummy uses `dummy_center` | A fixed configured list plus `other`; unknown names are counted |
| X-3 | Coupling: the Replanner's dev emulator publishes on `/sss_ai/detections` | It would be a second publisher beside Perception | It must never run with Perception. If Belief uses the arrays as its detector-coverage signal (S-3), the emulator must also publish one array per image, empty ones included. Reconcile in-machine |

---

## 7. Scientific check (brief §17; prompt F)

| # | Finding | Kind |
|---|---|---|
| S-1 | The miss update `ln(1 − ρ·PD)` and Bourgault's form are the **PFA = 0 special case** of `ln((1 − PD)/(1 − PFA))`. Bourgault's sensor "can miss but never false-alarm" (project notes `literature_conclusions.md:212-214`, marked [full] by their author; the primary is paywalled and **not re-read here**). The operating point is fixed upstream at confidence ≥ 0.10 (§2.4); PD and PFA at that point are [UNKNOWN] | [LIT] notes + [REPO] |
| S-2 | `ln LR = logit q − logit π` needs a calibrated posterior for the same event and population. Today `calibrated: false`, the YOLO score is a class score conditional on a candidate box (Perception DESIGN §4.4), π is undefined, and **no `calibrated` flag reaches Belief**. A candidate's existence update factorises into candidate generation (PD versus false-candidate density) × score likelihood ratio. Using `logit q` alone omits a factor; adding both naively double-counts | [LIT] reasoning (brief §17.5–17.6) + [REPO] |
| S-3 | **Is a non-detection evidence?** Pings flow whether or not a detector runs (empty world, replay, GCS stopped). Empty arrays are ambiguous (§2.4). An array carries only its newest row time. Crediting ping-level misses without detector coverage would invent negative evidence (brief §17.9). Options for Phase 2: (a) credit misses only for pings bracketed by received arrays (a heuristic coverage timeline); (b) the brief §12.1 optional `/sss_ai/detection_meta` companion (image span, status, `calibrated`, `dummy`) as a REQUEST; (c) geometry-only updates (looks, aspects, pings) without a detector | [REPO] + design question |
| S-4 | Image overlap affects only positive evidence: pings arrive once, but each object is detected about twice per pass with different ids. The brief's same-look replacement (§6.5) keeps one contribution per look | [REPO] |
| S-5 | Paull et al. 2013 (primary paywalled, **not re-read**; project notes [full] `literature_conclusions.md:221-236`, brief §17.4).<br>(a) α is the acute angle between insonification lines in [0°, 90°], so **directed looks 180° apart count as dependent**, while a directed-bearing Δ would call them independent.<br>(b) The pairwise rule orders c₁ ≥ c₂. The sequential recursion `1 − (1 − c_acc)(1 − ρ·PD)` differs when PD > c_acc: c_acc = 0.3, PD = 0.7, ρ = 0.5 gives 0.745 (Paull) against 0.545 (sequential).<br>(c) Paull does not define which earlier look ρ refers to.<br>The rule stays a labelled heuristic | [LIT] notes + [ASSUMPTION] |
| S-6 | Yordanova & Gips 2020, **primary read here** (arXiv 2006.12896 preprint).<br>- Eq. 2: `RR = ∏ᵢ(1 − Pdᵢ)` per grid point over all looks.<br>- Platform: side-looking **SAS** on the MUSCLE Bluefin-21.<br>- Pd: the ATR's detection probability from through-the-sensor features, reduced to range only and updated after each track.<br>- No false-alarm term.<br>It supports Q as a residual-risk map; its numbers do not transfer (SAS→SSS is an analogy) | [LIT] |
| S-7 | Prior `p0 = λ` against `1 − e^{−λ}`: at 5 /ha and 0.5 m, λ = 1.25e-4 and the difference is 7.8e-9 (6e-5 relative). Negligible; labelled | computed |
| S-8 | σ_nav (1 m) is added to **each** detection's covariance independently (§2.4). Information-form fusion of N detections would shrink it by √N, so it must be treated as common-mode (removed before fusion, restored as a floor). Validate by ellipse coverage (brief §17.7) | [REPO] + [ASSUMPTION] |
| S-9 | Association: every detection has σ ≥ 1 m per axis, so the χ² 9.21 gate on Σ_obs + Σ_c is ≥ ~4.3 m and **the 2 m project criterion dominates** except for tight contacts (to test both). Ids repeat across GCS streams (§2.4) | computed + [REPO] |
| S-10 | Aspect geometry is reconstructable per ping: `b = ψ ± π/2` from `robot_orientation`. On the real boat that is a yaw-only quaternion = odom yaw, absolute ENU. Each ground sample of a ping is a look along `b`. The beam is ±0.5° in azimuth (Perception's `beam_azimuth_rad` 0.00873) | [REPO] |
| S-11 | No data separate target-relative aspect response from inter-look dependence (E3 not run). The first version keeps one labelled angular factor | [UNKNOWN] |
| S-12 | No fitted PD exists. SSS-Sim `mission_metrics --aspect-bin-deg` (default 15°, `analysis/cli.py:59`) gives simulated ground-truth visibility; fitting there and evaluating there is circular (brief §8.5) | [REPO] |
| S-13 | **Unknown altitude**: the processor uses altitude 0, so `water_depth == 0.22` and `*_y` is slant + lever arm. Cells near nadir are then misplaced outward by up to about h. Detect the sentinel and do not credit those samples (Phase 2) | [REPO] |
| S-14 | Detections are georeferenced with the GCS compass heading at reception; pings use odom yaw. A compass/odom yaw offset shifts positive against negative evidence systematically. Check L-4 | [REPO] |
| S-15 | Up to ~2400 samples per ping row at 20 Hz. The brief's < 1 ms per swath budget at 0.5 m has to be re-measured at this sample count (cloud numbers are indicative only) | [REPO] |

---

## 8. Questions for the Phase 2 gate (preview; asked with recommendations there)

- **Brief §16:** latched against periodic publication (contract C-2: TRANSIENT_LOCAL + 1 Hz);
  per-sector miss accounting; where a synthetic detector lives; the meta companion; gate style;
  v1 reuse (moot, D-5).
- **New:**
  - detector-coverage gating (S-3);
  - behaviour while uncalibrated (S-2);
  - dummy-detector policy;
  - Paull angle folding and ordering (S-5);
  - the unknown-altitude sentinel (S-13);
  - recording Belief's four topics in real runs (§4.2);
  - id reuse across streams (S-9).

---

## Appendix A: revision a (image-records-only input), superseded by D-1/D-2

Kept because the facts still inform the design:
- `/sss_ai/image_records` carries per-side searched polygons, per-row tracks and look bearings, and
  `detections: []` against `null` semantics. That is exactly the "valid opportunity" signal S-3
  lacks on the four topics. A meta companion (S-3 option b) would carry the same information.
- Image records give offline = live byte-equal replay (`blueboat-perception-offline`).
  `ProcessedSSSPing` has no such guarantee: BEST_EFFORT, bursts. Belief's determinism must therefore
  be demonstrated on a **recorded** sequence replayed through the core offline (`rosbags`), not on
  live ROS replays.
- The GCS's `pose_source: auto` may re-stamp row poses from GCS telemetry with a different origin
  when ping poses freeze (`gcs/gui/main_window.py:440-477`). That concerns the GCS's images and so
  the detections' positions. Belief's own pings carry the processor's raw pose, so frozen (0,0)
  poses reach Belief directly (brief §7: frozen-pose detection, L-8).

---

## Addendum (2026-11-04): primary-source check, after the owner supplied the PDFs

Paull et al. 2013 and Bourgault et al. 2006 were read in full. S-1 and S-5 now rest on the
primary texts, not the project notes. Details: `MATH.md` §13, `REFERENCES.md`.

- **S-1 confirmed.**
  - Bourgault §4.1: "The sensor is assumed to have perfect discrimination i.e. no false target
    detection."
  - Its miss update is Eq. 4.
  - `Q_k = ∏ q_i` and `P_k = 1 − Q_k` are Eqs. 6–7, for **one normalised target PDF** (not
    per-cell occupancy).
- **S-5 confirmed and sharpened.**
  - (a) α is "the acute angle of the intersection of two lines with directions θ₁ and θ₂", so
    folding is Paull-faithful.
  - (b) "Without loss of generality, we can assume that c¹ ≥ c²". The owner then switched the
    default to `paull_pairwise`.
  - The linear rule is Paull's own assumption ("it is assumed"). Fawcett is cited for the value
    of multiple views, not for linearity: the brief §6.3 wording "Paull cite Fawcett 2010 for it"
    overstates.
- **New.**
  - Paull's confidence is `P(m = M)` with symmetric errors (Eqs. 5, 9), not PD, so Eq. 9 is no
    consistency check for Belief.
  - Paull §II-C turns gave the turn back-scan gate.
  - Bourgault Eq. 7 gives `summary.area_cumulative_pd`.
  - Bourgault §4.3 smoothness is the reason for the [P2] bearing-resolved ρ TODO.
