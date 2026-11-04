# BlueBoat-Belief: Phase 2 design (for approval)

Cloud session BELIEF-1, 2026-11-04. The facts are in `PHASE1_MEMO.md` (rev. b). The owner
decisions D-1 … D-5 are in memo §1. This is a design only: no code exists yet.

Contract conformance is in `CONTRACT_PROVIDED.md`; deviations are in `CONTRACT_DEVIATIONS.md`.
Everything numeric is a placeholder unless a source is named. Every result is model-conditional
(CM-13).

---

## 1. Architecture (smallest design that meets brief §1 and §11)

```
/sss_processor/processed (BEST_EFFORT) ─┐
/sss_ai/detections       (RELIABLE)    ─┤  belief_node (rclpy, thin)     ROS-free deterministic core
/blueboat/odom           (BEST_EFFORT) ─┼─► ros/adapters.py ──────────► core/belief.py (facade)
/mavros/.../global       (BEST_EFFORT) ─┘   (duck-typed msg → dataclass)   ├ alignment  (odom→map fit)
survey area file / world metadata.yaml ───► io/missions.py ──────────►     ├ pose       (buffer, frozen)
                                                                           ├ coverage   (detector timeline)
                                                                           ├ grid       (looks, misses)
                                                                           ├ contacts   (Bernoulli table)
                                                                           └ contract.to_grid_dict()
   /belief_map/belief_grid (BeliefGrid, TRANSIENT_LOCAL + 1 Hz) ◄─ adapters.grid_to_msg(to_grid_dict(...))
   /belief_map/query_belief (QueryBelief.srv)       ◄─ core.render.query_disc
   /belief_map/p_target, /belief_map/coverage (OccupancyGrid, display), /belief_map/contacts (MarkerArray)
   /diagnostics (DiagnosticArray: OK/WARN/ERROR + every rejection counter)
   <root>/data/Belief_data/<run>/snapshot_NNNN.npz + manifest.json  (O_EXCL; periodic and at shutdown)
```

- The node name is `belief_map`, so `~/belief_grid` resolves to the contract's `/belief_map/belief_grid`.
- The core takes **explicit times**: the sonar-profile clock, i.e. wall time on the real boat and
  sim time under Gazebo.
- A message with a zero stamp is stamped with its arrival time **by the node**, from the node clock
  (`use_sim_time` true in simulation, false on the real boat, L-2), and the core counts it.
- The core reads no wall clock, draws no random numbers, and iterates in sorted orders only.
- **Determinism (T12)** is demonstrated by replaying a recorded event sequence through the core
  offline. `cli/replay.py` reads a rosbag2 with `rosbags` in log-time order. Live ROS replays are
  not bitwise-reproducible, because of BEST_EFFORT drops and burst timing (memo Appendix A).

### 1.1 Repository layout

The layout follows Perception and SSS-Sim (memo §3).

```
BlueBoat-Belief/
├── README.md
├── .claude/CLAUDE.md (< 200 lines, §8), TODO.md ([P0]/[P1]/[P2] legend, SSS-Sim style)
├── blueboat_belief_interfaces/      ament_cmake + rosidl: msg/BeliefGrid.msg, msg/BeliefContact.msg,
│                                    srv/QueryBelief.srv; depends std_msgs, builtin_interfaces only
└── blueboat_belief/                 ament_python (one level down)
    ├── package.xml, setup.py, setup.cfg, resource/blueboat_belief
    ├── blueboat_belief/
    │   ├── core/  config.py geo.py area.py alignment.py pose.py models.py geometry.py
    │   │          grid.py contacts.py coverage.py belief.py contract.py   (numpy + stdlib)
    │   ├── io/    missions.py (world metadata / survey-area file), snapshot.py, dataroot.py
    │   ├── ros/   adapters.py (ROS-free, one field-name table), belief_node.py (only rclpy file)
    │   └── cli/   replay.py (rosbags → core → snapshots; console script)
    ├── launch/belief_launch.py      (SSS-Sim pattern: optional `simple_launch` import)
    ├── config/belief.yaml, config/survey_area.example.yaml
    ├── docs/  MATH.md (spec = code) topics.md DESIGN.md REQUESTS.md SURVEY_AREA_FORMAT.md
    │          CONTRACT_PROVIDED.md CONTRACT_DEVIATIONS.md LOCAL_CHECKLIST.md HANDOVER_CLOUD.md
    └── test/  smoke_test.py, msg_reference/ProcessedSSSPing.msg (verbatim copy, CM-3)
```

---

## 2. Input handling (node → adapters → core)

| Input | Subscription QoS | Adapter output | Core use |
|---|---|---|---|
| `/sss_processor/processed` | BEST_EFFORT, KEEP_LAST 50 | `PingObs(t, pn_port, pn_stbd, x, y, yaw, water_depth, port_y, port_db, stbd_y, stbd_db)`. Yaw comes from `robot_orientation`; `t` is the port stamp, else the starboard stamp | Pose clock, swath footprint, depth, intensity |
| `/sss_ai/detections` | RELIABLE, KEEP_LAST 50 | `DetArray(t_image, [Det(id, t_row, x, y, theta, size_x, size_y, class, score, cov2)])`. Covariance is read from indices [0,1,6,7] | Detector coverage timeline; positive evidence |
| `/blueboat/odom` | BEST_EFFORT, KEEP_LAST 50 (compatible with every RELIABLE/VOLATILE publisher, L-3) | `OdomObs(t, x, y, yaw, cov2)` | Pose buffer (detections' boat position, fix pairing); frozen-pose detector |
| `/mavros/global_position/global` | BEST_EFFORT, KEEP_LAST 10 | `FixObs(t, lat, lon, status)` | odom → map fit (D-3) |

### 2.1 Rejection reasons

Each is counted, published on `/diagnostics`, and never raised as an exception:
- `nan_input`
- `stale` (older than `look.time_reset_s` behind the newest)
- `future`
- `time_reset` (a backward jump larger than `time_reset_s`, 5 s; starts new looks; bursts below it are accepted)
- `no_pose_at_stamp`
- `frozen_pose` (≥ 20 consecutive poses exactly at (0, 0) → ERROR)
- `altitude_unknown` (`water_depth == 0.22`, memo S-13)
- `outside_area`
- `bad_covariance` (falls back to propagation)
- `alignment_pending` (queued) / `alignment_queue_overflow`
- `no_fix` (`status == −1` or `lat == lon == 0`)
- `ignored_class`
- `duplicate_detection` (same `(id, t_row)`)
- `late_detection`
- `coverage_expired` (geometry-only)
- `zero_stamp_arrival`

### 2.2 odom → map (D-3)

**Datum.** Every input is in the odom frame. The map frame is ENU about the **datum** = the survey
area's `geo_anchor` (§4; Q-2).

**Fit (`alignment.py`, the MCS `GeoFit` model).**
1. Pair each valid fix with the odom position interpolated at the fix time (tolerance 0.25 s).
   Fixes and odom are both zero-stamped on the real boat, so both are on arrival times (L-2).
2. Compute `d = EN(fix; datum) − odom_xy`.
3. Take the per-axis median over a window of 200 pairs.
4. It becomes valid after ≥ 10 pairs with a robust spread `1.4826·MAD ≤ 3 m`.
5. **Latch** once valid (Q-14), so cells already written never move.

**Published fields.**
- `odom_to_map_sigma_m` = the robust spread. It is conservative: GPS errors are time-correlated,
  so it is not divided by √n.
- `odom_to_map_yaw = 0`: real yaw is absolute ENU (memo §4.1, L-1).

**Override.** `alignment.origin_lat/lon`, the GPS of odom (0, 0), for example copied from
Control's `-origin.yaml` after checking it is not (0, 0). When set, the translation is
`t = EN(origin)`, valid at once, with the configured sigma.

**Sim cross-check.** When the area comes from a world `metadata.yaml`, a fitted `|t|` larger than
0.5 m raises a WARN: odom (0, 0) must equal the world anchor (L-5).

**Before alignment.** Pings and detections are queued (bounded at 6000 events, about 5 min) and
replayed in order once the fit is valid (brief §7 bug 1). Fixes go to the aligner first.

### 2.3 Detector coverage (memo S-3; Q-7)

Pings are evidence of a **miss** only where the detector processed them.

**The coverage timeline (`coverage.py`).**
1. Take the stamps `t_k` of received detection arrays, which are the images' newest row times.
2. Image k spans back past `t_{k−1}` (stride 128 of 256 rows, `seabed_imager.py:34`). So once
   array k has arrived, every ping with `t ≤ t_{k−1}` has been seen by all the images it belongs
   to, and all of its detections are in.

**What happens to a ping.**
- **Released as covered:** the ping is in `(t_{k−2}, t_{k−1}]`, the gap `t_{k−1} − t_{k−2}` is at
  most 30 s (`coverage.max_array_gap_s`), and the arrays involved held no ignored class.
- **Released as geometry-only:** the ping waits longer than 60 s, or falls in a gap. It updates
  `n_pings`, `n_looks` and `aspect_counts`, and **does not** update `ln Q` or `ℓ`.
- **Dummy detector (Q-9):** its arrays always hold `dummy_center` and are marked "not a detector".
  Their pings are geometry-only.

**Without a detector.** With `coverage.require_detector: false` (development only: empty world,
bag replay), every ping is credited at once with the placeholder PD. `model_params_json` then says
`negative_evidence: ungated`.

---

## 3. Model (the summary of what `MATH.md` will specify; each line names its function and test)

Symbols:

| Symbol | Meaning |
|---|---|
| ψ | yaw (rad, ENU) |
| s | side: +1 port, −1 starboard |
| y_i | the ping's ground offsets, signed, from base_link, lever arm included |
| r_i | `\|y_i\| − 0.285`: ground range from the transducer, used by PD |
| S | sector count, 12 |
| o | `aspect_sector_origin_deg`, 0 |

1. **Sample position** (`geometry.sample_xy`):
   - `b = ψ + s·π/2`;
   - `p_i = (x, y) + |y_i|·(cos b, sin b)`, mapped to the map frame by `+ t`;
   - this equals `(x − y_i sin ψ, y + y_i cos ψ)`, the processor's own geometry.
2. **Sector** (`geometry.sector`): `k = floor(mod(deg(b) − o, 360)/(360/S))`, i.e. contract C-5
   on the **directed** bearing.
3. **Prior** (`models.prior`): `p0 = λ = ρ_T·Δ²/10⁴`. `1 − e^{−λ}` is a switch.
   - 5 /ha → 1.25e-4.
   - The difference is 7.8e-9, labelled (memo S-7).
4. **Per-ping cell evidence** (`grid.observe_ping`):
   - Samples inside the area are binned to cells. Per cell and ping: `PD_cell = max_i PD(r_i)`,
     the mean dB, and the water depth.
   - `n_pings += 1` once per (cell, ping).
   - **PD placeholder** (`models.pd_range`): 0.7 plateau on 3–25 m with 3 m ramps (brief §10).
     PD = 0 for rows flagged `altitude_unknown`.
5. **Looks** (`grid`):
   - A look of cell i = observations from one sector with gaps ≤ 20 s (`look.timeout_s`).
   - A new look starts on a sector change, a timeout, or a time reset.
   - Within a look the contribution is that of the **best** PD seen: same-look replacement,
     `ln Q += ln(1−ρPD_new) − ln(1−ρPD_old)`. Coverage is therefore live during a pass and equal
     to applying it at look close.
6. **Dependence factor** (`models.rho`):
   - `ρ = min over already-seen sectors s' of clip(Δ(s, s')/90°, 0, 1)`. The first look has ρ = 1.
   - Δ is between sector centres.
   - Q-10 options:
     - `fold_opposite: true`: Δ ∈ [0°, 90°] is the line angle (Paull's α);
     - `false`: the directed difference ∈ [0°, 180°], clipped.
7. **Aggregate miss** (`grid.apply_miss`). The brief's sequential form (default; Q-10):
   - `ℓ ← clip(ℓ + ln((1−ρPD)/(1−PFA)))`, with `PFA` = 0 by default: **the PFA = 0 special
     case, stated**;
   - `ln Q ← ln Q + ln(1−ρPD)`;
   - `coverage = 1 − Q`.

   Option `paull_pairwise`: `Q_new = (1 − c_hi)(1 − ρ·c_lo)` with
   `c_hi = max(1 − Q, PD)` and `c_lo = min(1 − Q, PD)`.
8. **Per-sector miss** (`grid`; brief §8.1 minimal fix; Q-3):
   `aspect_log_miss[s] = ln(1 − max PD over looks in sector s)`. That is Paull's α = 0 rule,
   `c_total = max`. Published, decides nothing.
9. **`p_background`** = `σ(ℓ)`, clamped to [1e-5, 0.999]. `NaN` outside the area.
10. **Contacts** (`contacts.py`):
    - **Association.** A detection, transformed to the map frame, joins the contact that
      minimises Mahalanobis distance among those with **both** Euclidean distance ≤ 2 m **and**
      `d²_M ≤ 9.21` on `Σ_obs + Σ_c`. Ties go to the lower uid. Otherwise it creates a contact
      with a new uid (monotonic, never reused).
    - **Existence** (brief §6.5):
      - new contact: `L = logit q`;
      - detected look: `L += ρ(logit q − logit π)` (`π` = 0.5), replaced within the look by the
        best q;
      - covered look that closes **without** an associated detection while the contact lay in
        the look's observed cells: `L += ln((1−ρ·PD_c)/(1−PFA))`;
      - `L` is clamped.
      - Look closure follows the release rule of §2.3, so a contact's look closes only when all
        of its detections are in.
    - **Position.**
      - Information form over **one observation per look** (the best-q detection).
      - The common-mode part (`contact.common_mode_sigma_m` = 1.0 m, Perception's σ_nav) is
        removed from each observation's covariance before fusion and added back once.
      - `P = Λ⁻¹ + (σ_cm² + σ_floor² + σ_align²)·I`, where `σ_floor` = 0.5 m.
      - A covariance that is not finite, symmetric, PSD and of positive trace falls back to the
        propagation formula of brief §6.6.
    - **Class.** Dirichlet counts `α_c += q` per detected look (best q), with α₀ = 1. Unknown
      names go to `other`.
    - **State.** Confirmed when there are ≥ 2 detected looks; `confirm_distinct_sectors` is
      false by default, since the brief says "preferably". A resolvable shadow would also
      confirm, but there is no source yet (meta request).
    - **Pruning.** A candidate with `p < 1e-3` is pruned, and the table is capped at 500
      (lowest p first). Uids are retired.
    - `first_confidence`, `first_sector` and `max_confidence` are stored (contract C-4).
11. **Rendering** (`belief.render`): `p_target,i = 1 − (1 − p_bg,i)·∏_c(1 − p_c·w_c,i)`.
    - `w_c,i` = the contact's Gaussian mass in cell i, as pdf(cell centre)·Δ² over the cells
      within 4σ.
    - The independence of background and contact is an approximation (brief §17.8), labelled.
12. **Disc query** (`belief.query_disc`, `QueryBelief`):
    - `p_any = 1 − ∏_{i∈disc}(1 − p_bg,i)·∏_c(1 − p_c·m_c)`, where `m_c` = the contact's mass
      **inside the disc**, never taken from the rendered map (v1 bug 2);
    - expected count `= Σ p_bg + Σ p_c m_c`;
    - also returned: coverage mean and minimum, observed and total cells, min and max
      `n_aspects`, class distribution, contacts inside.

**Removed or deferred.**
- Orientation-marginalised model (brief §8.1 "later"): dropped until E3 data exist.
- Dempster–Shafer: future work (brief §6.5).
- Confusion-matrix class update: waits for D3 to supply one.

---

## 4. Survey area (D-4): a file format first, a tool later

`SURVEY_AREA_FORMAT.md` will specify it. Belief accepts either kind of file:

```yaml
# (a) a simulator world's metadata.yaml   format: blueboat_world_meta/1   (read-only; only these keys)
# (b) a survey-area file, the same keys:  format: blueboat_survey_area/1
format: blueboat_survey_area/1
name: harbour_east_2026_11
geo_anchor: {lat0: 33.8812345, lon0: 130.8812345, theta_deg: 0.0}   # = the map datum
limits:
  local: {x_min: -40.0, y_min: -25.0, x_max: 60.0, y_max: 35.0}       # metres, ENU about geo_anchor
  corners_gps: [[lat, lon] SW, [lat, lon] SE, [lat, lon] NE, [lat, lon] NW]
```

**Validation rules.**
- `theta_deg == 0`, as SSS-Sim requires.
- All values finite.
- `corners_gps` converted through `core/geo.py` must agree with `limits.local` within 0.05 m. On
  disagreement the file is refused, naming the corner.
- The polygon is simple, with area ≥ 1 m².
- A world `metadata.yaml` is read for `format`, `geo_anchor` and `limits` **only**; `objects` are
  ground truth and are never read online (brief §13.4).

**Future tool.**
- The future configuration tool writes form (b) from a drawn rectangle or a designer pattern.
- An optional `polygon_gps` (≥ 3 vertices) can be added later as an additive key. Readers then
  prefer it and keep `limits` as the bounding box.

**Grid geometry.** The grid is the area's bounding box at `grid.resolution_m` (0.5 m).
- Origin = the lower-left corner of cell (0, 0) in the map frame.
- Row 0 is the south edge, column 0 the west edge.
- Cells whose centre is outside the polygon are NaN (C-3).

---

## 5. Message definitions (proposed, conforming to `belief_grid/0`)

`blueboat_belief_interfaces/msg/BeliefGrid.msg`

```
# BlueBoat-Belief snapshot, contract belief_grid/0. One self-consistent instant.
# Layout: cell i = row*width + col; row 0 = south edge, col 0 = west edge.
# Probability channels are in [0,1] and NaN outside the survey area (only there).
# Per-sector arrays: index cell*aspect_sector_count + s.
std_msgs/Header header                    # stamp = snapshot time (sonar-profile clock); frame_id "map"
string contract_version                   # "belief_grid/0"
uint32 grid_seq                           # +1 per published snapshot
uint32 width
uint32 height
float32 resolution                        # m per cell
float64 origin_x                          # map-frame lower-left corner of cell (0,0) [m]
float64 origin_y
float64 datum_lat                         # WGS-84 of map (0,0) [deg]
float64 datum_lon
float64 odom_to_map_x                     # x_map = R(yaw)·x_odom + t  [m]
float64 odom_to_map_y
float64 odom_to_map_yaw                   # rad, CCW (0: odom yaw is absolute ENU)
float32 odom_to_map_sigma_m               # 1-sigma of the translation; NaN = unknown
bool odom_to_map_valid                    # false until converged: do not plan
uint8 aspect_sector_count                 # S
float32 aspect_sector_origin_deg          # lower edge of sector 0, deg CCW from East, directed look bearing
string[] class_names
float32 prior_density_per_ha
string model_params_json                  # {"schema": 1, ...} informational
float32[] p_background
float32[] p_target
float32[] coverage                        # 1 - Q (modelled cumulative detection probability)
uint16[] n_looks                          # alias of synthesis n_observations
uint32[] n_pings
uint8[] n_aspects
float32[] depth_m
float32[] last_intensity_db
float32[] max_confidence
uint8[] aspect_counts                     # width*height*S, saturating at 255 (synthesis aspect_coverage)
float32[] aspect_log_miss                 # width*height*S, ln Q per sector; may be empty
float64[] area_x                          # [ADDED] survey polygon vertices, map frame [m]
float64[] area_y                          # [ADDED]
BeliefContact[] contacts
```

`blueboat_belief_interfaces/msg/BeliefContact.msg`

```
uint8 STATE_CANDIDATE=0
uint8 STATE_CONFIRMED=1
uint8 SECTOR_UNKNOWN=255
uint32 uid                                # never reused
float64 x                                 # map frame [m]
float64 y
float64[4] covariance                     # row-major [xx, xy, yx, yy] m^2
float32 p_exists
uint8 state
float32[] class_probabilities             # aligned with BeliefGrid.class_names
uint8[] sectors_detected                  # unique, sorted
uint8[] sectors_missed                    # unique, sorted (sector-centre convention)
uint16 n_looks_detected
float32 first_confidence                  # score of the creating detection; NaN unknown
uint8 first_sector                        # 255 unknown
float32 max_confidence
bool shadow_known                         # false: has_shadow meaningless
bool has_shadow
builtin_interfaces/Time created
builtin_interfaces/Time updated
float32[] look_bearings_deg               # optional look history, one entry per look
float32[] look_ranges_m
bool[] look_detected                      # [ADDED] aligns the two arrays above
string[] detection_ids                    # "<array stamp ns>:<id>" (ids repeat across GCS streams)
```

`blueboat_belief_interfaces/srv/QueryBelief.srv`

```
float64 x                                 # disc centre, map frame [m]
float64 y
float64 radius_m
---
bool ok
string error
uint32 grid_seq
float32 p_any                             # exact per-contact disc mass (MATH §12)
float32 expected_count
float32 coverage_mean
float32 coverage_min
uint32 observed_cells
uint32 total_cells
uint8 min_aspects
uint8 max_aspects
string[] class_names
float32[] class_distribution
uint32[] contact_uids
```

Display topics use standard types:
- `nav_msgs/OccupancyGrid`. `coverage` is mapped 0–100; `p_target` on a documented log scale,
  because values are ≈ 1e-4; −1 outside the area or unseen. The data is filled with `int8`
  (v1 bug 4).
- `visualization_msgs/MarkerArray`: 2σ ellipses and labels.
- `diagnostic_msgs/DiagnosticArray`.

---

## 6. Configuration (`config/belief.yaml`)

- Validated dataclasses; unknown keys raise by name (the SSS-Sim `_strict` pattern,
  `sonar/raycast/config.py:16-22`).
- Flat ROS parameters `section.key`.
- Optional floats are typed with NaN = unset (v1 bug 5).

| Key | Default | Source / status |
|---|---|---|
| `grid.resolution_m` | 0.5 | brief §10 (engineering) |
| `aspect.sector_count`, `aspect.origin_deg` | 12, 0.0 | brief §10 (Fawcett 2010 increments; discretisation choice, brief §17.4) |
| `look.timeout_s`, `look.time_reset_s` | 20, 5 | brief §10 (engineering); the processor's 5 s epoch rule |
| `prior.density_per_ha`, `prior.exact_poisson` | 5, false | brief §10 (site-dependent placeholder) |
| `detector.base_rate`, `detector.calibrated` | 0.5, false | brief §10; memo S-2 (no flag on the wire) |
| `detector.class_names`, `detector.ignore_classes` | brief-contract 5 + `other`; [`dummy_center`] | contract C-3; memo §2.4 |
| `clamp.p_min`, `clamp.p_max` | 1e-5, 0.999 | brief §10 |
| `pd.plateau`, `pd.r_lo_m`, `pd.r_hi_m`, `pd.ramp_m`, `pd.pfa` | 0.7, 3, 25, 3, 0.0 | **placeholder** (brief §10); PFA neglected (memo S-1) |
| `rho.independence_angle_deg`, `rho.fold_opposite`, `rho.ordering` | 90, Q-10, `sequential` | Paull 2013 (**placeholder** until E3) |
| `association.gate_m`, `association.chi2` | 2.0, 9.21 | contact criterion; 99 % χ²(2) |
| `contact.position_floor_sigma_m`, `contact.common_mode_sigma_m` | 0.5, 1.0 | brief §10; Perception σ_nav (assumed) |
| `contact.prune_p`, `contact.max_contacts`, `contact.confirm_looks`, `contact.confirm_distinct_sectors` | 1e-3, 500, 2, false | brief §6.5 (project decision) |
| `pose.stamp_tolerance_s`, `pose.frozen_count` | 0.25, 20 | brief §10, §7 |
| `alignment.window`, `.min_pairs`, `.max_spread_m`, `.latch`, `.origin_lat`, `.origin_lon`, `.override_sigma_m`, `.queue_max` | 200, 10, 3.0, true, NaN, NaN, 1.0, 6000 | engineering; D-3 |
| `coverage.require_detector`, `.max_array_gap_s`, `.wait_s` | true, 30, 60 | memo S-3 (Q-7) |
| `area.file` | `""` (required) | D-4 |
| `publish.grid_rate_hz`, `publish.display_rate_hz`, `snapshot.period_s`, `data_dir` | 1.0, 0.5, 60, `""` | contract C-2; brief §4.2 |

---

## 7. Gates and tests (Q-6)

The gate is `python3 -m test.smoke_test`, run from `blueboat_belief/` without ROS. It uses the
SSS-Sim style:
- `check()` prints `[PASS]`/`[FAIL]` and fails fast with exit 1;
- `[N]` section headers;
- the run ends with `ALL CHECKS PASSED`;
- a visible `SKIP` when an optional dependency is absent.

| Section | What it shows (brief §11) |
|---|---|
| [0] | Import sweep: `core/`, `io/`, `ros/adapters.py`, `cli/` import with rclpy, `*_msgs`, Qt and cv2 blocked on `sys.meta_path` (the Perception `conftest.py:37-82` pattern) |
| [1] | Config: defaults, unknown key raises, flat parameter round-trip. Geo: the pair against a verbatim reference copy, and against `BlueBoat-MCS/mcs/core/geo.py` when present beside (read as text, never imported; SKIP otherwise) |
| [2] | Survey area: world metadata and survey-file forms, every refusal path, polygon mask |
| [3] | Alignment: convergence, spread, latch, override, zero-fix rules, queue/replay order (v1 bug 1) |
| [4] | Pose buffer interpolation; frozen-pose detection; zero-stamp handling |
| [5] | Geometry: sample positions equal the processor formula; sector formula on boundary cases; one-sided rows; the unknown-altitude sentinel |
| [6] | Closed forms:<br>- one look: `coverage = PD` and the exact Bourgault value;<br>- same-sector repeat: +1 look, no information;<br>- orthogonal look doubles `ln Q`;<br>- 45° look matches Paull Eq. 24 (sequential form, and pairwise when selected);<br>- per-sector max rule;<br>- PFA term |
| [7] | Contacts:<br>- log-odds after detected and missed looks match their formulas;<br>- same-look replacement;<br>- miss applied once per closed look;<br>- association with tight and broad covariances (the 2 m gate against χ²);<br>- two nearby contacts, duplicates, ids repeated across streams;<br>- uids never reused;<br>- pruning and cap |
| [8] | Rendering against brute force; disc query uses per-contact mass (counter-example to the rendered-map product, v1 bug 2) |
| [9] | `to_grid_dict()` has every C-3/C-4 name with the contract dtype; NaN rules |
| [10] | `.msg` files parsed with `rosbags` (ROS-free) against the C-3/C-4 dtypes; CDR round-trip of what the adapters write. Standard types (Detection2DArray, NavSatFix, Odometry) and `ProcessedSSSPing` (reference copy) are parsed with real layouts. SKIP if `rosbags` is absent (installed in the cloud session) |
| [11] | Adapters: stub messages by attribute access; zero-stamp handling; NaN covariance |
| [12] | Determinism: the same event sequence twice gives byte-identical snapshot `.npz`; insensitive to dict/set order |
| [13] | Localisation: synthetic objects from one pass are within 2 m (the T12 criterion) |
| [14] | Rejections: each reason in §2.1 is triggered once and counted; no exception escapes |
| [15] | Performance: µs per swath at 0.5 m with 2 × 1199 samples; snapshot encode time. Cloud numbers are indicative only |
| [16] | Snapshot codec: write (O_EXCL, `-2` on collision), read back equal, manifest keys |

**Synthetic data** (Q-4): in-code scenarios inside `test/`, seeded, never reading any
`ground_truth/*` or `/sim/*` topic.

---

## 8. Module `CLAUDE.md` outline (< 200 lines, the Perception pattern)

1. **Layout**: the two packages; core / io / ros / cli.
2. **Interfaces**: the input and output tables (pointer to `docs/topics.md`); QoS; the clock rule.
3. **Non-negotiables**: brief §13 items 1–12, plus:
   - D-1 (the brief wins on design) with the CM-9 deviation stated;
   - no ground truth online;
   - `MATH.md` = spec.
4. **Model**: one paragraph plus a pointer to `MATH.md`; what is established and what is assumed.
5. **Commands**: build (`colcon build --packages-up-to blueboat_belief`), launch, offline replay, smoke.
6. **Gates**: `python3 -m test.smoke_test` (count), shell pitfalls (`/usr/bin/python3`, no sourcing).
7. **Status**: updated every session.
8. **Where documented**: `docs/*`, `TODO.md`, `LOCAL_CHECKLIST.md`.

---

## 9. Brief [ANALYSE] items: keep, change, drop

| Brief item | Verdict | Why |
|---|---|---|
| §3.1 pose error at 1 m/s | **drop the concern** | The pose is interpolated now; ~25 ms ≈ 2.5 cm at 1 m/s, far below the 0.5 m cell |
| §3.2 what detections must carry | **changed** | Stamp and covariance are provided by Perception. The score is uncalibrated, with no flag (config). Shadow is absent (meta request) |
| §3.3 map frame = anchor ENU | **keep, change the source** | Datum = the survey area's `geo_anchor` (Q-2) |
| §4.2 output set | **keep** | D-1 (the brief wins); `/diagnostics` is new to the project, additive |
| §4.3 channels | **keep, plus `aspect_log_miss`** | Synthesis names kept as aliases: `n_observations` = `n_looks`, `aspect_coverage` = `aspect_counts`/`n_aspects`, `last_intensity` = `last_intensity_db` |
| §5.1 replanner expectations | **keep** | All are in the contract; conformance in `CONTRACT_PROVIDED.md` |
| §8.1 per-sector misses / marginalised model | **keep minimal / drop later model** | Cheap and decides nothing; there are no E3 data for the later model |
| §8.2 everything in BeliefGrid | **keep** | Plus the survey polygon (`area_x/y`, ADDED) |
| §8.3 detection meta channel | **keep as REQUEST** | Belief works without it (§2.3) |
| §8.4 datum and limits | **keep** | D-4 |
| §8.5 fitted PD before E3 | **keep separation** | Planner-model and world-model parameters stay in separate files; circularity stated |
| §8.6 live pathologies | **keep** | §2.1 counters; frozen pose; one-sided rows; bursts |
| §9 structure | **keep** | 12 core modules, close to the brief's "~10" |
| §12.1 meta minimal fields | **change** | Image key, row span, status (ran / no_pixels), calibrated, dummy, shadow, side, ground range |

---

## 10. Scientific-model table (prompt F)

| Quantity / event | Conditioning | Evidence / source | Assumptions | Validation data available | Unresolved |
|---|---|---|---|---|---|
| Target prior `p0` | cell area, site density | project placeholder; Poisson rare-event approximation (brief §17.8) | independent cells; ≤ 1 target per cell | none (site-dependent) | density per site |
| `PD(r)` | ground range only | **placeholder** (brief §10). Precedents: Yordanova & Gips 2020 (SAS, primary read), Nguyen & Murdoch 2012 (not read here) | range-only; aspect, class, seabed omitted | sim ground truth (`mission_metrics`, circular); real labelled: none | fitted model (E3/E4) |
| `PFA` | per look, per cell | neglected: Bourgault 2006 (notes) | 0 (stated) | none | is it material at confidence ≥ 0.10? |
| `q` (score) | candidate box, class | Perception: raw, `calibrated: false` | used as a posterior only if config says calibrated | held-out sim worlds, planned (synthesis §8.2) | event definition, π |
| `ρ(Δ)` | angle between looks | Paull 2013 (notes; assumption citing Fawcett) | linear; fold or not (Q-10); min over seen sectors | none (E3) | ordering (Q-10) |
| `Q`, coverage | looks, PD, ρ | Bourgault 2006 (notes); Yordanova & Gips 2020 Eq. 2 (primary) | conditional independence modulo ρ; PFA = 0 | sim runs (model-conditional) | — |
| `Q_s` per sector | same, per sector | Paull α = 0 ⇒ max | full same-sector dependence | none | how the replanner uses it |
| `p_background` | misses only | Bourgault; Thrun et al. 2005 (log-odds, clamping) | independent cells (Thrun 2003 caveat) | sim | — |
| Contact `p_exists` | association, looks | Bernoulli-inspired (Mahler 2007), not an RFS filter | nearest neighbour, sparse scene, no clutter model | sim | clutter density (brief §17.11.6) |
| Position covariance | detection, common mode, alignment | Perception `geometric_v1` (assumed sigmas) | common mode = σ_nav; floor 0.5 m | sim ellipse coverage; USBL later | σ values |
| Class distribution | q-weighted counts | engineering (Dirichlet smoothing) | no confusion matrix | none | D3 confusion matrix |
| Rendering union | background × contacts | brief §6.6 | background independent of contacts (approximation) | — | — |
| Disc query | mass in disc | brief §6.6 (exact per-contact mass) | Gaussian contacts | closed-form tests | — |

---

## 11. Questions for the owner (recommendations first)

| # | Question | Recommendation | What changes with each option |
|---|---|---|---|
| Q-1 | Publication | TRANSIENT_LOCAL depth 1 **and** 1 Hz periodic (contract C-2) | Periodic-only VOLATILE: late joiners wait up to 1 s, and the contract text changes |
| Q-2 | Datum | **The survey area's `geo_anchor`.** In a built world this is the world anchor, which is the odom origin, so `t ≈ 0` | Mission designer `geo_anchor`: needs a second file, and in a built world differs from odom by `design_offset`. Either works for the consumer, which converts via the datum |
| Q-3 | Per-sector miss accounting now | **Yes** (`aspect_log_miss`, 12 floats per cell) | No: the replanner cannot compute aspect-resolved risk (brief §8.1) |
| Q-4 | Synthetic detections | **In-code scenarios in `test/` only.** A ground-truth-driven oracle belongs to the Replanner's `eval/` or to D7 | Here: Belief would own a ground-truth reader, against brief §13.4 |
| Q-5 | Meta companion | **A REQUEST to Perception** (fields in §9); Belief works without it | No request: the coverage gating stays a heuristic |
| Q-6 | Gate style | **`test/smoke_test.py` only** (brief §11; SSS-Sim style) | pytest plus smoke: two runners, more files |
| Q-7 | Negative evidence without a detector | **Gated on the coverage timeline (§2.3)**; geometry-only otherwise; `require_detector: false` for development only | Ungated: misses credited where no detector ran (brief §17.9 violation) |
| Q-8 | Uncalibrated scores | **Use the score as q, with `detector.calibrated: false` published** in `model_params_json`; `p_exists` documented as "not a probability until calibrated" | Fixed q per detection: hides score information; a refusal mode leaves no contacts until calibration |
| Q-9 | Dummy detector | **Ignore class `dummy_center`** (counted), and treat its arrays as no detector coverage | As-is: a contact chain along the nadir and fake misses |
| Q-10 | Paull details | **`fold_opposite: true`** (Paull's line angle) with **`ordering: sequential`** (the brief's form; `paull_pairwise` available) | Directed Δ: opposite-side looks become independent. Pairwise: Paull-faithful when a later look has the higher PD |
| Q-11 | Unknown-altitude rows (`water_depth == 0.22`) | **Skip their evidence** (counted) | Credit with slant geometry: up to ~h misplacement near nadir |
| Q-12 | Recording Belief's inputs on real runs | **A launch argument `record_inputs:=true`** that starts `ros2 bag record` of the four topics into Belief's data folder (O_EXCL) | Leave it to the operator: CM-14 offline iteration depends on remembering it |
| Q-13 | Survey polygon in the message | **Yes** (`area_x/y`, ADDED, ignored by a consumer that does not need it) | No: the replanner recovers the area only from NaN cells |
| Q-14 | Alignment after convergence | **Latch** | Track: the map shifts under cells already written |

---

## 12. Owner answers (2026-11-04)

| # | Answer |
|---|---|
| Q-7 | Gate negative evidence on the detection-array coverage timeline (recommended) |
| Q-8 | Use the score as q and publish `detector_calibrated: false`. **Calibration is a stated priority**: listed at the top of `README.md`, `TODO.md` [P0], `DESIGN.md` and `topics.md` |
| Q-10 | `fold_opposite: true`, `ordering: sequential`; `paull_pairwise` available as a switch, with both closed forms tested |
| Q-1 … Q-6, Q-9, Q-11 … Q-14 | All recommendations accepted as written in §11 |

## 13. Implementation notes: where the build differs from §1–§11 (2026-11-04)

| Item | Design text | As built | Why |
|---|---|---|---|
| Survey polygon | from `corners_gps` | `limits.local` exactly; `corners_gps` is the 0.05 m cross-check | GPS round trip put the grid origin 1e-9 m off its value |
| Sector formula | `floor(mod(deg(b) − o, 360)/(360/S))` | the angle is rounded to 1e-9° first | `degrees(radians(30))` = 29.999999999999996 put edges in the lower sector |
| Contact cap | at pruning | also at creation | the table could exceed the cap between look closures |
| `stale_detection` | listed as a reason | `t_row` older than newest ping − (`coverage.wait_s` + `look.timeout_s`) | defined at build |
| `test/msg_reference/` | a `ProcessedSSSPing.msg` copy | `<pkg>/msg/<Name>.msg`: `blueboat_interfaces` (verbatim from Control `1fadbb2`) + `vision_msgs` (verbatim from upstream `ros2` branch) | the rosbags Jazzy store has no `vision_msgs`; one layout serves the smoke test and `blueboat-belief-replay --msg-dir` |
| Adapters | duck-typed | plus `_seq()`: never truth-test a sequence field | the real layouts (numpy arrays) broke `x or default` |
| Gate | sections [0]–[16] | plus [17] packaging, [18] mcap bag replay (= direct feeding, byte-identical) | the replay CLI needed its own proof |

## 14. Revision after the primary sources (owner, 2026-11-04)

| # | Change | Why |
|---|---|---|
| Q-10 revised | `rho.ordering` default **`paull_pairwise`** (was `sequential`) | Paull §III-C orders c¹ ≥ c². Sequential under-credits a stronger later look |
| new | Turn back-scan gating (`turn.*`, `core/turn.py`) | Paull §II-C: the inner side of a turn is distorted for ATR, and fake aspects came from one turn |
| new | `summary.area_cumulative_pd` in `/diagnostics` and manifests | Bourgault Eq. 7 for one target uniform in the area |
| new | `blueboat-belief-make-area` | a minimal survey-area tool (D-4) |
| deferred | bearing-resolved ρ | Bourgault §4.3 smoothness. It needs a contract addition, so it is settled with the Replanner |
