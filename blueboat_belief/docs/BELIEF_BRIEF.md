# BlueBoat-Belief (D4) — Build brief for Claude Code

*Brief dated 2026-10-09. Audience: the local Claude Code session that will build the Belief Module in
`~/ros2_ws/src/BlueBoat-SideScanSonar/`. It condenses the project context, the science, the lessons of a
first implementation ("v1"), the constraints of the superproject, and the open questions.*

**How to read this brief.** It gives directions, not a finished design. Everything marked
**[ANALYSE]** is a proposal you must check against the code and the documents before adopting.
Everything marked **[ASK]** needs the user's decision. Everything marked **[VERIFY]** is a fact that
must be checked in the local repository before you rely on it.

**Authority order** (same as the superproject):
1. `project_synthesis.md` — scope, scientific argument, ⚠ design rules. Single source of truth.
2. The superproject root `.claude/CLAUDE.md` and its skills (`file-interfaces`, `module-gates`, `svlog`):
   cross-module rules CM-1…CM-16 and the interface contract.
3. Each neighbour module's `.claude/CLAUDE.md` for that module's internals.
4. This brief.

Where this brief disagrees with (1)–(3), (1)–(3) win and you report the disagreement.

---

## 1. What the module is for

**One sentence.** Maintain, from every processed sonar ping and every detection, a recursive
probabilistic estimate of the survey area: where targets probably are, where the seabed has really been
searched, and **from which aspects**. Publish that estimate as the **only** input of the replanner (CM-9).

**Deliverable and timeline.** D4 of the thesis plan (task T12, planned 26 Oct – 4 Nov 2026). Done-criteria
from the plan and the experiment discussions:
- belief updates **deterministic** under repeated replay of the same recording;
- known objects localised **within 2 m** from a single replayed pass;
- an **aspect-deficit** answer available to the replanner.

**Why it matters scientifically** (synthesis §2). A surface vehicle cannot choose its sonar altitude,
so in shallow water shadows are starved and detection relies on the strongly aspect-dependent specular
highlight. One pass can miss a present target seen from an uninformative heading. The thesis asks
whether aspect-aware replanning recovers targets that a tuned two-pass orthogonal survey (CM-12) misses.
The replanner can only make that decision if the belief knows coverage **and aspect history**.

**What the module is NOT.**
- Not a detector: it does not look at pixels.
- Not a planner: no path or "next best view" logic. It exposes what a planner needs; nothing more.
- Not a mosaic: the GCS mosaic is operator display only.
- Not a ground-truth evaluator: it never reads simulation ground truth online (CM-2, file 02 §5.3).

---

## 2. Project status that constrains the build (2026-10-09)

| Item | State | Consequence for the belief |
|---|---|---|
| D1 SSS pipeline (`sss_processor_node`) | Built, field-used | Main input `/sss_processor/processed` exists in real and sim |
| D2 beach dataset | Not started (critical path) | No real detections exist |
| D3 detector | Not implemented, unowned | Detection input must be designed against the existing `/sss_ai/detections` contract; development needs synthetic detections |
| **D4 belief** | **Not implemented, unowned** (a v1 was written outside the repo, see §7) | You build it |
| D5 mission interface | Base built; "Update Mission" between runs; no mid-mission swap | Not your concern; the replanner consumes your output |
| D6 replanner | Not implemented | Your downstream consumer; design the output for it |
| D7 evaluation | Building blocks only (`mission_metrics`) | Write belief snapshots it can read offline |
| S1 simulator | Built and calibrated; aspect response predicted by facet physics, **not yet field-measured** | Sim runs are the integration environment; aspect models stay placeholders |
| Control / MCS | v1.0 frozen, **v1.1 imminent** | Anything needed there is a **v1.1 request**, never an edit |
| `/blueboat/odom` live | Reads zero / other clock in real runs (open) | Robustness: detect frozen pose, never crash |
| Transducer lever arm | sim 0.20 m lateral, processor 0.0 (unmeasured) | Do not add your own lever arm on top of the processor's geometry |

Superproject rules you must follow: sessions **never `git commit` or `git push`** (the user commits);
**no edits to any other module** (CM-3); every new module follows file 02 §5.

---

## 3. Inputs

All inputs exist today and are identical in real and simulation (CM-2). Names and types are a contract
(CM-1): never rename or retype.

| Input | Type | QoS of the publisher | Use |
|---|---|---|---|
| `/sss_processor/processed` | `blueboat_interfaces/ProcessedSSSPing` | **BEST_EFFORT**, KEEP_LAST (subscribe BEST_EFFORT, depth ≥ 50) | Swath footprint of every ping: the source of **negative evidence**, coverage, aspect history, depth, intensity; and the pose clock |
| `/sss_ai/detections` | `vision_msgs/Detection2DArray` | default (reliable) | Detections. Today nobody publishes; the GCS already subscribes |
| `/blueboat/odom` | `nav_msgs/Odometry` | default (reliable) | Pose fallback and covariance |
| `/mavros/global_position/global` | `sensor_msgs/NavSatFix` | **BEST_EFFORT** | Odom → map anchoring on the real boat |
| Mission geo context (files, read-only) | `blueboat_trajectory/1` YAML (has `geo_anchor`); world `metadata.yaml` (`blueboat_world_meta/1`) | — | Datum and, optionally, survey limits. Never read `builder_state.yaml` |

### 3.1 `ProcessedSSSPing` facts and traps

```
builtin_interfaces/Time port_stamp, starboard_stamp
uint32 port_ping_number, starboard_ping_number   # 0 = side absent (one-sided rows are legal, CM-6)
float32 robot_x, robot_y                         # odom frame, metres
geometry_msgs/Quaternion robot_orientation
float32 water_depth, transducer_x_offset
float32[] port_intensity_db, port_y, starboard_intensity_db, starboard_y   # ground offsets; +y = port
```
- One-sided rows happen (per-device counter offsets, drops). Handle each side independently.
- The pose is the **nearest odom sample** (5 s buffer, 1 s tolerance) — usually ~25 ms off at 20 Hz odom,
  worse when odom stalls. **[ANALYSE]** whether that error matters at 1 m/s.
- Water column is already removed; `*_y` are ground-range offsets in base_link with the processor's
  transducer offsets folded in (currently 0.0). Do not re-apply a lever arm.
- Sim stamps are sim time: use `use_sim_time` under Gazebo; judge "stream stopped" by **wall clock vs
  last arrival**.
- Range setting: implemented default 20 m (real = sim); synthesis §8.5 reserves 30 m coverage / 15 m
  revisit for the experiments. Useful detection band 5–20 m.

### 3.2 `/sss_ai/detections` contract (existing; GCS `detections_listener.py`)

- `results[0].hypothesis.class_id`, `.score`;
- `bbox.center.position.x/.y` = position **in the local odom frame [m]**;
- `bbox.size_x` = extent [m];
- `detections[i].id` = **stable uid**: re-publishing the same id updates rather than double-counts.

What the belief additionally needs, and where it can travel without a new type (**[ANALYSE]**):
- **Acquisition time** → each `Detection2D.header.stamp` must be the ping time of the detection's centre
  row, not inference time. The belief looks up the pose at that stamp.
- **Position covariance** → `results[0].pose.covariance` (6×6, x/y block; all-zero = unknown).
- **Calibrated** confidence → `score` (temperature scaling, synthesis §8.2). The belief assumes it.
- **Shadow flag, side, ground range** → not in Detection2D. Side and ground range can be **derived** by
  the belief from the pose at the stamp and the detection position. A resolvable shadow (contact
  criterion) needs an optional companion channel — see §12.1.

Treat every optional field as "unknown" when absent or zero. The belief must work with the bare GCS
contract.

### 3.3 Pose and frames

- World convention: **ENU**, internal yaw in radians CCW from east; compass degrees only at user-facing
  boundaries (`radians(90 − hdg)`).
- `/blueboat/odom`: **local ENU in both modes**. Real: origin = boat at launch, **yaw absolute ENU**
  (file 01 §3.1, marked VERIFIED). Sim: Gazebo ENU, the boat spawns at world (0,0) = the world's
  `geo_anchor`. **[VERIFY]** in `robot_interface.py`: a v1 reading of older code concluded the real
  heading was rotated by −yaw₀; today's context pack says absolute. This decides whether odom → map is a
  translation only.
- NavSatFix: `lat == 0 and lon == 0` = no fix; in sim the MCS synthesises it with `status = -2`, so
  **only `status == -1` means no fix**.
- No TF anywhere in the project. Do not introduce one.
- Geodesy: the project's single equirectangular pair, carried as **your own copy** (CM-3 forbids importing
  a neighbour), asserted **bit-identical** to `BlueBoat-MCS/mcs/core/geo.py`:
  `east = radians(lon − lon0)·R·cos(radians(lat0))`, `north = radians(lat − lat0)·R`, `R = 6378137.0`.

**[ANALYSE] map frame.** Recommended: the belief's map frame = local ENU about the **mission
`geo_anchor`** — the same frame as the MCS map scene and the simulator world. In sim odom = map
(identity). On the real boat, odom → map = ENU(launch position) relative to the anchor, obtained from
GPS/odom pairs (translation, plus a small rotation only if verification shows yaw is not absolute).

---

## 4. Outputs

### 4.1 Message types live in this module's own interfaces package (decided by the owner)

The module ships a sibling ament_cmake package **`blueboat_belief_interfaces`** in the BlueBoat-Belief
repository, which defines the belief's own messages and services. **This is the intended design, not a
bug and not a rule violation to work around.** (v1 already did this; it is kept.)

Why it is the better choice here:
- **Independence.** Control is frozen between releases (v1.0, v1.1 imminent). Putting the belief's types
  in `blueboat_interfaces` would tie every belief-message change to a Control release and make a frozen,
  field-tested module depend on an experimental one. With its own package, the belief versions its
  messages together with its code, and Control is never touched (CM-3 is satisfied by construction).
- **Clean dependency direction.** `blueboat_belief_interfaces` depends only on `std_msgs` /
  `builtin_interfaces`. Its consumers (replanner D6, evaluation D7, optional displays) depend on it, not
  on Control. `blueboat_belief` reads `blueboat_interfaces` only for `ProcessedSSSPing`.
- **Standard ROS practice.** One `*_interfaces` / `*_msgs` package per subsystem (as in Nav2, MAVROS)
  is how ROS 2 projects keep message ownership with the component that owns the semantics.
- **Typed beats JSON for this contract.** Build-time schema checking, a header for timing, efficient
  numeric arrays (no base64), rosbag introspection and `ros2 interface show` all come for free.

How this reads against CM-1: CM-1's purpose is that **existing** topics, services and parameters are never
renamed or retyped unilaterally, and that one place owns each type. Both still hold: each belief type is
defined in exactly one place (`blueboat_belief_interfaces`), and no existing topic changes type. The
sentence "`blueboat_interfaces` is the only place a message type is defined" was written when only one
interfaces package existed; the owner has decided that new modules may own their interfaces package.
The root `CLAUDE.md` wording of CM-1 should be updated accordingly by the user (§12.6) — **do not edit it
yourself**.

What still applies:
- never retype an existing topic: `/sss_ai/detections` stays `vision_msgs/Detection2DArray` (§3.2);
- prefer a standard type where it fits (OccupancyGrid, MarkerArray, DiagnosticArray);
- create a custom type only where there is a clear need (the full belief state, the contact, the query).

### 4.2 Recommended output set **[ANALYSE]**

| Output | Type | Content | Consumer |
|---|---|---|---|
| `~/belief_grid` (e.g. `/belief_map/belief_grid`) | `blueboat_belief_interfaces/BeliefGrid` | **One self-consistent snapshot**: header (stamp, map frame), sequence/version, grid geometry and datum, odom → map transform and its sigma, aspect-sector convention, model parameters in use (prior, PD model, ρ model, base rate), per-cell channel arrays, `BeliefContact[]` | Replanner (D6) — the only object it reads (CM-9) |
| `~/query_belief` | `blueboat_belief_interfaces/QueryBelief` (srv) | Disc query: `p_any`, expected count, coverage, observed cells, min/max distinct aspects, class distribution, contacts inside — computed with the exact per-contact mass of §6.6 | Replanner (optional convenience) |
| `~/p_target`, `~/coverage` | `nav_msgs/OccupancyGrid` | Display only, 0–100, −1 outside/unseen. `p_target` values are tiny (≈1e-4), so document a log mapping or display coverage only | RViz, MCS/GCS overlay (optional) |
| `~/contacts` | `visualization_msgs/MarkerArray` | 2σ ellipses + labels | RViz |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | OK/WARN/ERROR + counters per rejection reason | Operator |
| Snapshot files | `.npz` + manifest JSON under the data root (`O_EXCL`, never overwrite) | Full state at run end, and periodically | D7 evaluation, offline analysis |

Why one `BeliefGrid` message rather than several topics: the replanner needs geometry, channels and
contacts **from the same instant**. Size check: a beach basin of 100 × 60 m at 0.5 m = 24 000 cells;
8 float32 channels + 12 aspect bytes per cell ≈ 1 MB — fine at 0.2–1 Hz on localhost. Fill array fields
with numpy arrays of the exact field dtype. QoS: reliable; if latched (TRANSIENT_LOCAL), consumers must
subscribe TRANSIENT_LOCAL too (CM-4) — or publish periodically with default QoS. Decide and document.

**Everything needed to reproduce a prediction is in `BeliefGrid`** (separate background and contact
channels, model parameters, per-sector look history), so the replanner never needs to import the belief's
code (CM-3); the query service is a convenience, not a dependency.

### 4.3 Per-cell channels (minimum) **[ANALYSE]**

| Channel | Meaning |
|---|---|
| `p_background` | Probability that an *undetected* target is in the cell (lowered only by misses) |
| `p_target` | Rendered: background ⊕ contacts (§6.6) |
| `coverage` | 1 − Q, Q = cumulative miss probability over independent looks |
| `n_looks`, `n_pings` | Independent looks vs raw pings |
| `aspect_counts[S]` (or a bitmap) | Looks per aspect sector, S = 12 × 30° |
| `n_aspects` | Distinct sectors seen |
| `depth_m`, `last_intensity_db`, `max_confidence` | Last depth, last mean intensity, best detection confidence |

Names in synthesis §5.2 / file 02 §5.4: `p_target`, `n_observations`, `aspect_coverage`,
`last_intensity`, `max_confidence`. Keep those names visible (aliases or identical) so the documents and
the code agree.

### 4.4 Contact fields (minimum)

uid; x, y (map); 2×2 covariance; `p_exists`; state `candidate | confirmed`; class distribution;
detected and missed looks with their sectors and ranges; `has_shadow` (unknown allowed); created /
updated stamps; ids of the detections that formed it (traceability).

---

## 5. Connections (one picture)

```
 sss_processor_node ──/sss_processor/processed (BEST_EFFORT)──┐
 detector D3 (future) ──/sss_ai/detections (Detection2DArray)─┤
 robot_interface / Gazebo ──/blueboat/odom────────────────────┤──► belief node (BlueBoat-Belief)
 MAVROS ──/mavros/global_position/global (BEST_EFFORT)────────┤        │  ROS-free core
 MCS trajectory YAML / world metadata.yaml (files) ───────────┘        │
                                                                        ├─► ~/belief_grid (BeliefGrid) + ~/query_belief ──► replanner D6 (CM-9)
                                                                        ├─► ~/p_target, ~/coverage (OccupancyGrid) ──► RViz / MCS / GCS
                                                                        ├─► ~/contacts (MarkerArray), /diagnostics
                                                                        └─► snapshots (.npz + manifest) ──► evaluation D7
```
The GCS already shows `/sss_ai/detections` as markers; nothing changes there.

---

### 5.1 Downstream consumer: what the replanner expects (check in Phase 2)

BlueBoat-Replanner (D6) is built in parallel and reads only `BeliefGrid` (CM-9). Its brief assumes the
items below. Each is a *requirement to confirm or refute*, not a decision. Report gaps in `docs/REQUESTS.md`.

| Replanner expects | Belief status to settle |
|---|---|
| Topic `/belief_map/belief_grid`, reliable, possibly transient local | Final name and QoS; latched vs periodic (§16.1) |
| Datum + `odom → map` published in the message | Datum = `geo_anchor`; translation-only vs rotation (§3.3, §16.2–3) |
| `coverage` (= 1 − Q), `aspect_counts` (cell-major, S = 12), `n_aspects` | Layout, dtype, ordering; the 30° sector convention (sector 0 = East, `b = ψ ± π/2`, port = +) stated in `topics.md` |
| `p_target` NaN outside the polygon; `p_background` separate | Whether NaN or sentinel; publish `p_background` (§8.2) |
| Contacts with uid, x, y, 2×2 covariance, `p_exists`, state, class probabilities, detected/missed sector sets, `max_confidence`, `has_shadow` | Exact field names and what look history is stored (sectors only, or bearings/ranges too); uids never reused |
| Sample position / look bearing / sector formula reproducible from the message | Document so the replanner's vectorised look predictor can be tested against the Belief core (> 97 % agreement) |
| Importable core for the replanner's `eval/` geometry test | Package/import path of the ROS-free core; CM-3 allows or not (user decision) |

---

## 6. Scientific core (main ideas, with sources)

Keep the mathematics in a short `docs/MATH.md` that is the specification: each equation names the
function that implements it.

### 6.1 Representation: grid for negative evidence, contacts for positive evidence
- Occupancy grids with independent cells and recursive log-odds updates, with clamping: Elfes 1989;
  Thrun, Burgard & Fox 2005, ch. 9.
- A pure grid double-counts a poorly localised object seen twice and loses identity; a pure tracker
  cannot represent "searched and empty". Hence **hybrid**: misses, coverage and aspect history on the
  grid; positive evidence on **Bernoulli contacts** (existence probability + Gaussian position), the
  multi-Bernoulli idea of Mahler 2007.
- Cell prior from an expected target density: `p0 = ρ_T·Δ²/10⁴` (targets per hectare).

### 6.2 Count looks, not pings
Consecutive pings of one pass are nearly fully dependent; same-aspect looks are "nearly completely
dependent" (Myers & Williams 2012). A **look** = one pass of a cell from one aspect sector; a new look
starts when the sector changes or after a timeout (v1: 20 s), or when time goes backwards (bag restart).

### 6.3 Aspect dependence between looks (placeholder)
- Paull et al. 2013 (Eqs. 22–24): parallel tracks dependent, orthogonal tracks independent, linear in
  between: `c_tot = (2α/π)·c₂(1 − c₁) + c₁`.
- v1 implemented it as an independence factor `ρ = clip(Δbearing/90°, 0, 1)` against aspects already
  seen, and a dependent look acts with **effective detection probability ρ·PD**. That reproduces Paull's
  Eq. 24 exactly. The tempered form `(1−PD)^ρ` was rejected: by Bernoulli's inequality it credits a
  partially dependent look with *more* coverage (c = 0.7, ρ = 0.5 → 0.45 instead of 0.35).
- **This is an assumption.** Paull cite Fawcett 2010 for it. Nobody has measured the Δ-heading
  dependence in shallow water. E3 does.

### 6.4 Negative evidence (every new look of a cell)
`ℓ ← clip(ℓ + ln(1 − ρ·PD(r)))`, `ln Q ← ln Q + ln(1 − ρ·PD(r))`, `coverage = 1 − Q`.
For ρ = 1 this is Bourgault et al. 2006's miss update `p⁺ = p(1−PD)/(p(1−PD)+1−p)` in log-odds. `Q` is
Yordanova & Gips 2020's residual-risk map. PD(range) is a **placeholder** (v1: 0.7 plateau on 3–25 m
with 3 m ramps) until a fitted model exists.

### 6.5 Positive evidence: contacts
- Association: within **2 m** (project contact criterion) **and** a χ² gate (99 %, 2 dof = 9.21) on
  `Σ_obs + Σ_c`.
- Calibrated confidence q under base rate π carries `ln LR = logit q − logit π` (Guo et al. 2017 for
  calibration). New contact `L = logit q`; later detected look `L += ρ(logit q − logit π)`; missed look
  (its look closed without a detection) `L += ln(1 − ρ·PD)`. A better detection **within the same look
  replaces** that look's contribution, never accumulates.
- Position: information-form fusion, one observation per look, plus a covariance floor (common-mode
  error: alignment, tide, extent).
- Class: confidence-weighted counts with Dirichlet smoothing. A confusion matrix would make it a proper
  Bayes update, once D3 supplies one.
- State **confirmed** if a resolvable shadow is reported, **or** ≥ 2 detected looks at a consistent
  position (≤ 2 m), preferably from different aspects; otherwise **candidate** (adopted contact
  criterion). Prune below a small p; cap the table size.
- Dempster–Shafer instead of log-odds? With matched sensor models the Bayesian/Dempster difference on 2-D
  grids is small and its sign depends on the matching criterion (Berlenko & Krinkin 2026). Keep
  log-odds; DS class fusion (Fawcett 2010, Wiig 2012) is future work.

### 6.6 Spatial uncertainty, rendering, queries
- Detection covariance when the detector sends none:
  `Σ = Σ_pose + (J_ψ b)(J_ψ b)ᵀσ_ψ² + R(ψ) diag(σ_along², (σ_across + f·r)²) R(ψ)ᵀ`, then rotated into the
  map frame `+ σ_align² I`. Missing or all-zero odom covariance → configured sigmas.
- Rendering: `p_target,i = 1 − (1 − p_bg,i)·∏_c(1 − p_c·w_c,i)`, `w_c,i` = Gaussian mass in the cell.
- Any disc query must use each contact's **mass inside the disc** (`p_any = 1 − ∏(1−p_bg)·∏(1 − p_c·m_c)`),
  not a cell-by-cell product of the rendered map (that under-counts a spread contact). Keep it in the
  core (it backs `~/query_belief`) and state it in the docs.

### 6.7 What is established vs assumed (keep this distinction in the docs)

| Statement | Status |
|---|---|
| Bayesian miss update, log-odds grids, clamping | Established (Bourgault 2006; Thrun 2005) |
| Residual risk ∏(1 − PD) per cell | Established practice (Yordanova & Gips 2020) |
| Look dependence linear in angle | **Assumption** (Paull 2013, citing Fawcett 2010) — replace with E3 |
| PD(range) window | **Placeholder** — replace with a fitted model |
| Confidences calibrated | **Assumption** about D3 |
| Static seabed, independent cells, one object per contact | Modelling assumptions |
| Contact criterion (shadow or ≥ 2 consistent looks) | Project decision |

---

## 7. Lessons from v1 (an implementation written outside the repository)

A complete v1 exists (`BlueBoat-Belief` zip: ROS-free core, rclpy node, 73 ROS-free tests, docs, guide
PDF). It was built **before** the context pack of 2026-10-09. Its own interfaces package
(`blueboat_belief_interfaces`) is **correct and kept** (§4.1); its one contract problem is the custom type
it put on the existing `/sss_ai/detections` topic. Use it as a **reference and a test oracle**,
not as a template to copy. Decide per component: port with its tests, or rewrite.

**Keep (validated in v1):**
- hybrid grid + contacts; looks with timeout and sector change; effective-PD (Paull Eq. 24) miss form;
- same-look replacement; misses at contacts applied when the look *closes*, once per look;
- exact disc query (per-contact mass);
- covariance validation (finite, symmetric, PSD, positive trace) with fallback to propagation;
- rejection reasons counted (stale, future, no pose at stamp, frozen pose, outside polygon, NaN, bad
  covariance, alignment pending), never silent drops, never exceptions on bad sensor data;
- frozen-pose detection (≥ 20 poses at (0,0)) → diagnostics ERROR;
- determinism: bitwise-identical replay; no wall clock, no randomness, no set-order dependence in the core;
- vectorised grid update: 0.17–0.25 ms per swath at 0.5 m (600 samples/side). Budget < 1 ms;
- own interfaces package `blueboat_belief_interfaces` (BeliefGrid, BeliefContact, QueryBelief; the
  `SssDetection*` messages are dropped because detections arrive as `Detection2DArray`);
- ROS-free tests with stub messages, plus `rosbags` (pip, pure Python) to parse **real** message layouts
  and CDR round-trip what the adapters write. Still useful for standard types and a reference copy of
  `ProcessedSSSPing.msg`.

**Bugs found in v1 (do not reintroduce):**
1. Online alignment never converged: the ping pose was fed to the aligner *after* the readiness check.
   Feed first; queue (bounded) observations that arrive before alignment and replay them in order.
2. A disc query computed from the rendered map under-counted spread contacts.
3. The first miss form `ρ·ln(1−PD)` was optimistic vs Paull (§6.3).
4. rclpy rejects numpy arrays of the wrong dtype for sequence fields; fill message arrays with the exact
   dtype (`int8` for OccupancyGrid data).
5. Optional float parameters declared with a `None` default are fragile: declare them typed with NaN =
   "unset"; empty lists need dynamic typing.

**v1 decisions that are now wrong:**
- Custom `SssDetectionArray` on `/sss_ai/detections` → retypes an existing topic (CM-1). Consume
  `Detection2DArray`.
- Default datum = polygon centroid → use the mission `geo_anchor` (§3.3).
- Odom assumed rotated by −yaw₀ on the boat → **[VERIFY]**, likely absolute.

**Never run:** the v1 rclpy node never ran in a ROS graph. Do the first live run against the simulator
early.

---

## 8. Improvements to analyse **[ANALYSE]** (do not over-engineer)

1. **Aspect-agnostic coverage vs an aspect-centric thesis.** With PD independent of aspect,
   `coverage = 1 − Q` declares a cell "searched" after one strong look. In a v1 synthetic run, an object
   visible only from N/S **lost** belief after an E/W pass, because a miss from a blind aspect counted
   fully. A replanner that maximises coverage would therefore undervalue the very multi-aspect revisits
   the thesis studies.
   - **Minimal fix (recommended):** keep per-cell, per-sector miss accounting (look counts and ln Q per
     sector; 12 floats per cell). Publish it, so an aspect-resolved residual risk can be computed. Keep
     v1's aggregate update as the default. Cheap, exposes the right information, decides nothing early.
   - **Later, with E3 data:** an orientation-marginalised model. Hypotheses θ for target orientation;
     looks are conditionally independent given θ with PD(r)·A(b − θ); marginalise θ. Note it does *not*
     remove credit for same-aspect repeats by itself — real passes differ by 1.8–3.1 dB rms, so some
     pass-to-pass randomness is real. Needs E3 to fit A and that randomness. Put behind a model switch;
     do not build before data exists.
2. **Everything the replanner needs, in `BeliefGrid`** (§4.2): separate `p_background`; model parameters;
   per-contact look history with sectors and ranges; stable uids across pruning (never reuse a uid).
3. **Detection meta channel** for shadow (§12.1); belief derives side and range itself.
4. **Datum = geo_anchor**; survey limits from config (WGS-84 polygon) or the world/trajectory files.
5. **Fitted PD before E3.** The simulator's `mission_metrics --aspect-bin-deg` gives ground-truth
   visibility per aspect bin. A PD fitted from simulation and then evaluated in simulation is circular.
   Planner-model and world-model parameters must be separable configurations. State it in the docs.
6. **Robustness to the live pathologies**: odom zero / other clock, one-sided rows, ~8 % ping loss,
   detections arriving late.

---

## 9. Possible structure **[ANALYSE]**

File 02 §5.2 template (ROS node package; layout of `blueboat_sss_sim`):

```
BlueBoat-Belief/                        new repo, submodule at the superproject root (user adds and commits)
├── blueboat_belief_interfaces/         ament_cmake: msg/BeliefGrid, msg/BeliefContact, srv/QueryBelief
│                                       (+ detection meta if §12.1 is adopted); depends std_msgs, builtin_interfaces
├── .claude/
│   ├── CLAUDE.md                       file 02 §5.7 skeleton, < 200 lines
│   ├── TODO.md                         open items only, [P0]/[P1]/[P2]
│   └── rules/ (optional)               path-scoped rules, e.g. core/ = no ROS
└── blueboat_belief/                    ament_python package (one level down, like the simulator)
    ├── package.xml                     rclpy, std_msgs, nav_msgs, sensor_msgs, vision_msgs,
    │                                   visualization_msgs, diagnostic_msgs, blueboat_belief_interfaces,
    │                                   blueboat_interfaces (read only, ProcessedSSSPing)
    ├── setup.py / setup.cfg            console_scripts: belief_node, replay/snapshot CLIs
    ├── resource/blueboat_belief
    ├── blueboat_belief/
    │   ├── core/                       NumPy only: config, geo (bit-identical copy), polygon, uncertainty,
    │   │                               alignment, pose_buffer, models (PD, ρ), grid, contacts, belief (facade)
    │   ├── io/                         snapshot writer/reader (npz + manifest),
    │   │                               trajectory/world metadata readers
    │   └── ros/                        adapters (duck-typed, ROS-free) + the node (only rclpy file)
    ├── launch/belief_launch.py         simple_launch, imported inside try/except ImportError
    ├── config/*.yaml                   one idea per file; unknown keys raise by name
    ├── docs/                           MATH.md (spec = code), topics.md (interfaces), DESIGN.md (decisions)
    └── test/                           smoke_test.py runnable as `python3 -m test.smoke_test` WITHOUT ROS,
                                        + unit tests; msg_reference/ProcessedSSSPing.msg (verbatim copy)
```

Keep it as small as the requirements allow. The v1 core was ~10 modules; that is about the right size.

---

## 10. Configuration (starting values; every number has a source or is labelled placeholder)

| Key | Start value | Source |
|---|---|---|
| grid resolution | 0.5 m | ≈ across-track localisation; finer smears objects |
| aspect sectors | 12 (30°) | Fawcett 2010 increments |
| look timeout | 20 s | engineering choice |
| prior density | 5 targets/ha | site-dependent; set from deployed objects / area |
| detection base rate π | 0.5 | must equal the calibration split's base rate (D3) |
| clamps | p ∈ [1e-5, 0.999] | engineering choice |
| PD(range) | 0.7 plateau, 3–25 m, 3 m ramps | **placeholder** |
| ρ independence angle | 90° | Paull 2013, **placeholder** until E3 |
| association | 2 m and χ² 9.21 | contact criterion; 99 % gate |
| position floor σ | 0.5 m | engineering choice |
| pose stamp tolerance | 0.25 s | must cover the odom/sonar clock offset |
| survey polygon | WGS-84, ≥ 3 vertices, validated (finite, simple, minimum area) | mission |

Validated dataclasses; unknown keys raise; flat ROS parameter names `section.key`; one YAML per idea.

---

## 11. Validation and gates

**Gate** (file 02 §4–5): `python3 -m test.smoke_test` from the package root, **without ROS**.
Pitfalls: do not source the workspace before a pytest gate (it swaps `python3` to a venv without pytest
and `lark`) — use `/usr/bin/python3`; in a ROS-sourced shell prefix `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1`;
import launch modules optionally.

Must demonstrate:
1. **Closed-form updates**: one look gives `coverage = PD` and the exact Bourgault value; a same-aspect
   repeat adds a look but no information; an orthogonal look doubles ln Q; a 45° look matches Paull
   Eq. 24; contact log-odds after detected / missed looks match their formulas.
2. **Determinism**: bitwise-identical snapshots on repeated replay (T12 criterion).
3. **Localisation ≤ 2 m** for synthetic objects from one pass (T12 criterion).
4. **Rejections** counted for every bad-input path; no exception escapes.
5. **Message layouts**: every field the adapters write exists in BeliefGrid / QueryBelief / OccupancyGrid /
   MarkerArray / DiagnosticArray with the exact dtype and CDR round-trips (`rosbags` parses the module's own
   `.msg` files ROS-free); adapters read real `ProcessedSSSPing` and `Detection2DArray` layouts.
6. **Performance**: < 1 ms per swath at 0.5 m; snapshot encode time measured.
7. **Live, simulator first**: `full_mission_launch.py world_dir:=… with_control:=true` + GCS processor
   + ping enable (file 03 B4). Then the CM-2 check against `fault_injection_sonar.yaml`.
8. **Real recordings**: replay the beach corpus (`~/ros2_ws/data/SSS_data/beach_sept/`, harbour files) —
   GCS "Save as rosbag" converts `.svlog` to rosbag2 (mcap). **[VERIFY]** which topics it contains.
   Real recordings give negative evidence, coverage and performance; there are no real detections yet.

Development detections before D3 exists: **[ASK]** whether a *dev-only* synthetic detector from
simulator ground truth (offline replay, or a sim-only tool clearly outside the online node) belongs in
this repository or in the future D7 module. The online belief node must never read `ground_truth/*` or
`/sim/*` (CM-2).

---

## 12. Requests and specifications for other modules (write them; never edit them)

Deliver these as a single `docs/REQUESTS.md` in this module, addressed to the user. Each item: what,
why, proposed spec, who decides.

### 12.1 To the detector (D3, future module) — input contract
- Publish `vision_msgs/Detection2DArray` on `/sss_ai/detections` exactly per the GCS contract (§3.2).
- `Detection2D.header.stamp` = ping time of the detection's centre row.
- `results[0].pose.covariance` x/y block in the odom frame (zero = unknown).
- `score` calibrated (temperature scaling, reliability diagram, base rate reported).
- Optional companion `/sss_ai/detection_meta` (a small typed message in `blueboat_belief_interfaces`, or
  JSON — the detector then depends on that package), keyed by detection
  `id`: `shadow` (none | weak | resolvable), `side`, `ground_range_m`, `ping_number`. The belief works
  without it. **[ANALYSE]** the minimal field set.

### 12.2 To BlueBoat-Control — v1.1 (or later) candidates
- No message work: the belief's types live in `blueboat_belief_interfaces` (§4.1).
- `/blueboat/odom` zero / other clock in real runs (existing open item; the belief only mitigates).

### 12.3 To BlueBoat-SSS
- Measured transducer offsets in the processor (existing open item; the belief inherits the geometry).

### 12.4 To the replanner (D6) — output contract
- Reads `BeliefGrid` (and optionally calls `QueryBelief`) only (CM-9); depends on
  `blueboat_belief_interfaces`. Units, frames, the sector convention and the query semantics of §6.6 are
  documented in `docs/topics.md`.

### 12.5 To the evaluation framework (D7)
- Snapshot file format and manifest (config, seed, model version, input recording, belief version).

### 12.6 Superproject integration (user's decision; file 02 §5.6)
- `git submodule add` (user), a gate line in skill `module-gates`, new file artifacts in skill
  `file-interfaces`, new topics and the `blueboat_belief_interfaces` types in the root `CLAUDE.md` §4
  tables.
- Update the CM-1 wording in the root `CLAUDE.md` (and file 02 §5.3) to say that a module may own its
  interfaces package (one owner per type; existing topics never retyped). Build order: interfaces
  packages first (`colcon build --packages-up-to blueboat_belief`).

---

## 13. Non-negotiables for this module (draft for its CLAUDE.md)

1. No edits outside `BlueBoat-Belief/` (CM-3). Needs elsewhere go to `docs/REQUESTS.md`.
2. Custom types only in `blueboat_belief_interfaces`, only where a standard type does not fit; never in
   `blueboat_interfaces` (Control is not touched).
3. Never rename or retype an existing topic; QoS matches the other end (CM-1, CM-4).
4. Online code never reads sim-only topics (CM-2).
5. `core/` imports no ROS and no Qt; adapters are ROS-free; rclpy only in the node.
6. Deterministic core: no wall clock, no randomness, stable ordering; seeded RNG only in dev tools.
7. Looks, not pings; negative evidence always updates; misses use effective PD = ρ·PD.
8. PD and ρ are placeholders until fitted; never tune a number to make a result look better.
9. Inputs and recordings are write-once (CM-7); outputs claimed with `O_EXCL`, with a manifest.
10. The belief is the only thing the replanner reads (CM-9): everything it needs is in `BeliefGrid`.
11. MATH.md is the specification; results are stated model-conditionally (CM-13).
12. Never `git commit` or `git push`.

---

## 14. Working with Claude Code on this module

**Memory and context**
- The superproject root `.claude/CLAUDE.md` loads at start. The module's `BlueBoat-Belief/.claude/CLAUDE.md`
  is the project convention for module memory: write it early (file 02 §5.7 skeleton), keep it < 200 lines,
  put details in `docs/`. Run `/memory` to see which memory files are loaded.
- Path-scoped rules (`.claude/rules/*.md` with a `paths:` frontmatter) keep, for example, "core = no ROS" out
  of context until `core/` is touched.
- Keep this brief as a file and refer to it; do not paste it into CLAUDE.md.
- Long session: `/compact` with a focus instruction (e.g. "keep decisions, open questions, test status");
  `/clear` between unrelated phases. Rewind a bad edit with Esc Esc (checkpoints).

**Workflow**
- Start in **plan mode** (Shift+Tab). Read, analyse, propose, ask. Leave plan mode only after the user
  approves.
- Use **Explore subagents** for broad read-only sweeps (v1 reference, neighbour modules, message
  definitions) so the main context stays small; ask them for conclusions with file:line references.
- Keep a task list for the phases; mark each done only when its gate passes.
- After each change: run the gate; report the pass count.

**Settings and hooks (optional)**
- `.claude/settings.json` for permissions, e.g. allow `Bash(python3 -m test.smoke_test)`, `Bash(ruff check *)`,
  `Bash(colcon build *)`; deny `Bash(git commit *)`, `Bash(git push *)`. In this project `.claude/settings.json`
  is usually gitignored, so it does not survive a clone.
- A `Stop` hook running the gate is used by the simulator module; copy that pattern if useful. Check the
  hook syntax with `/hooks` or the current docs rather than from memory.

**Skills worth writing for this module** (`.claude/skills/<name>/SKILL.md`, frontmatter `name` and
`description`; they load on demand): `belief-math-change` (spec → code → closed-form test), `replay-and-debug`
(rejection counters → likely cause), `sensor-model-integration` (plugging in E3/E4 fits). The v1 zip
contains drafts of all three; adapt them to the new interfaces.

---

## 15. References (bibliographic details verified 2026-09-29; use only for what the "Used for" says)

| Reference | Used for |
|---|---|
| Bourgault, Furukawa, Durrant-Whyte (2006). Optimal search for a lost target in a Bayesian world. *Field and Service Robotics*, STAR 24, Springer, 209–222 | Miss update; cumulative miss probability |
| Paull, Saeedi, Seto, Li (2013). Sensor-driven online coverage planning for AUVs. *IEEE/ASME Trans. Mechatronics* 18(6), 1827–1838 | Angle-dependent look dependence, Eqs. 22–24 |
| Yordanova, Gips (2020). Coverage path planning with track spacing adaptation for AUVs. *IEEE RA-L* 5(3), 4774–4780 | Residual risk ∏(1 − PD) per grid point |
| Myers, Williams (2012). Adaptive multiview target classification in SAS images using a POMDP. *IEEE JOE* 37(1), 45–55 | Same-aspect looks nearly completely dependent; class-dependent stopping |
| Fawcett et al. (2010). Multiaspect classification of sidescan sonar images. *IEEE JOE* 35(4), 863–876 | Aspect increments; multi-aspect fusion cannot fix a poor classifier |
| Wiig, Krogstad, Midtgaard (2012). Autonomous identification planning for mine countermeasures. *IEEE/OES AUV 2012*, 1–8, doi:10.1109/AUV.2012.6380733 | Contact fusion and filtering in a fielded loop |
| Guo, Pleiss, Sun, Weinberger (2017). On calibration of modern neural networks. *ICML*, PMLR 70, 1321–1330 | Calibrated confidences (temperature scaling) |
| Berlenko, Krinkin (2026). Sensor-model matching for controlled comparison of Bayesian and belief-function occupancy grid fusion. *Sensors* 26(13), 4266, doi:10.3390/s26134266 | Bayesian vs Dempster on grids: small, criterion-dependent difference |
| Elfes (1989). Using occupancy grids for mobile robot perception and navigation. *Computer* 22(6), 46–57 | Occupancy grids |
| Thrun, Burgard, Fox (2005). *Probabilistic Robotics*, MIT Press, ch. 9 | Log-odds update with clamping |
| Mahler (2007). *Statistical Multisource-Multitarget Information Fusion*, Artech House | Bernoulli / multi-Bernoulli contacts |
| Umeyama (1991). Least-squares estimation of transformation parameters between two point patterns. *IEEE TPAMI* 13(4), 376–380 | Rigid fit for odom → map (if needed) |
| IHO (2022). *S-44*, Ed. 6.1.0 | Exclusive Order 200 % search (baseline context); §3.7 search radius ≥ 3σ |
| ROS REP-103 | ENU, +y = port, radians |

Do not cite a reference from memory for a new claim; open it first. Do not quote a dB-per-degree aspect
figure (the old "20 dB / 10°" claim is unsourced and dropped).

---

## 16. Questions to settle with the user before implementation **[ASK]**

1. Output contract: `BeliefGrid` + `QueryBelief` in `blueboat_belief_interfaces` (+ OccupancyGrid display
   topics) as in §4.2; latched or periodic publication?
2. Map frame = mission `geo_anchor`; where the anchor and survey polygon come from (trajectory YAML,
   world `metadata.yaml`, or explicit config)?
3. Odom on the real boat: absolute yaw confirmed (§3.3)?
4. Per-sector miss accounting (§8.1 minimal fix) in this version?
5. Dev-only synthetic detector from simulator ground truth: here or in D7 (§11)?
6. Shadow/meta companion topic for the detector (§12.1): agreed, or derive everything without it?
7. Gate style: `test/smoke_test.py` only (simulator style) or pytest suite plus smoke entry (SSS style)?
8. Reuse level of v1: port validated components with their tests, or rewrite from the brief?
---

## 17. Scientific supplement: foundations, limitations and missing model questions

*Additive scientific supplement, 2026-11-03. The original brief above is intentionally preserved. This section expands the scientific background and identifies assumptions that need explicit treatment; it does not change project status, interfaces, ownership, module boundaries, existing decisions or the authority order in §1. It is not a claim that the project has the data needed to implement every model described here.*

### 17.1 Probability vocabulary: keep four different quantities separate

The implementation and its documentation should distinguish at least the following quantities. They answer different questions and must not be used interchangeably.

1. **Target prior / occupancy probability**: the probability that a target exists in a cell or at a contact before incorporating the current look. This is a belief about the world.
2. **Probability of detection, `PD`**: the probability that the complete operational sensing-and-detection chain produces a valid detection, conditional on a target of the specified kind being present and on stated observation conditions. It is a property of the target, sensor, detector, environment and operating point—not the probability that a target exists.
3. **Probability of false alarm, `PFA`**: the probability that the same chain produces a target detection when the target is absent. It is required to interpret both detections and non-detections as Bayesian evidence.
4. **Detector score or confidence, `q`**: a value emitted for a candidate. It is a posterior probability only if its event definition, calibration procedure and prior prevalence are known. A class score such as “probability this candidate is class X” is not automatically the probability that a target exists at that location.

A fifth quantity is useful but distinct: **cumulative probability of detection / modelled coverage**, `1 − Q`, where `Q` is the probability of missing a target over the sequence of looks under a stated per-look detection model and dependence assumption. It is not geometric swath coverage and it is not the posterior probability that a target exists.

Use explicit names in `MATH.md` and in message documentation. Avoid using “confidence”, “coverage”, “detection probability” and “target probability” as synonyms.

### 17.2 What controls sidescan-sonar detectability

A sidescan-sonar image is not a uniform range sensor. Each ping illuminates a seabed strip from an oblique, side-looking geometry. A target can produce a strong highlight, a shadow, or both; the appearance depends on acoustic illumination, target properties and the local seabed. The shadow is geometrical evidence, not a guaranteed consequence of target presence. Its visibility and extent depend on target height and shape, grazing/incidence geometry, sonar altitude and range, seabed slope/roughness, and image quality. Highlight strength is likewise not determined by range alone.

The classical active-sonar equation is a useful physical starting point. In a simplified noise-limited monostatic form, signal excess depends on source level, two-way transmission loss, target strength, ambient noise, array/directivity gain and detection threshold. Reverberation-limited operation introduces a different limiting background. The sonar equation organizes these variables; it does not, by itself, supply a validated `PD` curve for this particular sidescan sonar and detector. Statistical detection theory maps signal conditions and a chosen threshold to `PD` at a specified `PFA` (Hodges, 2010; Urick, 1983).

For this module, a useful operational definition is:

`PD(x, z) = P(the deployed detector emits an accepted target detection | a target is present at x, observation conditions z)`

where `z` may include at least lateral/ground range, target class and size, target-relative insonification aspect, seabed/environment class, sonar settings, altitude/geometry, vehicle motion/navigation quality, and image-quality indicators. The list is a scientific model inventory, not a requirement to implement all covariates in the first version. Start with the simplest model justified by available data and label omitted effects.

An acoustic/physics model and an algorithmic detector model should not be confused. A target may create an acoustic return but still fail the image-processing or neural-detector stages. Conversely, a classifier may report a false candidate. If a physical model and an ATR model are composed, define the conditional events and validate the composition; do not multiply two quantities merely because both are called “detection probabilities”. The end-to-end operational `PD` is the quantity most directly relevant to whether a survey look can find a target.

**Evidence and limits.** Blondel (2009) is a broad reference on sidescan-sonar geometry, data and interpretation. Hożyń (2021) reviews sonar-image target detection/classification and describes the physical significance of highlight and shadow features. Li, Li and Wu (2025) specifically discuss acoustic illumination, radiometric effects and the use of shadow information in SSS mine-like-target detection. Qiao et al. (2026) provide a recent AUV/SSS survey covering preprocessing, target recognition and survey planning. These sources support the need for a multi-factor model; they do not provide parameters for this project's sensor or target classes.

### 17.3 `PD` should be an empirical sensor-and-detector performance model

A single plateau value such as the placeholder in §10 is suitable only as an explicitly labelled engineering placeholder. It is not a generally valid sidescan-sonar performance law. Existing SSS performance work demonstrates that probability of detection varies with lateral range and operating/environmental conditions. Nguyen and Murdoch (2012) study the statistical accuracy of SSS detection-performance estimates as a function of range; they investigate piecewise logistic regression with cubic B-splines and confidence intervals using simulated and CITADEL sea-trial data. Paull et al. (2013) describe a lateral `P(y)` curve and enumerate seabed, target, sonar and platform variables that affect it. Yordanova and Gips (2020) use through-the-sensor performance estimates to construct `PD`-versus-range curves for an AUV-mounted side-looking sonar and update those estimates from observed data.

A defensible progression is:

- **Initial placeholder:** keep a simple, documented curve only to exercise the software and integration. Do not interpret it as a measured performance claim.
- **Range-only empirical model:** estimate a `PD(r)` curve from labelled observations if the dataset supports only range reliably. State the sensor, detector, target population, operating threshold and conditions to which it applies.
- **Conditioned model:** add aspect, seabed, target class/size, vehicle motion or image-quality covariates only when observations support their estimation and validation.
- **Uncertainty:** report confidence/credible intervals or another uncertainty representation for the fitted curve. A fitted mean curve without its data support can create false precision.

Nguyen and Murdoch's spline approach is a relevant precedent, not a mandatory algorithm. A logistic model, spline/GAM, calibrated empirical bins or hierarchical model may be preferable depending on data volume and coverage. Do not select a complex model merely because it is available.

A model trained on one sonar, ATR version, threshold, seabed population or platform regime is not automatically transferable to another. Version the model and record the calibration dataset and operating point in snapshots. The original brief's instruction to separate simulator/world parameters from planner/model parameters (§8.5) remains important: fitting and evaluating on the same simulated generation model is circular evidence of real-world validity.

### 17.4 Separate target-relative aspect from dependence between repeated looks

Two different angular effects are relevant and should be named separately.

- **Target-relative insonification aspect** describes the direction from which the target is illuminated relative to the target's own orientation. A target's scattering/highlight/shadow response can vary with this angle. If target orientation is unknown, it is a latent variable. A possible model has a response function `A(β − θ)`, where `β` is the look direction and `θ` is target orientation; this is a model family to estimate, not an assumed universal angular law.
- **Inter-look dependence** describes how much new information a later observation contributes after earlier observations. It can arise from repeated illumination geometry, common image conditions, correlated pings, common navigation error or persistent clutter. It is not the same physical effect as target orientation.

Paull et al. (2013), §III-C, Eqs. 22–24, explicitly **assume** a linear interpolation between fully dependent parallel looks and independent perpendicular looks. If `c1` is the stronger previous confidence, `c2` the new single-look confidence and `α` the acute angle between the two insonification lines, their rule is:

`c_total = c1 + (2α/π) c2 (1 − c1)`

This can be written as:

`c_total = 1 − (1 − c1)(1 − ρ c2)`, with `ρ = 2α/π`.

That algebra explains the effective-detection form used in §6.3–6.4 when cumulative miss probability is represented as a product. The important qualification is that Paull et al. present this as a modelling assumption for combining confidence—not as an experimentally established correlation law for every sonar, target or environment. The paper also states that its purpose is not to verify the ESPRESSO sensor-performance model. The model's application to the present shallow-water SSS setup therefore remains an assumption until experimentally checked.

The angular quantity must be defined geometrically before coding. A vehicle course change, a track-line angle, a transducer-to-cell look direction and the target-relative aspect are not necessarily identical. For a particular cell, the relevant look direction is determined by the acoustic line of sight and the port/starboard side of observation. Whether the current data and frame conventions permit that direction to be reconstructed accurately must be verified against the repository and sensor geometry; this supplement does not decide it.

The 12-sector / 30° representation is a discretization choice. It is not established by the cited literature as the uniquely correct resolution. Sector counts are useful for compact history, but a sector bitmap alone loses the exact angular difference needed by a continuous interpolation. Retaining the actual look bearing (or sufficient angular information) alongside the sector index is a possible future refinement. Do not add it without checking interface size, replay requirements and downstream needs.

There is also a sequential-use caveat. In Paull et al.'s pairwise derivation, the two confidence values are ordered without loss of generality so that `c1 ≥ c2`. A sequential grid update that always treats the accumulated prior confidence as `c1` and the new look's `PD` as `c2` should therefore be checked when the new look has a higher single-look `PD` than the accumulated confidence, or when `PD` varies materially between looks. The pairwise equation does not, by itself, prove that every order-dependent multi-look recursion is exact. Verify the implementation's handling of this case with closed-form examples; if it remains a heuristic, document it as such.

Do not apply both an orientation-dependent `PD(r, β − θ)` and an angular novelty factor `ρ(Δβ)` as if they described the same effect. They can coexist only if their meanings are distinct and the joint model avoids double-counting aspect-related performance. E3/other data must determine whether the simpler model is adequate.

### 17.5 Bayesian evidence: non-detections also depend on false alarms

For a binary observation event `D` (“the detector emitted an accepted detection”) and target hypothesis `H` (“the target exists”), define:

- `PD = P(D | H)`
- `PFA = P(D | not H)`

Bayes' rule updates prior log-odds by a likelihood ratio. For a detection:

`L_new = L_old + ln(PD / PFA)`

For a non-detection:

`L_new = L_old + ln((1 − PD) / (1 − PFA))`.

These are the binary-observation likelihood-ratio updates for a specified detector and operating condition. They make clear that `PD` and `PFA` are a pair: a change in detector threshold generally changes both. The detection threshold is ordinarily assessed through an ROC operating point or equivalent `PD`-at-`PFA` description (Hodges, 2010; DOSITS, “Detection Threshold for Sonar”).

The existing miss increment in §6.4, `ln(1 − ρ·PD)`, and the displayed Bourgault form are the zero/negligible-false-alarm special case for the non-detection update (with `PD_eff = ρ·PD`). Preserve the existing equations as the current proposed model, but explicitly state this assumption in `MATH.md`. If the detector has non-negligible false alarms, a fully Bayesian occupancy update requires the corresponding `PFA` term and a clearly defined spatial/temporal false-alarm model. A residual-risk product may still be useful as a conditional miss-risk metric, but it must not be presented as a complete posterior occupancy update when the false-alarm likelihood has been omitted.

Similarly, a detected candidate's calibrated score can provide a likelihood ratio only under the conditions in §17.6. Do not update the same observation once with `PD/PFA` and again with a confidence-derived likelihood ratio unless the two terms represent explicitly factorised, conditionally distinct evidence. Otherwise the same evidence is counted twice.

### 17.6 Detector-score calibration is not the same as estimating `PD`

Guo et al. (2017) show that modern neural-network confidence scores may be miscalibrated and that temperature scaling can improve calibration in their studied classification settings. This supports calibration as a requirement; it does not establish that temperature scaling will calibrate this project's detector or that a classification score is an existence probability.

The conversion already proposed in §6.5,

`ln LR = logit(q) − logit(π)`,

is valid when `q` is a calibrated posterior probability for the same binary event being updated, and `π` is the prior prevalence for the same sampling process and calibration population. Under those conditions, posterior odds divided by prior odds recover the likelihood ratio represented by the observed features. If the calibration sample was artificially balanced, filtered to hard negatives, or otherwise has a different prevalence from the deployment population, the prior needs to be handled explicitly. If `q` is a class probability conditional on a candidate already having been proposed, its event is not automatically “a target exists in this cell”.

Before using `score` as `q`, document and verify:

- What exact event is being predicted: target-vs-background, a class label conditional on a detection, or something else?
- What examples were included in calibration, including candidate-generation and negative-sampling rules?
- What prevalence/base rate `π` applies to that exact population?
- Whether the score is calibrated overall and by relevant target class/range/aspect/site strata.
- Whether thresholding, non-maximum suppression or other candidate-selection steps change the population to which calibration applies.

Use reliability diagrams and proper scoring rules such as log loss and the Brier score, while separating calibration from discrimination. Brier score and log loss combine several aspects of probabilistic prediction quality; neither alone proves calibration. Report ROC/precision-recall behaviour and `PD` at the selected false-alarm operating point as well. Calibration should be evaluated on held-out targets/missions/sites, not on the same examples used to fit the calibration transform.

### 17.7 Contacts: existence, association, clutter and spatial uncertainty

A Bernoulli random finite set is a principled representation for a single object that may or may not exist, with an existence probability and a state distribution. Multi-Bernoulli and labelled-RFS filters extend this idea to multiple objects and explicit data-association hypotheses (Mahler, 2007; Vo and Vo, 2013; Vo, Vo and Phung, 2014). The contact representation in §6.5 is inspired by this family, but the proposed nearest-neighbour/2 m/χ²-gate management should not be described as a complete multi-Bernoulli or labelled-RFS filter unless its association, clutter and birth/death models actually implement those assumptions.

In a multi-contact scene, multiple candidate detections can fall inside multiple gates. Greedy nearest-neighbour assignment can then make an irreversible wrong association. Probabilistic data association, joint probabilistic data association, multiple-hypothesis tracking and random-finite-set methods exist because target-to-measurement association in clutter is a central inference problem (Bar-Shalom, Daum and Huang, 2004). The current simpler association rule may be appropriate for the expected sparse scene; that is an engineering choice to test, not a general guarantee. At minimum, tests should cover two nearby contacts, crossing/overlapping gates, duplicate detections, clutter candidates, late observations and stable IDs.

The 99% two-dimensional χ² threshold of 9.21 is a statistical Mahalanobis gate under the stated Gaussian residual model. The additional fixed 2 m gate is a separate project criterion. Their combined effect depends on covariance size; document whether the two are intentionally conjunctive and test both tight and broad covariance cases.

Information-form Gaussian fusion assumes that the observations being fused have the relevant conditional independence and covariance model. Repeated detections may share common errors from odometry, map alignment, transducer geometry, timing and target extent. A covariance floor can prevent unbounded confidence, but it does not in itself model shared systematic error. Validate the reported covariance by checking empirical position-error coverage of its confidence ellipses on data with known target locations. The ≤2 m localisation criterion and covariance calibration are related but distinct tests.

The rules “shadow resolves” or “two consistent detected looks” for confirmation are project-level decisions. They are not universal statistical thresholds. Their false-confirmation and missed-confirmation behaviour should be measured on labelled examples, especially where shadow evidence is unavailable or ambiguous.

### 17.8 Grid assumptions, dependence and the meaning of coverage

Classical occupancy grids use Bayesian cell updates and commonly represent cells independently for tractability (Elfes, 1989; Thrun, Burgard and Fox, 2005). This is a useful approximation, not a statement that real seabed targets or sonar measurements are spatially independent. A single extended target can span many cells; one ping samples a correlated swath; pose and registration errors shift many cell observations together. Thrun (2003) discusses how cell-wise independence can discard spatial dependencies and produce inconsistent maps, even when the world is static.

The hybrid representation in §6.1 is a practical way to separate diffuse negative evidence from localised positive target hypotheses. Keep its assumptions visible:

- `p_background` is a cell-level approximation for targets not represented by explicit contacts.
- Contact existence and spatial mass are a second representation of the same physical target; rendering and query operations must combine them without double-counting.
- The union formula in §6.6 assumes independence of the background and contact existence events used in the product. This is an approximation when a contact was created from evidence already represented in the grid.
- A Gaussian contact mass spread over cells is not equivalent to independent target existence in every cell. The exact disc query in §6.6 is important for this reason.
- The product `Q = ∏(1 − PD_i)` is the probability of no detection over looks only if those looks are conditionally independent under the model. When a dependence discount is used, it is a modelled approximation to that joint miss probability, not an observed fact.

If the target count in each cell is modelled as Poisson with mean `λ = ρ_T·Δ²/10⁴`, the probability of at least one target is `1 − exp(−λ)`. The existing `p0 = λ` in §6.1 is the rare-event approximation for small `λ`, not the exact Poisson conversion. At the small prior values contemplated in §10 the numerical difference may be negligible, but label the approximation and retain the exact expression as the alternative if priors or cell size change.

### 17.9 A non-detection is evidence only when there was a valid opportunity to detect

A geometric swath crossing a cell does not, by itself, prove that a detector made a valid observation of that cell. To credit a miss, the system needs a defensible chain of evidence that:

1. The cell was inside the usable insonified region, not merely inside a nominal range rectangle (consider nadir gap, range cutoffs, geometry, shadow/occlusion and quality limits).
2. The relevant ping/image was acquired and processed successfully.
3. The detector was enabled and completed its inference for the relevant data.
4. The detector's output contract makes an empty result meaningful as “processed; no candidate”, rather than “no message”, “inference skipped”, “dropped frame” or “pipeline unavailable”.
5. The `PD` model corresponds to the sensor, detector, threshold and conditions in force for that look.

If the detector only publishes non-empty detections, the absence of a message cannot be interpreted as a miss. The contract needs a way to establish processing coverage/health, or an equivalent reliable per-ping completion signal. This is a scientific observability requirement, not a proposal to change any existing topic without the owner's decision. It should be investigated during the existing Phase 1/2 work and raised as a request if another module must provide information.

Likewise, do not apply a miss to a contact merely because a ping arrived. The contact must lie in the valid detection opportunity for that look and be within the conditions represented by its `PD` model. Close a look and apply its miss contribution once, consistently with the same-look replacement rule in §6.5 and the v1 lesson in §7.

### 17.10 Data and validation needed to support scientific claims

A useful validation dataset should preserve target-level and look-level information rather than treating every ping as an independent trial. For each known target and each observation opportunity, retain the target identity/location and the relevant acquisition conditions; record whether the detector produced a detection, its score/class, and the reasons a look was unusable. Include target-absent/background opportunities so false alarms can be measured. Record the sensor/detector version, threshold and calibration prevalence.

When estimating `PD` and `PFA`:

- Stratify or model by range, target class/size, aspect and relevant environment/platform conditions where the data support it.
- Use confidence intervals, especially in sparse bins. Nguyen and Murdoch (2012) specifically address uncertainty in SSS detection-performance estimates.
- Split evaluation by target identity and, where possible, by mission/site/recording. Do not put near-duplicate looks of the same target in both training and test sets; that leaks target-specific information.
- Account for repeated observations within a pass/target. Treating highly correlated pings as independent Bernoulli trials will make intervals too narrow and performance look better than it is.
- Report both `PD` at a stated `PFA`/operating threshold and score calibration. Neither substitutes for the other.
- Keep simulator-derived and real-data-derived results separate. A simulation can test the implementation against its own known model; it does not independently validate the model against the sea.

For the angular-dependence model, evaluate predicted cumulative miss probability against repeated observations of known targets from measured look bearings. Compare at least the current Paull-style linear rule with simpler baselines (no credit for same-aspect repeats; full independence) and with any data-supported alternative. Avoid fitting and validating the angular rule on the same targets or passes. If the available data cannot distinguish these models, retain the current rule as a labelled assumption and report the uncertainty rather than claiming empirical validation.

For position uncertainty, report error distribution and confidence-ellipse coverage in addition to mean/maximum localisation error. For contact existence, assess calibration and false/duplicate contact rates, not only whether a track was produced. For modelled coverage/residual risk, compare predicted risk with observed target misses over independent survey trials or held-out target/look sequences.

### 17.11 Scientific decisions that remain open (do not resolve by inference)

The following should be treated as explicit [ANALYSE], [VERIFY] or [ASK] items during Phase 1/2. They are additions to—not replacements for—the original §16 questions.

1. **Operational `PD`:** Is the model intended to represent acoustic/imagery observability, the detector's candidate-generation probability, or the end-to-end probability that the deployed detector outputs an accepted target detection? Which stages are already represented by the detector score?
2. **False alarms:** Is `PFA` measurable and non-negligible at the chosen detector operating point? What is the spatial/temporal false-candidate density, and does the proposed grid/contact update model it?
3. **Empty detections:** Does the detector publish an empty array or another completion/health indication for every processed observation opportunity? How can the belief distinguish a valid non-detection from missing inference/output?
4. **Aspect geometry:** Is the sector assigned from vehicle heading, track direction or per-cell acoustic look bearing? Can port/starboard and the transducer/cell geometry be distinguished reliably? What angle is actually used in the Paull interpolation?
5. **Two angular models:** Is there evidence to separate target-relative aspect response from inter-look dependence, or should the first version retain one clearly labelled simplified angular factor?
6. **Contact association:** Is the expected target density low enough for nearest-neighbour association, or do plausible cases require global/probabilistic assignment and an explicit clutter model?
7. **Common-mode uncertainty:** Which pose, alignment, timing and geometry errors are shared across observations and therefore should not shrink as independent measurement noise?
8. **Grid independence and prior:** Is the low-density independent-cell approximation acceptable for the intended target density and map resolution? Is the current linear `p0` approximation sufficient?
9. **Model-transfer claims:** Which results are simulator-only, which are fitted to real labelled observations, and which have an independent held-out evaluation?

Do not change the existing equations, interface contracts, module boundaries or project decisions solely because these questions are listed here. Record the decision, evidence and consequences in the project-design documents after the user approves them.

### 17.12 Minimum scientific documentation expected in `docs/MATH.md`

For every probability and update, state: (i) the event/hypothesis being modelled, (ii) the conditioning variables, (iii) the prior and likelihood terms, (iv) independence/dependence assumptions, (v) whether `PFA` is modelled or neglected, (vi) the range of validity and data source, and (vii) what is a project decision versus a literature-supported result. Each equation should point to its implementation function and a closed-form test, as already required in §6.

At minimum, the math document should include a compact model table separating:

| Quantity | Meaning | Evidence/status to report |
|---|---|---|
| Target prior | Probability a target exists before the current look | Site/mission prior or engineering assumption |
| `PD` | End-to-end detection probability conditional on target presence | Placeholder, simulator-derived, or empirical; state sensor/detector/conditions |
| `PFA` | False-alarm probability at the same operating point | Measured, modelled, or explicitly neglected |
| `q` | Detector score/posterior for a clearly named event | Calibrated or uncalibrated; state prevalence and dataset |
| `ρ` | Incremental-information/dependence factor between looks | Paull-style assumption or empirically fitted model |
| `Q` | Conditional probability of no detection over stated looks | Independence/dependence model and per-look `PD` |
| Contact `p_exists` | Probability a particular contact exists | Association/clutter/measurement model and project confirmation criteria |
| Position covariance | Uncertainty in target location in the stated frame | Measurement, pose, common-mode and alignment terms |

---

## 18. Expanded scientific references and source-use notes

*The original §15 bibliography is retained unchanged. The following additions are intended to strengthen its scientific coverage. “Use” describes what the source can support; it is not permission to extend a result beyond the source's sensor, data or assumptions.*

### 18.1 Sidescan-sonar physics, image formation and detection performance

1. **Blondel, P. (2009).** *The Handbook of Sidescan Sonar*. Springer Praxis Books. https://doi.org/10.1007/978-3-540-49886-5. **Use:** SSS geometry, image interpretation, seabed structures and processing context. A broad technical reference; not a project-specific `PD` calibration.
2. **Urick, R. J. (1983).** *Principles of Underwater Sound*, 3rd ed. McGraw-Hill. **Use:** Underwater acoustics, propagation, scattering, noise and sonar performance foundations. The sonar equation is a physical performance framework, not by itself an empirical detector-performance curve.
3. **Hodges, R. P. (Ed.) (2010).** *Underwater Acoustics*. Wiley. In particular, Chapter 2, “The Sonar Equations”, https://doi.org/10.1002/9780470665244.ch2, and Chapter 17, “Cumulative Probability of Detection”, https://doi.org/10.1002/9780470665244.ch17. **Use:** Sonar-equation performance terms, statistical detection, and cumulative probability of detection. Use the chapters on false alarms/detection theory in the same book when specifying the `PD`/`PFA` operating point.
4. **Hożyń, S. (2021).** A Review of Underwater Mine Detection and Classification in Sonar Imagery. *Electronics*, 10(23), 2943. https://doi.org/10.3390/electronics10232943. **Use:** Review of sonar-image detection/classification methods and target highlight/shadow characteristics. It is a mine-like-target review, not a calibration dataset for this system.
5. **Li, S., Li, T., & Wu, Y. (2025).** Side-scan sonar mine-like target detection considering acoustic illumination and shadow characteristics. *Ocean Engineering*, 336, article 121711. https://doi.org/10.1016/j.oceaneng.2025.121711. **Use:** Recent SSS-specific treatment of acoustic illumination, radiometric distortion and shadow information in detection. Algorithm/data scope should be checked before transferring its results.
6. **Qiao, J., Yu, J., Huang, Y., Feng, H., Jia, D., Wang, Z., & Wang, B. (2026).** AUVs for Seabed Surveying: A Comprehensive Review of Side-Scan Sonar-Based Target Detection. *Journal of Marine Science and Engineering*, 14(2), 145. https://doi.org/10.3390/jmse14020145. **Use:** Recent end-to-end review of AUV SSS preprocessing, detection/recognition and survey planning. Use it to discover literature and research gaps, not as primary evidence for a particular equation.
7. **Wang, Z., Lei, H., Feng, H., Qiao, J., Huang, Y., & Yu, J. (2026).** A physics-informed and data-driven framework for AUV side-scan sonar imaging quality prediction under motion-induced degradation. *Ocean Engineering*, 366, Part 4, article 127967. https://doi.org/10.1016/j.oceaneng.2026.127967. **Use:** Very recent work connecting sonar imaging quality to acoustic effects and AUV motion, using a physics-informed prior and Gaussian-process residual learning. It motivates considering motion/quality covariates and uncertainty; it is not a validated model for this project's sensor.

### 18.2 SSS probability of detection, coverage and multi-aspect observations

8. **Nguyen, P., & Murdoch, D. (2012).** Accuracy assessment of detection performance for sidescan sonars. *Naval Research Logistics*, 59(8), 629–642. https://doi.org/10.1002/nav.21513. **Use:** SSS `PD` as a function of range, estimation uncertainty, confidence intervals and piecewise logistic spline modelling demonstrated with CITADEL data. A particularly relevant source for fitting and reporting a real `PD` curve.
9. **Paull, L., Saeedi, S., Seto, M., & Li, H. (2013).** Sensor-Driven Online Coverage Planning for Autonomous Underwater Vehicles. *IEEE/ASME Transactions on Mechatronics*, 18(6), 1827–1838. https://doi.org/10.1109/TMECH.2012.2213607. **Use:** SSS/AUV coverage planning, lateral `P(y)` performance curves, relevant sensor/environment/platform factors, and the assumed linear confidence-combination rule for different insonification angles in §III-C, Eqs. 22–24. The paper explicitly presents the angular interpolation as an assumption; it does not empirically establish it for every SSS configuration.
10. **Yordanova, V., & Gips, B. (2020).** Coverage Path Planning With Track Spacing Adaptation for Autonomous Underwater Vehicles. *IEEE Robotics and Automation Letters*, 5(3), 4774–4780. https://doi.org/10.1109/LRA.2020.3003886. Preprint: https://arxiv.org/abs/2006.12896. **Use:** Through-the-sensor performance estimates, `PD`-versus-range curves, and residual risk `RR = ∏(1 − PD_i)` over looks. The paper's actual platform used side-looking synthetic aperture sonar; treat it as closely related AUV side-looking sonar evidence, not as a direct calibration for this SSS. Its product relies on the per-look probability model and independence assumptions.
11. **Fawcett, J., Myers, V., Hopkin, D., Crawford, A., Couillard, M., & Zerr, B. (2010).** Multiaspect classification of sidescan sonar images: Four different approaches to fusing single-aspect information. *IEEE Journal of Oceanic Engineering*, 35(4), 863–876. https://doi.org/10.1109/JOE.2010.2069832. **Use:** Empirical two-look SSS classification/fusion on Klein 5500 data. It supports the value and study of multiaspect fusion; it is not a general physical `PD` model or proof of a universal angular dependence function.
12. **Myers, V., & Williams, D. P. (2012).** Adaptive multiview target classification in synthetic aperture sonar images using a partially observable Markov decision process. *IEEE Journal of Oceanic Engineering*, 37(1), 45–55. https://doi.org/10.1109/JOE.2011.2175510. **Use:** Adaptive selection of additional sonar views to reduce classification uncertainty, validated with SAS data. Its modality is SAS, so cite it as a multiview-planning/classification analogy rather than direct SSS evidence. Do not use it alone to assert a measured same-aspect SSS correlation coefficient.
13. **Cai, C., Chen, J., Yan, Q., Liu, F., & Zhou, R. (2022).** A prior information-based coverage path planner for underwater search and rescue using autonomous underwater vehicle (AUV) with side-scan sonar. *IET Radar, Sonar & Navigation*, 16(7), 1225–1239. https://doi.org/10.1049/rsn2.12256. **Use:** Related SSS/AUV search planning combining prior target information and sensor detection capability. This is principally a planning paper; it provides context for downstream use, not a required design for this belief module.

### 18.3 Probabilistic mapping, dependence and multi-target estimation

14. **Thrun, S. (2003).** Learning Occupancy Grid Maps with Forward Sensor Models. *Autonomous Robots*, 15, 111–127. https://doi.org/10.1023/A:1025584807625. **Use:** Explains limitations of independent-cell occupancy formulations and develops forward sensor models that preserve dependencies among cells. Important when documenting the grid as a tractable approximation.
15. **Ribo, M., & Pinz, A. (2001).** A comparison of three uncertainty calculi for building sonar-based occupancy grids. *Robotics and Autonomous Systems*, 35(3–4), 201–209. https://doi.org/10.1016/S0921-8890(01)00116-6. **Use:** Comparison of Bayesian, Dempster–Shafer and fuzzy uncertainty approaches for sonar-based occupancy grids. The sensor/environment differ from sidescan seabed search; use only for general grid-fusion context.
16. **Bar-Shalom, Y., Daum, F., & Huang, J. (2004).** The probabilistic data association filter. *Proceedings of the IEEE*, 92(3), 536–557. https://doi.org/10.1109/JPROC.2003.823149. **Use:** Data association in clutter with less-than-unity detection probability; explains why hard nearest-neighbour decisions can fail as false alarms or ambiguity increase.
17. **Vo, B.-T., & Vo, B.-N. (2013).** Labeled random finite sets and multi-object conjugate priors. *IEEE Transactions on Signal Processing*, 61(13), 3460–3475. https://doi.org/10.1109/TSP.2013.2259822. **Use:** Formal labelled-RFS and multi-object Bernoulli foundations. Relevant if the contact layer is developed toward principled multi-target existence and association filtering.
18. **Vo, B.-N., Vo, B.-T., & Phung, D. (2014).** Labeled random finite sets and the Bayes multi-target tracking filter. *IEEE Transactions on Signal Processing*, 62(24), 6554–6567. https://doi.org/10.1109/TSP.2014.2364014. **Use:** Multi-target Bayes filtering and tractable labelled-RFS implementation. It is a more formal framework than a simple gated contact table; cite it to describe the theoretical boundary, not to imply the proposed implementation is a GLMB filter.
19. **Bar-Shalom, Y., & Tse, E. (1975).** Tracking in a cluttered environment with probabilistic data association. *Automatica*, 11(5), 451–460. https://doi.org/10.1016/0005-1098(75)90021-7. **Use:** Foundational probabilistic data association under uncertain measurement origin and clutter.

### 18.4 Calibration and probabilistic prediction quality

20. **Guo, C., Pleiss, G., Sun, Y., & Weinberger, K. Q. (2017).** On Calibration of Modern Neural Networks. *Proceedings of the 34th International Conference on Machine Learning*, PMLR 70, 1321–1330. https://proceedings.mlr.press/v70/guo17a.html. **Use:** Neural-network confidence calibration and temperature scaling. It motivates testing calibration; it does not prove that any particular detector score is calibrated or that its score means target existence.
21. **Murphy, A. H. (1973).** A new vector partition of the probability score. *Journal of Applied Meteorology*, 12(4), 595–600. **Use:** Decomposition of the Brier probability score into reliability, resolution and uncertainty; useful background when reporting probabilistic quality. The Brier score should not be treated as a calibration-only statistic.

---

## 19. Source interpretation and scientific caution register

| Existing claim or reference use | Clarification to retain |
|---|---|
| Paull et al. (2013), Eqs. 22–24 support `ρ = Δangle/90°` | The paper explicitly assumes a linear interpolation between dependent parallel looks and independent perpendicular looks. It is a principled starting hypothesis, not a universal empirical law. Use the angle defined by the actual insonification geometry. |
| Myers & Williams (2012) support near-complete same-aspect dependence | The cited study is about SAS and adaptive multiview classification. It supports the relevance of additional views, but should not alone be cited as a measured same-aspect SSS dependence model. |
| Fawcett et al. (2010) support multiaspect fusion | This is direct SSS two-look classification/fusion evidence. It does not establish the per-look `PD` model or Paull's angular dependence function. |
| Yordanova & Gips (2020) support residual risk multiplication | The paper uses `RR = ∏(1 − PD_i)` for repeated looks, with its stated performance model and assumptions. It uses SAS on an AUV; do not transfer its numerical `PD` curve or sensor thresholds. |
| Guo et al. (2017) support calibrated detection scores | They study calibration of neural-network classification probabilities. The actual detector's score semantics, class/event definition and prevalence still require empirical verification. |
| Bourgault-style negative log-odds update | The current no-detection expression assumes negligible/zero false-alarm probability. If `PFA` is material, use a likelihood ratio including `PFA` or state why that term is neglected. |
| Bernoulli contacts based on Mahler (2007) | The proposed contact table is Bernoulli-inspired. It is not automatically a complete multi-Bernoulli/RFS tracker without the corresponding association, clutter and measurement models. |
| Grid-cell independence | A deliberate tractability approximation. Correlated swath observations, shared pose error and targets spanning cells can violate it; Thrun (2003) discusses the dependency limitation. |
| Range-only `PD(r)` | A useful first empirical model if supported by data. It is conditional on the sensor/detector/target population/threshold and is not universal across environments or platform states. |

**Citation rule.** For every new scientific claim in code comments or `docs/MATH.md`, cite the source that directly supports that claim and state any sensor/modality mismatch (SAS versus SSS), modelling assumption or domain limitation. A source that is relevant to the general idea is not automatically evidence for a numerical parameter or a project-specific performance claim.
