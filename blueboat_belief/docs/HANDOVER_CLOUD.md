# HANDOVER_CLOUD: BlueBoat-Belief cloud session BELIEF-1

Updated at the end of every phase; the newest entry is first.

## Second pass: primary sources (Paull 2013, Bourgault 2006): 2026-11-04

The owner supplied both PDFs and asked to "understand what they change, improve anything
related, push the module as far as possible". Everything is in the repo, uncommitted.

### What the papers changed (details in `MATH.md` §13, `REFERENCES.md`)
- **Paull §III-C (Eqs. 22–24).**
  - The rule assumes c¹ ≥ c², and α is the acute angle between insonification lines.
  - The default ordering is now **`paull_pairwise`** (owner decision). Sequential under-credited
    a stronger later look.
  - Folding is confirmed Paull-faithful.
  - The linear rule is Paull's own assumption: the brief's "citing Fawcett" overstates.
- **Paull Eqs. 5 and 9.** The confidence is `P(m = M)` with symmetric errors (range 0.5–1), not
  PD, so Eq. 9 is no consistency check for Belief. This corrects the superproject note.
- **Paull §II-C (turns).** A new **turn back-scan gate** (`core/turn.py`) excludes inner-side
  samples beyond `r* = v/|ω|`. In a synthetic half turn it removed all 508 fake far-side looks.
- **Paull Eqs. 15–16** ("no overlap between subsequent sonar pings") support "looks, not pings".
- **Bourgault Eqs. 4–8 and §4.1** confirm the PFA = 0 miss update for **one normalised target
  PDF**. Belief's per-cell grid is the cell-conditioned analogue.
  - Eq. 7 gives a new `summary.area_cumulative_pd` (mean in-area coverage) in `/diagnostics`
    and manifests.
- **Bourgault §4.3 (smooth models)** makes sector-stepped ρ a [P2] item: a bearing-resolved ρ, a
  contract addition to settle with the Replanner.
- **Paull Eqs. 11–13** (expected confidence over unknown parameters) make a seabed-mixture PD a
  [P2] item.

### Also added
- `blueboat-belief-make-area`: a minimal survey-area tool (anchor + rectangle, or from a world;
  exclusive create).
- `docs/REFERENCES.md`: the read status of every source.

### Gate
**188 checks, 0 skipped, ~29 s.** New checks:
- [6]: pairwise default; stronger and weaker same-sector repeats; sequential switch.
- [19]: turn estimator closed forms; far-side cells; gap restart; straight pass.
- [20]: make-area (anchor, never overwrites, invalid input refused, from world without ground
  truth); `area_cumulative_pd` identity.

### Contract
No field changed. Predictors must read `model_params_json.rho_model.ordering` (now
`paull_pairwise`) and `turn_gating` (`CONTRACT_DEVIATIONS.md` coupling notes).

## Phases 3–5 (build, offline validation, report): 2026-11-04, complete offline

Delivery is in the repository on branch `claude/amazing-knuth-p432sx`, **uncommitted**: the owner
commits and pushes. Packaged archives are discontinued (owner, 2026-11-04).

### What was built
- **`blueboat_belief_interfaces/`** (ament_cmake): `BeliefGrid.msg`, `BeliefContact.msg`,
  `QueryBelief.srv`, all to contract `belief_grid/0`, plus the ADDED `area_x/area_y` and
  `look_detected`.
- **`blueboat_belief/blueboat_belief/core/`**, ROS-free and deterministic, 12 modules:
  `config`, `geo`, `area`, `alignment`, `pose`, `models`, `geometry`, `grid`, `contacts`,
  `coverage`, `belief`, `contract`.
- **`io/`**: survey-area loader, byte-deterministic snapshot codec + manifest, data root with O_EXCL.
- **`ros/adapters.py`** (ROS-free, duck-typed) and **`ros/belief_node.py`**, the only rclpy file:
  4 inputs, grid / display / markers / diagnostics outputs, query service, periodic and final
  snapshots.
- **`cli/replay.py`**: `blueboat-belief-replay`, a rosbags-based deterministic offline replay.
- **`launch/belief_launch.py`**, with `record_inputs:=true` for Belief's four inputs.
- **Config:** `config/belief.yaml` (equal to the code defaults), `config/survey_area.example.yaml`.
- **Docs:** `MATH.md` (spec), `topics.md`, `DESIGN.md`, `SURVEY_AREA_FORMAT.md`, `REQUESTS.md`,
  `CONTRACT_PROVIDED.md`, `CONTRACT_DEVIATIONS.md`, `LOCAL_CHECKLIST.md` (L-1…L-8, B-1…B-7).
- **Module files:** `.claude/CLAUDE.md` (127 lines), `.claude/TODO.md`; `README.md` with the
  calibration priority at the top.

### Gate
`python3 -m test.smoke_test` from `blueboat_belief/` gives **169 checks passed, 0 skipped,
~29 s** (`/usr/bin/python3` 3.13, numpy 2.4.4, rosbags installed).

### Demonstrated offline, in the cloud (model-conditional; synthetic inputs)
- **Closed forms:**
  - one look gives coverage = PD and the Bourgault value;
  - a same-sector repeat adds a look with no information;
  - an orthogonal look doubles ln Q;
  - a 45° look matches Paull Eq. 24, in the sequential form and with `paull_pairwise`;
  - the per-sector max rule; the PFA term;
  - contact log-odds after detected and missed looks; same-look replacement; one miss per look;
  - common-mode-aware fusion; both association gates;
  - disc mass against the Rayleigh closed form;
  - the disc query against the rendered-map product (v1 bug 2).
- **Determinism (T12):**
  - byte-identical `.npz` and manifest on repeated runs;
  - an mcap bag replayed twice is byte-identical, and equal to direct feeding.
- **Localisation (T12):** 4 synthetic targets, errors 0.09–0.49 m (≤ 2 m), one contact each.
- **Rejections:** all 18 reasons triggered and counted; 200 fuzzed malformed inputs, no exception.
- **Layouts:**
  - every C-3/C-4 field parsed from the `.msg` files by rosbags, with the contract types;
  - CDR round-trips of `BeliefGrid`, `QueryBelief` response, OccupancyGrid, MarkerArray and
    DiagnosticStatus;
  - the adapters read real `ProcessedSSSPing` (verbatim copy), `Detection2DArray`
    (`vision_msgs` upstream), `Odometry` (zero stamp) and `NavSatFix` (status −2).
- **Performance (indicative):** 0.55–0.65 ms per swath at 0.5 m (2 × ~1170 samples); snapshot
  encode ~4 ms for a 240 × 160 grid (3.5 MB).
- **Geodesy:** bit-identical to `BlueBoat-MCS/mcs/core/geo.py`, read as text.

### Assumed, not demonstrated (LOCAL_CHECKLIST)
- Every ROS runtime behaviour:
  - the build;
  - rclpy message filling (v1 bug 4 path);
  - QoS matching;
  - the live simulator run;
  - fault injection;
  - the real-recording replay;
  - laptop performance.
- **Placeholders:** the PD curve, PFA = 0, ρ (Paull's assumption), the prior.
- **Scores:** raw, so `p_exists` is not a probability (**priority**).
- **Coverage gating:** a heuristic on array stamps.
- **v1 comparisons:** [NOT AVAILABLE] (no v1).

### Open items
- `.claude/TODO.md`:
  - [P0] calibration;
  - [P0] first ROS run;
  - [P1] contract reconciliation, gating check, compass/odom offset, performance;
  - [P2] fitted models, the marginalised aspect model, polygon areas.
- `docs/REQUESTS.md`:
  - R-1 calibration and flag;
  - R-2 meta companion;
  - R-3 recording on real runs;
  - R-4 Control origin latch / zero stamps;
  - R-5 SSS altitude flag;
  - R-6 status gate;
  - R-7 Replanner / D7 coupling;
  - R-8 integration.

### Contract status (CONTRACT_PROVIDED.md)
- Every C-3/C-4 field is provided.
- **Deviations:**
  - D-C1: datum = the survey area's `geo_anchor`;
  - D-C2: configured class list;
  - D-C3: `first_confidence` is the raw score while uncalibrated;
  - D-C6: shadow is always unknown.
- **Added:** D-C4 `area_x/y`; D-C5 `look_detected`, `detection_ids` format.
- **Coupling:** the Replanner's emulator must publish one array per image and never run beside
  Perception.

### Proposed superproject text (for the owner; this session edits nothing outside the module)

**Root `.claude/CLAUDE.md` §1, module table: replace the Belief part of the "Belief · Replanner ·
Evaluation" row with:**

> | **BlueBoat-Belief** — `blueboat_belief` + `blueboat_belief_interfaces` | The belief map (D4), built
> 2026-11-04, **not yet run in a ROS graph**. Reads `/sss_processor/processed`, `/sss_ai/detections`,
> `/blueboat/odom`, `/mavros/global_position/global` and a survey-area file (a world `metadata.yaml`
> or `blueboat_survey_area/1`). Publishes `/belief_map/belief_grid` (`BeliefGrid`, contract
> `belief_grid/0`, RELIABLE + TRANSIENT_LOCAL, 1 Hz) and `/belief_map/query_belief`, display grids,
> contact markers and `/diagnostics`. Snapshots go under `<root>/data/Belief_data/`. ROS-free
> deterministic core; offline replay `blueboat-belief-replay`. Scores uncalibrated: `p_exists` is not a
> probability until Perception calibrates |

**Root §1 chain bullet and CM-9: amend "Belief reads only Perception's per-image records" to:**

> Belief reads the processed pings (`/sss_processor/processed`), Perception's detections
> (`/sss_ai/detections`), odometry and GPS, plus the mission's survey area and anchor (owner
> decision 2026-11-04: the Belief brief governs Belief's design). It sends to the Replanner
> (`BeliefGrid`) and to displays and diagnostics only. The Replanner reads only Belief.

**Root §4.1 / §4.3: add `BlueBoat-Belief` as a consumer** in the rows
`/sss_processor/processed` (BEST_EFFORT), `/sss_ai/detections`, `/blueboat/odom` and
`/mavros/global_position/global` (BEST_EFFORT). Add new rows `/belief_map/belief_grid`,
`/belief_map/query_belief`, `/belief_map/p_target`, `/belief_map/coverage`,
`/belief_map/contacts` and `/diagnostics` (producer `belief_map`), with the QoS of `topics.md` §1.
§4.7 ("no `/diagnostics` topic in any module") then needs updating.

**Root §2 build order:** `blueboat_belief_interfaces` is built before `blueboat_belief`
(`colcon build --packages-up-to blueboat_belief`); the latter also needs `blueboat_interfaces`
and `vision_msgs`.

**CM-1:** no change needed. Root `CLAUDE.md:422` already allows new modules to own their
interfaces package (brief §12.6 is satisfied upstream).

**Skill `module-gates`, new row:**

> | `BlueBoat-Belief` | `python3 -m test.smoke_test` from `BlueBoat-Belief/blueboat_belief/` (offline, no ROS):
> **188 checks** (2026-11-04, cloud). Sections [10] and [18] need `pip install rosbags` (visible SKIP
> otherwise); [1]'s MCS bit-identity check needs the `BlueBoat-MCS` checkout beside it. Fail-fast
> `check()`; no lint or type-checker | survives a clone (`test/` tracked) |

**Skill `file-interfaces`, new rows:**

> | Belief snapshots `<root>/data/Belief_data/<YYYY_MM_DD-HH_MM_SS>/snapshot_NNNN.npz` + `.json`, `snapshot_final.*` | `belief_map` / `blueboat-belief-replay` → D7 evaluation, analysis | byte-deterministic npz (all `BeliefGrid` arrays + `contacts_json`); manifest `blueboat_belief/snapshot/1` (config, model params, counters, input recording, sha256; wall clock only under `provenance`); O_EXCL, never overwritten (CM-7) |
> | Belief recorded inputs `<root>/data/Belief_data/<stamp>-inputs/` (mcap) | `belief_launch.py record_inputs:=true` → `blueboat-belief-replay` | the four input topics; write-once |
> | Survey-area file `blueboat_survey_area/1` | operator / future tool → Belief | `geo_anchor` + `limits {local, corners_gps}` (the world `metadata.yaml` shape); spec `BlueBoat-Belief/blueboat_belief/docs/SURVEY_AREA_FORMAT.md` |

## Phase 2 (design): 2026-11-04, delivered, waiting for the owner's answers (STOP gate)

**What exists.** Phase 1 docs, plus:
- `docs/PHASE2_DESIGN.md`:
  - architecture;
  - input handling and rejection reasons;
  - odom → map fit;
  - detector-coverage gating;
  - model summary (the future `MATH.md`);
  - survey-area file format;
  - proposed `BeliefGrid` / `BeliefContact` / `QueryBelief` text;
  - config table;
  - gate plan [0]–[16];
  - `CLAUDE.md` outline;
  - [ANALYSE] verdicts;
  - scientific-model table;
  - questions Q-1 … Q-14.
- `docs/CONTRACT_PROVIDED.md`: field-by-field conformance (planned).
- `docs/CONTRACT_DEVIATIONS.md`: D-1 framing, D-C1 … D-C6, and the emulator coupling notes.

**Gate.** None yet (0 checks): no code.

**Contract.** Every C-3/C-4 field is planned. Deviations are D-C1 (datum source), D-C2 (class
list), D-C3 (`first_confidence` raw while uncalibrated), and [ADDED] D-C4/D-C5. Shadow is degraded
(D-C6).

**Next.** The owner answers Q-1 … Q-14. Then Phase 3 build in this order:
1. interfaces package, `CLAUDE.md`, `TODO.md`, `MATH.md`;
2. core, component by component, with the gate run after each;
3. adapters, node, launch;
4. docs.

## Phase 1, revision b (understand): 2026-11-04, complete

**What exists.**
- `README.md` (from `3ba9281`).
- `blueboat_belief/docs/PHASE1_MEMO.md` (rev. b): the findings for the brief's four inputs.
- `blueboat_belief/docs/LOCAL_CHECKLIST.md`: L-1 … L-8.
- This file.
- No code, package or message definitions.

**Gate.** None yet (0 checks).

**Owner decisions taken this phase** (memo §1):
- **D-1** The brief wins on design where it disagrees with the root `CLAUDE.md` or the synthesis.
- **D-2** Inputs are the four topics: `/sss_processor/processed`, `/sss_ai/detections`,
  `/blueboat/odom`, `/mavros/global_position/global`. Image records are not an input.
- **D-3** odom → map comes from an online GPS/odom translation fit, with a config override.
- **D-4** The survey area is the world `metadata.yaml` limits in simulation, or a config file of
  the same shape on the real boat, documented for a future configuration tool.
- **D-5** There is no context pack and no v1. v1 comparisons are [NOT AVAILABLE].

**Deviations.**
- D-1 deviates from CM-9, root `CLAUDE.md` §1 and synthesis §5.1. It is to be recorded in
  `CONTRACT_DEVIATIONS.md` and `DESIGN.md` in Phase 2.
- Contract collisions X-1 … X-3 are listed in memo §6.

**Open for Phase 2.** The memo §8 questions, asked with recommendations at the gate.

**Superseded.** Revision a (image-records analysis), summarised in memo Appendix A.

**Next.** Phase 2 design (plan mode, no edits), then STOP for the owner's answers.
