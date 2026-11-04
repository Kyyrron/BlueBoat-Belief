# CONTRACT_DEVIATIONS: where the repository, the brief or the owner changed `belief_grid/0`

The embedded contract text in the session prompt is never edited. This file records every change
with its evidence and decision. It is a design-stage list (Phase 2); the decisions marked
"proposed" wait for the owner's answers (`PHASE2_DESIGN.md` §11).

## Project-level decision that frames the contract

**D-1 (owner, 2026-11-04): the brief wins on design.** Belief takes the brief's four inputs
(`/sss_processor/processed`, `/sss_ai/detections`, `/blueboat/odom`,
`/mavros/global_position/global`) and its full output set. This deviates from:
- root `.claude/CLAUDE.md:53-55` (§1);
- CM-9 (`:476-479`);
- `project_synthesis.md:216`.

All three say "reads **only** Perception's per-image records … sends **only** to the Replanner".

**Effect on the contract:**
- C-1 ("Belief subscribes to `/sss_ai/detections`") holds as written.
- The Replanner still reads only `BeliefGrid` (CM-9's consumer half is unchanged).

Proposed root-document wording comes at Phase 5. This session edits nothing outside the module.

## Field-level deviations

| # | Contract item | Change | Evidence | Decision |
|---|---|---|---|---|
| D-C1 | C-3 `datum_lat/lon` = "the mission `geo_anchor`" (provisional) | Datum = the **survey area's** `geo_anchor`: a world `metadata.yaml` in simulation, a `blueboat_survey_area/1` file on the real boat | No mission file carries an area. Deployed and session trajectories drop `geo_anchor` (`BlueBoat-MCS/mcs/designer/io_yaml.py:231-235`). In a built world, odom (0,0) = the world anchor (`BlueBoat-SSS-Sim/.../ros/mavros_shim_node.py:120-133`) | Proposed (Q-2). Consumer impact: none; it converts via the datum |
| D-C2 | C-3 `class_names` default (block, pipe, cage, tyre, plate) | A configured list plus `other`. Detections with other names go to `other` (counted) | YOLO classes are the training worlds' object types (Perception `core/detectors/yolo.py:175-187`); the dummy is `dummy_center` | Proposed (the contract says "Belief's list wins") |
| D-C3 | C-4 `first_confidence` = "calibrated confidence q" | The creating detection's `score`, which is **raw while uncalibrated**. `model_params_json.detector_calibrated` (config, default `false`) says which | No `calibrated` flag on `/sss_ai/detections` (Perception `ros/convert.py:47-75`); `calibrated: false` today (`core/calibration.py:31-40`) | Proposed (Q-8) |
| D-C4 | C-3 (none) | `area_x`, `area_y` (survey polygon) [ADDED] | Synthesis §5.1: Belief carries the survey area so the Replanner needs no other input | Proposed (Q-13); optional for the consumer |
| D-C5 | C-4 optional look history | `look_detected` (bool[]) [ADDED]; `detection_ids` formatted `"<array stamp ns>:<id>"` | Perception ids restart at every GCS START (`core/pipeline.py:69-…`, `station_uid`), so a bare id is not unique | Proposed |
| D-C6 | C-4 `shadow_known/has_shadow` | Always `false/false` (degraded, as C-6 allows) | No shadow information on any input; meta companion requested (`REQUESTS.md`) | Per contract C-6 |

## Coupling notes for the in-machine reconciliation (not contract changes)

- **The Replanner's dev-only emulator publishes on `/sss_ai/detections`.**
  - It must never run together with Perception, which would make two publishers.
  - Belief gates negative evidence on received arrays (`PHASE2_DESIGN.md` §2.3), so the emulator
    must publish **one array per image, empty ones included**, stamped with the newest covered
    ping time. Otherwise Belief must run with `coverage.require_detector: false`, labelled.
- The emulator's class names must be in Belief's configured `class_names`, or they land in `other`.
- **Model keys a predictor must read (2026-11-04).** A consumer reproducing Belief's predictions
  must read `model_params_json.rho_model.ordering` and `model_params_json.turn_gating`, and
  record that in its `CONTRACT_CONSUMED.md`.
  - The default ordering is now `paull_pairwise`: Paull Eqs. 22–24 with the larger confidence
    first. It was `sequential` before.
  - Under turn gating, back-scanned inner-side samples (beyond `r* = v/|ω|`) add no look.
  - Neither changes any field name or type.
