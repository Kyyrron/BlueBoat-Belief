# CONTRACT_PROVIDED: `belief_grid/0`, producer side (BlueBoat-Belief)

**Status: implemented 2026-11-04 (cloud, offline gate 188/188); not yet run in a ROS graph.**

Status values:
- `[MATCH]`: the contract name and dtype, as specified.
- `[DEVIATION]`: see `CONTRACT_DEVIATIONS.md`.
- `[ADDED]`: an extra field, optional for the consumer.
- `[NOT PROVIDED]`: absent.

Tests are sections of `test/smoke_test.py`:

| Test | What it checks |
|---|---|
| [9] | the `to_grid_dict` names, dtypes, sizes and NaN rules |
| [10] | `.msg` parse against `GRID_FIELDS` / `CONTACT_FIELDS`, and a CDR round-trip |
| [18] | a recorded-bag replay |

The C-7 dictionary is `blueboat_belief.core.contract.to_grid_dict(belief, grid_seq, stamp)`. The
reference field tables `GRID_FIELDS` / `CONTACT_FIELDS` are in the same module.
`blueboat_belief.ros.adapters.grid_to_fields()` builds the message **only** from that dict.

## C-2: topic, QoS, publication

| Item | Contract | Implemented | Status |
|---|---|---|---|
| Topic | `/belief_map/belief_grid` | node `belief_map`, publisher `~/belief_grid` (`ros/belief_node.py`) | [MATCH] |
| Type | `blueboat_belief_interfaces/BeliefGrid` | `blueboat_belief_interfaces/msg/BeliefGrid.msg` | [MATCH] |
| QoS | RELIABLE, TRANSIENT_LOCAL, KEEP_LAST 1 | `latched` profile in `BeliefNode.__init__` | [MATCH] (B-2 checks it live) |
| Republication | periodic, default 1 Hz, latched | timer `publish.grid_rate_hz`; each publish `grid_seq += 1` | [MATCH] |
| Query | `/belief_map/query_belief`, `QueryBelief` | `~/query_belief` → `Belief.query_disc` | [MATCH] |

## C-3: `BeliefGrid`

| Field | dtype | Implemented in | Status | Test |
|---|---|---|---|---|
| `header` | `std_msgs/Header` | stamp = `Belief.last_event_t`, `frame_id "map"` | [MATCH] | [9], [10] |
| `contract_version` | string | `contract.CONTRACT_VERSION` | [MATCH] | [9] |
| `grid_seq` | uint32 | node counter | [MATCH] | [9] |
| `width`, `height`, `resolution` | uint32, uint32, float32 | `area.grid_for_area` | [MATCH] | [2], [9] |
| `origin_x`, `origin_y` | float64 | `GridGeometry.origin_*` = `limits.local` x_min / y_min | [MATCH] | [2] |
| `datum_lat`, `datum_lon` | float64 | survey area's `geo_anchor` | [DEVIATION] source D-C1 | [2] |
| `odom_to_map_x/y/yaw` | float64 | `Aligner.tx/ty`; yaw 0 | [MATCH] | [3] |
| `odom_to_map_sigma_m` | float32 | `Aligner.sigma` (robust spread or override sigma) | [MATCH] | [3] |
| `odom_to_map_valid` | bool | `Aligner.valid` | [MATCH] | [3] |
| `aspect_sector_count`, `aspect_sector_origin_deg` | uint8, float32 | `aspect.*` config | [MATCH] | [5] |
| `class_names` | string[] | `detector.class_names` (+ `other`) | [DEVIATION] list D-C2 | [7] |
| `prior_density_per_ha` | float32 | `prior.density_per_ha` | [MATCH] | [9] |
| `model_params_json` | string | `contract.model_params` (schema 1). Keys: `pd_model`, `pfa`, `rho_model` (`ordering` default `paull_pairwise` since 2026-11-04, `fold_opposite`), `turn_gating`, `negative_evidence`, `detector_calibrated`, … | [MATCH] | [9] |
| `p_background` | float32[w·h] | `Grid.p_background` | [MATCH] | [6] |
| `p_target` | float32[w·h] | `Belief.render_p_target` | [MATCH] | [8] |
| `coverage` | float32[w·h] | `Grid.coverage` | [MATCH] | [6] |
| `n_looks` | uint16[w·h] | `Grid.n_looks` | [MATCH] | [6] |
| `n_pings` | uint32[w·h] | `Grid.n_pings` | [MATCH] | [6] |
| `n_aspects` | uint8[w·h] | `Grid.n_aspects` | [MATCH] | [6] |
| `depth_m` | float32[w·h] | `Grid.depth` (flat floor) | [MATCH] | [9] |
| `last_intensity_db` | float32[w·h] | `Grid.intensity` | [MATCH] | [9] |
| `max_confidence` | float32[w·h] | `Grid.max_conf` (0 inside if none) | [MATCH] | [9] |
| `aspect_counts` | uint8[w·h·S] | `Grid.aspect_counts` | [MATCH] | [6] |
| `aspect_log_miss` | float32[w·h·S] | `Grid.aspect_log_miss` (always provided) | [MATCH] | [6] |
| `area_x`, `area_y` | float64[] | survey polygon | [ADDED] D-C4 | [9], [10] |
| `contacts` | `BeliefContact[]` | `ContactTable.to_dicts` | [MATCH] | [7], [10] |

NaN rules: the probability channels are NaN outside the area only (checked in [9]). Inside, unseen
cells are `coverage 0`, `n_looks 0`, `aspect_counts 0`, `depth_m` / `last_intensity_db` NaN, and
`max_confidence 0`.

## C-4: `BeliefContact`

| Field | dtype | Implemented | Status |
|---|---|---|---|
| `uid` | uint32 | `ContactTable.next_uid`, never reused (checked in [7]) | [MATCH] |
| `x`, `y`, `covariance` | float64, float64, float64[4] | `Contact.x/y/P`, row-major | [MATCH] |
| `p_exists` | float32 | `σ(L)` | [MATCH] (not a probability while uncalibrated) |
| `state` | uint8 | 0 / 1 (`STATE_*` constants in the `.msg`) | [MATCH] |
| `class_probabilities` | float32[] | Dirichlet | [MATCH] |
| `sectors_detected`, `sectors_missed` | uint8[] | unique, sorted | [MATCH] |
| `n_looks_detected` | uint16 | | [MATCH] |
| `first_confidence` | float32 | the creating detection's score | [DEVIATION] semantics D-C3 |
| `first_sector` | uint8 | | [MATCH] |
| `max_confidence` | float32 | | [MATCH] |
| `shadow_known`, `has_shadow` | bool | `false/false` | [MATCH] degraded D-C6 |
| `created`, `updated` | `builtin_interfaces/Time` | event times | [MATCH] |
| `look_bearings_deg`, `look_ranges_m` | float32[] | per look | [MATCH] |
| `look_detected` | bool[] | per look | [ADDED] D-C5 |
| `detection_ids` | string[] | `"<array stamp ns>:<id>"` | [MATCH] (format D-C5) |

## C-5: conventions

All are [MATCH]: map ENU about the datum; geodesy bit-identical (checked in [1]); look bearing
`b = ψ + s·π/2`; sector formula on the directed bearing (checked in [5]); time on the
sonar-profile clock; units m / rad.

Two details are documented in `topics.md` §4:
- the sector angle is rounded to 1e-9° (an edge goes to the upper sector);
- the sample range `|y|` includes the processor's 0.285 m lever arm, and the PD argument is
  `|y| − 0.285`.
