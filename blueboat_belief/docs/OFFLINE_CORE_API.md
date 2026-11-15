# OFFLINE_CORE_API: driving the Belief core without ROS

This is the supported way to run the real Belief core from another program: an evaluation loop,
an analysis script, or the Replanner's `eval/belief_real.py`. Everything named here is ROS-free
(numpy and the standard library). The example below is executed by the gate (smoke [22]), so it
cannot drift from the code.

**What is stable.** The names in the table are the public surface. They change only with a
line in this file and in `topics.md` §10. Anything else (attributes starting with `_`,
`Grid` internals, `ContactTable` internals) may change without notice: read results through
`to_grid_dict`, never from the objects.

| Import | What it is |
|---|---|
| `blueboat_belief.core.config.BeliefConfig` | the configuration; `BeliefConfig().with_values(section={key: value})` |
| `blueboat_belief.core.area.make_area_dict`, `area_from_dict` | a rectangular survey area about a WGS-84 anchor |
| `blueboat_belief.core.belief.Belief` | the core: `ingest_ping`, `ingest_detections`, `ingest_record`, `ingest_odom`, `ingest_fix`, `finish` |
| `blueboat_belief.core.belief.PingObs`, `Det`, `DetArray`, `ImageRec`, `OdomObs`, `FixObs` | the input records |
| `blueboat_belief.core.geometry.processor_side_offsets` | the ground offsets `sss_processor_node` publishes for one side |
| `blueboat_belief.core.contract.to_grid_dict` | the output: every `belief_grid/0` field under its contract name |
| `Belief.counters`, `Belief.stats`, `Belief.summary()`, `Belief.health()` | run health: what was rejected, applied and released (§5) |

## 1. Minimal example

```python
import numpy as np

from blueboat_belief.core.area import area_from_dict, make_area_dict
from blueboat_belief.core.belief import Belief, Det, DetArray, PingObs
from blueboat_belief.core.config import BeliefConfig
from blueboat_belief.core.contract import to_grid_dict
from blueboat_belief.core.geometry import processor_side_offsets

LAT0, LON0 = 34.5, 133.5                             # the map datum (any WGS-84 point)
area = area_from_dict(make_area_dict(LAT0, LON0, 0.0, -30.0, 100.0, 30.0, name="example"))
cfg = BeliefConfig().with_values(
    # the caller's poses are already in the map frame: odom (0, 0) is the datum, no GPS needed
    alignment={"origin_lat": LAT0, "origin_lon": LON0, "override_sigma_m": 0.0},
    # the caller's detector sees every ping it feeds (labelled "ungated" in model_params_json)
    coverage={"require_detector": False})
belief = Belief(cfg, area)

h = 2.0                                              # transducer altitude above the seabed [m]
arm = -0.345                                         # transducers' fore/aft arm from (x, y), + forward [m]
port, stbd = processor_side_offsets(h, +1), processor_side_offsets(h, -1)
db_p = np.full(port.size, -20.0, dtype=np.float32)   # intensities are display-only
db_s = np.full(stbd.size, -20.0, dtype=np.float32)
t0, speed, yaw = 100.0, 0.5, 0.0                     # one pass heading East along y = 0

for i in range(2001):                                # 20 pings per second for 100 s
    t = t0 + i / 20.0
    x = speed * (t - t0)
    belief.ingest_ping(PingObs(t=t, x=x, y=0.0, yaw=yaw, water_depth=h + 0.22,
                               port_ping_number=i + 1, starboard_ping_number=i + 1,
                               port_y=port, port_db=db_p, starboard_y=stbd, starboard_db=db_s,
                               transducer_x_offset=arm))
    if i == 800:                                     # a detection 8 m to port of this ping
        det = Det(id="1", t_row=t, x=x, y=8.0, theta=yaw, class_name="block", score=0.8,
                  cov=np.array([[1.09, 0.0], [0.0, 1.09]]))
        belief.ingest_detections(DetArray(t=t, detections=(det,)))

belief.finish()                                      # end of input only: closes every look
grid = to_grid_dict(belief, grid_seq=1)              # dict of the belief_grid/0 fields
```

`grid` holds numpy arrays in the message dtypes, and `grid["contacts"]` is a list of dicts. A
snapshot can be taken at any time during the run with `to_grid_dict`; it does not change the
belief.

## 2. Rules for the inputs

1. **Times** are explicit floats in seconds on one clock, strictly positive. A step backwards of
   more than `look.time_reset_s` (5 s) starts a new epoch.
2. **Frames.** `PingObs.x, y, yaw` and `Det.x, y` are in the odom frame (ENU metres, yaw in
   radians counter-clockwise from East). With the alignment override above, the odom frame is
   the map frame. Without it, feed `OdomObs` and `FixObs` and wait for `odom_to_map_valid`.
3. **Ping geometry.** `PingObs.x, y` are the navigation pose. `port_y > 0`, `starboard_y < 0`,
   near to far, measured from the boat's centre line: they already contain the 0.285 m lateral
   lever arm. `transducer_x_offset` (the last field, default 0.0) is the transducers' fore/aft
   arm from (x, y) along the heading, + forward: the processor's `ProcessedSSSPing.transducer_x_offset`,
   −0.345 m since 2026-11-15. Belief places the samples from `(x + a cos yaw, y + a sin yaw)`
   (`MATH.md` §1 items 2 and 7). With the default 0.0 a caller's placement is the one it had before
   the field existed. `water_depth` is the altitude plus 0.22 m; a `water_depth` of exactly 0.22
   means "altitude unknown" and the row is dropped. A side is absent when its `*_ping_number` is
   0.
4. **Ping rate.** One look of a cell is a run of pings from one aspect sector with gaps of at
   most `look.timeout_s` (20 s). Pings must therefore be denser than that along a pass. They do
   not have to be 20 Hz: a 0.5 m cell needs at least one ping while the swath crosses it, so keep
   the along-track step at or below the cell size (`grid.resolution_m`).
5. **Detections.** `Det.t_row` is the acquisition time of the row the detection lies on, and a
   ping at or near that time must already have been fed (the pose is looked up within 0.25 s).
   `Det.cov` is the 2 × 2 position covariance in m²; Perception's includes σ_nav = 1 m, which
   Belief treats as common-mode. An invalid covariance is replaced by the propagation model.
   `class_name` must be one of `detector.class_names`, otherwise the detection is kept under
   `other` and `stats.detections_class_other` counts it.
6. **`finish()`** closes every open look and applies the misses of contacts that were looked at
   and not detected. Call it once, at the end. Calling it in the middle of a pass and then
   feeding the same pass again would count that pass as two looks.

## 3. Choosing how misses are credited

| Mode | Configuration | Behaviour | Use it when |
|---|---|---|---|
| ungated | `coverage={"require_detector": False}` | every ping counts as searched at once; a detection is applied when it is fed | the caller's own detector model decides what was searched (a dev loop with an emulator) |
| gated on arrays | default, feed one `DetArray` per image, **empty ones included** | a ping is credited once two arrays bracket it (`MATH.md` §8.1) | reproducing the live behaviour with a detections-only publisher |
| gated on records | default, feed one `DetArray` and then one `ImageRec` per image | exact image spans; `searched=False` images are not credited (`MATH.md` §8.2) | reproducing the live behaviour with Perception |

In the gated modes the belief lags the input by one to two image strides, exactly as it does
live. `grid["header"]["stamp"]` is the evidence horizon: every ping at or before it is in the
grid, and none after it. A planner that must not count a look twice compares its own look times
with that stamp.

`model_params_json` in the grid states which mode produced it (`negative_evidence`,
`coverage_source`). A result computed in the ungated mode must be labelled as such.

## 4. What the output means

The field-by-field contract is `topics.md` §3. Three points matter to a predictor:

1. **Searched sectors.** A sector of a cell has been searched iff `aspect_log_miss[cell·S + s] < 0`.
   `aspect_counts > 0` also counts looks that had no detector coverage, so it is a superset.
   The dependence factor ρ of a new look is taken against the searched sectors only.
2. **Scores.** `first_confidence`, `max_confidence` and `p_exists` are probabilities only when
   `model_params_json.detector_calibrated` is true. Today it is false.
3. **Coverage is order-dependent beyond two looks** (`MATH.md` §7). It is a planning aid, not a
   metric for comparing survey policies.

## 5. Run health: what Belief dropped, applied and released

Read-only, at any time, also after `finish()`. The key sets below are stable (smoke [22] checks
this page against the code); the values are the same as on `/diagnostics` and in the snapshot
manifest (`topics.md` §8–§9).

**`Belief.counters`**: `dict[str, int]`, the number of inputs (or parts of inputs) **not used**,
by reason (meaning of each: `topics.md` §9). A mission that lost nothing has `coverage_expired`,
`stale_detection`, `future_detection` and `no_pose_at_stamp` at 0.

Keys: `nan_input`, `frozen_pose`, `altitude_unknown`, `time_reset`, `no_pose_at_stamp`,
`outside_area`, `bad_covariance`, `alignment_pending`, `alignment_queue_overflow`, `no_fix`,
`ignored_class`, `unknown_class`, `duplicate_detection`, `stale_detection`, `future_detection`,
`zero_stamp_array`, `coverage_expired`, `empty_ping`, `bad_record`.

Two of them are not losses: `alignment_pending` counts observations queued until odom → map was
valid (then applied), and `ignored_class` counts the dummy detector's detections.

**`Belief.stats`**: `dict[str, int]`, what **was** applied. A ping waiting for its images is not
counted yet; after `finish()` every fed ping is either counted in `pings` or in a counter.
`detections_class_other` counts detections kept under `other` because their class is not in
`detector.class_names`; `records_overlap` is non-zero only if Perception's images overlap more
than their two neighbours (`MATH.md` §8.2).

Keys: `pings`, `pings_covered`, `pings_geometry_only`, `arrays`, `detections`,
`contacts_created`, `fixes`, `odom`, `backscan_pings`, `backscan_samples`,
`detections_class_other`, `records`, `records_searched`, `records_unsearched`,
`records_without_array`, `records_overlap`.

**`Belief.summary()`**: `dict`, run-level numbers. `area_cumulative_pd` is Bourgault 2006 Eq. 7
for one target uniform in the area (a planning aid, `MATH.md` §13 item 5).
`transducer_x_offset_min_m` and `transducer_x_offset_max_m` are the range of the fore/aft arms
the applied pings were placed with (NaN before any ping); equal values mean one arm.

Keys: `area_cumulative_pd`, `contacts`, `contacts_confirmed`, `contacts_retired`,
`alignment_valid`, `transducer_x_offset_min_m`, `transducer_x_offset_max_m`.

**`Belief.health()`**: `(level, message)`, with level `"OK"`, `"WARN"` (odom → map not valid
yet, the simulator anchor check failed, or observations were dropped before alignment) or
`"ERROR"` (poses frozen at (0, 0)).
