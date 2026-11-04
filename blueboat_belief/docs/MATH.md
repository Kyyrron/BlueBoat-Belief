# MATH: the specification of BlueBoat-Belief's model

This file is the specification: each equation names the function that implements it and the
smoke-test section that checks it (`test/smoke_test.py [N]`). Results computed with this model are
**model-conditional** (CM-13). The PD curve, PFA and ρ below are **placeholders** until fitted
(brief §8.5, §17.3).

For every quantity, §0 gives:
- the event;
- the conditioning variables;
- the evidence or source;
- the independence assumptions;
- whether PFA is modelled;
- the validity range;
- whether it is a literature result or a project decision (brief §17.12).

> **Priority (owner, 2026-11-04): detector calibration.** Perception's calibration is the identity
> today (`calibrated: false`), and `/sss_ai/detections` carries no calibration flag. Every equation
> below that uses the score `q` as a probability (§9, §10) is therefore applied to an
> **uncalibrated score**. `p_exists` is then a score-derived index, **not a probability**, until the
> config key `detector.calibrated` is set true for a calibrated detector. `model_params_json`
> publishes the flag.

## 0. Quantity table

| Quantity | Event / meaning | Conditioning | Evidence / status | Independence / PFA | Implemented in |
|---|---|---|---|---|---|
| `p0` | ≥ 1 target in the cell before any look | cell area Δ², site density ρ_T | project placeholder (5 /ha) | Poisson rare-event approximation; cells independent | `models.prior_cell` |
| `PD(r)` | the detector emits an accepted detection (score ≥ 0.10 upstream) of a present target in one look | ground range from the transducer only | **placeholder** (brief §10). Range-only curves are SSS practice: Paull 2013 Fig. 2 P(y) from ESPRESSO; Yordanova & Gips 2020 Eq. 2, SAS. Both papers read in the primary text |  aspect, class, seabed omitted | `models.pd_range` |
| `PFA` | the same chain emits a detection in a target-free cell in one look | — | **neglected** (`pd.pfa` = 0). Bourgault 2006 §4.1 assumes "perfect discrimination i.e. no false target detection" (primary text); its miss update (Eq. 4) is the PFA = 0 special case | stated assumption | `models.miss_log_factor` |
| `q` | the detector's score for a candidate | candidate box, class | raw while `detector.calibrated` is false (priority above) | — | `contacts` |
| `ρ` | the share of a new look's information not already given by earlier looks | angle between look lines | Paull et al. 2013 §III-C Eqs. 22–24: Paull's own **assumption** ("it is assumed"). Fawcett 2010 is cited for the value of multiple views, Williams / Rajala & Edwards for the dependent / independent extremes. Primary text read | linear in angle | `models.rho_table` |
| `Q`, coverage | no detection in any look so far, given a target | the looks' PD and ρ | Bourgault 2006 Eqs. 6–7 (`Q_k = ∏q_i`, `P_k = 1 − Q_k`), here per cell (conditioned on the target in that cell); Yordanova & Gips 2020 Eq. 2 (residual risk per grid point, SAS) | conditional independence discounted by ρ; PFA = 0 | `grid.Grid.observe_ping` |
| `Q_s` | the same, within aspect sector s | sector s | Paull's α = 0 rule (`c_total = max`) | full dependence within a sector | `grid.Grid.observe_ping` |
| `p_bg` | an **undetected** target in the cell | misses only | Bourgault 2006 miss update in log-odds; clamping (Thrun et al. 2005) | independent cells (Thrun 2003 caveat) | `grid.Grid.p_background` |
| `p_exists` | contact c is a real target | its looks | Bernoulli-contact idea (Mahler 2007); **not** an RFS filter | nearest-neighbour association; no clutter model | `contacts.Contact` |
| `P_c` | uncertainty of contact c's position | its detections, common-mode, alignment | Perception `geometric_v1` (assumed sigmas) + floor | common-mode error removed before fusion, added once | `contacts.Contact.covariance` |

## 1. Frames, sides, sectors

These implement contract C-5 (`geometry.py`). Notation:
- ψ is the boat yaw (rad, ENU, CCW from East), from `ProcessedSSSPing.robot_orientation`;
- s = +1 is port and s = −1 is starboard;
- `y_i` are the ping's signed ground offsets (`port_y` > 0, `starboard_y` < 0), measured from
  base_link with the processor's lever arm (0.285 m) already included;
- t = (t_x, t_y) is the odom → map translation (§3).

1. **Look bearing**: `b = ψ + s·π/2`.
2. **Sample position (map frame)**: `p_i = (x + t_x − y_i sin ψ, y + t_y + y_i cos ψ)`. This is
   identical to `p + |y_i|·(cos b, sin b)` and to the processor's own geometry. It is implemented
   by `geometry.sample_xy` and checked in [5].
3. **Ground range from the transducer** (the PD argument): `r_i = |y_i| − a`, with
   a = `pd.lever_arm_m` = 0.285.
4. **Sector**: `k = floor(mod(deg(b) − o, 360) / (360/S))`, with S = `aspect.sector_count` and
   o = `aspect.origin_deg`. It is implemented by `geometry.sector_of` and checked in [5].
   `deg(b) − o` is rounded to 1e-9° before the floor, so a bearing exactly on a sector edge
   lands in the upper sector: `degrees(radians(30))` is 29.999999999999996 in floating point.
5. **Side of a detection** (`geometry.side_of_point`):
   `s = sign(cos ψ·(Y − y) − sin ψ·(X − x))`, with (x, y, ψ) the boat pose at the detection's row
   time. The detection's look sector is `sector_of(ψ + s·π/2)`.
6. **Cell of a point** (`area.GridGeometry.cell_of`):
   - `col = floor((X − origin_x)/Δ)`, `row = floor((Y − origin_y)/Δ)`;
   - index `i = row·W + col`;
   - row 0 is the south edge.

## 2. Prior

`λ = ρ_T·Δ²/10⁴`, and `p0 = λ` (the default) or `p0 = 1 − e^{−λ}` (`prior.exact_poisson`).
At 5 /ha and Δ = 0.5 m the two differ by 7.8e-9 (6e-5 relative). Implemented by
`models.prior_cell`; checked in [6].

## 3. odom → map

Implemented by `alignment.Aligner`; checked in [3].

1. Valid fixes only: `status ≠ −1`, `lat` and `lon` not both 0, all values finite.
2. `EN_k = latlon_to_local_en(lat_k, lon_k; datum)`, using the MCS pair (`geo.py`, asserted
   bit-identical in [1]).
3. Pair each fix with the odom position at the fix time (`pose.PoseBuffer.at`, tolerance
   `pose.stamp_tolerance_s`).
4. `d_k = EN_k − odom_k`. Over the last `alignment.window` pairs: `t = median_k(d_k)` per axis, and
   the per-axis spread `σ_a = 1.4826·median|d − t|`.
5. `σ = sqrt((σ_x² + σ_y²)/2)`. It is not divided by √n, because GPS errors are time-correlated:
   a conservative choice.
6. The fit is valid once `n ≥ alignment.min_pairs` and `σ ≤ alignment.max_spread_m`. It then
   latches (`alignment.latch`).
7. **Override**: `alignment.origin_lat/lon` gives `t = latlon_to_local_en(origin; datum)`,
   σ = `alignment.override_sigma_m`, and it is valid at once.
8. Yaw is 0: the real-boat odom yaw is absolute ENU (`robot_interface.py:627-639`).

## 4. Per-ping cell evidence

Implemented by `grid.Grid.observe_ping`; checked in [5] and [6].

1. A ping side contributes samples `p_i` inside the survey area.
2. Per cell j and ping:
   - `PD_j = max_{i∈j} PD(r_i)`;
   - `I_j = mean_{i∈j} dB_i`;
   - `D_j = water_depth` (flat floor, labelled).
3. **Unknown altitude**: the processor publishes `water_depth = 0 + 0.22` exactly, and `y_i` is
   then slant range. Such rows are **rejected** (`altitude_unknown`).
4. **PD placeholder** (`models.pd_range`):
   `PD(r) = P·clip((r − (r_lo − w))/w, 0, 1)·clip(((r_hi + w) − r)/w, 0, 1)`, with
   P = 0.7, r_lo = 3 m, r_hi = 25 m, w = 3 m.
5. `n_pings_j += 1` per (cell, ping), on every released ping, covered or not.
6. **Turn back-scan gating** (`turn.TurnEstimator`; checked in [19]).
   - Computed from the poses of processed pings, in time order, over `turn.window_s` (1 s,
     Perception's `turn_window_s`): `ω = Δψ_unwrapped/Δt` and `v = |Δp|/Δt`.
   - When `|ω| ≥ turn.min_yaw_rate_rad_s`, the boat is turning: `r* = v/|ω|` (0 below
     `turn.min_speed_mps`), and the inner side is port when ω > 0.
   - Inner-side samples with `|y_i| > r*` are **excluded** from looks, aspects and coverage, and
     counted (`stats.backscan_*`).
   - A ping gap longer than the window restarts the estimate.
   - Evidence: Paull 2013 §II-C, primary text: "areas on the outside of the turn are missed
     completely … areas on the inside of the turn can become completely distorted. In both
     cases, it becomes very difficult for automatic target recognition (ATR) systems…".
   - The radius `r*` is Perception's `backscan_from_m` (Cobra et al. 1992, cited by Perception,
     not read here).
   - Without the gate, the sweeping heading of one turn would give back-scanned cells several
     sectors, i.e. fake aspects. Detections are not gated.

## 5. Looks

Implemented in `grid.Grid.observe_ping`; checked in [6].

A **look** of cell j is a maximal run of observations from one sector k with gaps
≤ `look.timeout_s` (20 s) within one epoch.

- A new look starts on a sector change, on a timeout, or on a new epoch.
- A new epoch starts when time jumps backwards by more than `look.time_reset_s` (5 s), for example
  on a bag restart.
- Smaller backward steps are processor bursts: accepted, and they do not open a look.

At the opening of a look:
- `n_looks_j += 1`, saturating;
- `aspect_counts_{j,k} += 1`, saturating at 255;
- the look stores `ρ_j` (§6, against the sectors already **searched**, i.e. with a covered look),
  `lnQ_open` and `ℓ_open`.

**Coverage (§8 gating).** Only *covered* pings carry miss evidence. Within a look the contribution
is that of the **best** covered PD, `PD*` (same-look replacement):
- replacing `PD_old*` by `PD_new*` is exact, because the look's terms are recomputed from
  `lnQ_open` / `ℓ_open`;
- the result equals applying the look once at its close.

## 6. Dependence factor ρ (placeholder)

Implemented by `models.rho_table`; checked in [6].

1. Sector centres are `c_k = o + (k + ½)·360/S`.
2. `Δ(k, k') = min(|c_k − c_k'| mod 360, 360 − …)` ∈ [0°, 180°].
3. With `rho.fold_opposite` (the default, owner 2026-11-04), `Δ ← min(Δ, 180° − Δ)` ∈ [0°, 90°]:
   Paull's α, the angle between insonification **lines**.
4. `ρ(k | M) = min_{k'∈M} clip(Δ(k, k')/θ, 0, 1)`, where M is the set of searched sectors and
   θ = `rho.independence_angle_deg` (90°). `ρ(k | ∅) = 1`.
5. The minimum over earlier sectors is a heuristic: Paull defines only the pair (§III-C:
   "Let two confidences obtained from subsequent passes…"; brief §17.4).
6. Paull computes α from the actual insonification angles θ₁, θ₂. Belief uses sector centres, so
   ρ moves in steps of 1/3 at S = 12. Bourgault 2006 §4.3 warns that a stepped detection model
   makes a planner's objective jagged. Bearing-resolved ρ is a [P2] contract addition
   (`TODO.md`), not built.

## 7. Aggregate miss and background (the PFA = 0 special case, stated)

Implemented by `models.combine_miss` and `grid.Grid.observe_ping`; checked in [6].

**`rho.ordering: paull_pairwise`** (the default since 2026-11-04: owner decision, after reading
Paull §III-C). With `c_acc = 1 − Q_open`, the accumulated coverage at look open:

    c_hi = max(c_acc, PD*)
    c_lo = min(c_acc, PD*)
    Q = (1 − c_hi)(1 − ρ·c_lo)
    ℓ = ℓ_open + (lnQ − lnQ_open) − ln(1 − PFA)

This is Paull Eqs. 22–24 with the larger confidence first ("Without loss of generality, we can
assume that c¹ ≥ c²"):
- parallel (ρ = 0): `max(c₁, c₂)`, Eq. 22;
- perpendicular (ρ = 1): `1 − (1 − c₁)(1 − c₂)`, Eq. 23;
- in between: `c₂·ρ·(1 − c₁) + c₁`, Eq. 24.

Applied look by look against the accumulated coverage, it is exact for two looks and a
heuristic beyond.

**`rho.ordering: sequential`** (a switch: the brief's §6.4 effective-PD form):

    lnQ  = lnQ_open + ln(1 − ρ·PD*)
    ℓ    = ℓ_open + ln((1 − ρ·PD*)/(1 − PFA))
    coverage = 1 − Q

It equals the pairwise rule whenever `c_acc ≥ PD*`. It **departs from Paull** when the new look
is stronger:
- a stronger same-sector look adds nothing (Paull: `max`);
- 0.3 then 0.7 at ρ = ½ gives 0.545 (Paull: 0.745).

**Closed forms** (checked in [6]):
- one look from fresh: `coverage = PD`;
- `p⁺ = p(1−PD)/(p(1−PD) + 1 − p)` (Bourgault 2006, in log-odds);
- same-sector repeat at equal or lower PD: ρ = 0, so a look is added but no information;
- same-sector repeat at **higher** PD: coverage rises to that PD (pairwise, Eq. 22), but is
  unchanged under `sequential`;
- orthogonal second look at equal PD: `lnQ` doubles;
- a 45° (folded) look with c₁ ≥ c₂: ρ = ½, `c = c₁ + ½·c₂(1 − c₁)` (Eq. 24; both orderings);
- 0.3 then 0.7 at ρ = ½: 0.745 pairwise, 0.545 sequential.

The grid receives misses only. With PFA = 0 (the default) ℓ therefore never increases, and the
clamp `p ∈ [p_min, p_max]`, applied at read (`Grid.p_background`), gives the same values as
clamping at update (Thrun et al. 2005).

With PFA > 0, a covered look whose PD is below PFA raises ℓ slightly: `ln((1 − PD)/(1 − PFA)) > 0`,
the correct likelihood ratio. Clamping at read and at update then differ only if ℓ reaches the
upper clamp, which misses alone cannot do.

**Per-sector miss** (brief §8.1 minimal fix): `aspect_log_miss_{j,k} = ln(1 − max covered PD in
sector k)`. That is Paull's α = 0 rule (`c_total = max`). It is 0 for an unsearched sector inside
the area and NaN outside.

## 8. Detector coverage: when a non-detection is evidence

Implemented by `coverage.CoverageTimeline`; checked in [6] and [14].

1. Arrays `a_n` arrive with stamps `t_n` = the newest row of an image. Images are 256 rows at
   stride 128 (`seabed_imager.py:34`).
2. When array n arrives, every ping at or before `t_{n−1}` has been seen by all the images it
   belongs to (images n−1 and n). The pings with `t ≤ t_{n−1}` are therefore **released**.
3. A released ping at τ is **covered** iff consecutive stamps `t_{m−1} < τ ≤ t_m` exist with:
   - `t_m − t_{m−1} ≤ coverage.max_array_gap_s`;
   - neither array holding a class in `detector.ignore_classes` (the dummy is not a detector).
4. A ping still pending after `coverage.wait_s` is released **geometry-only**: it updates
   `n_pings`, looks and `aspect_counts`, but not `Q` or ℓ (`coverage_expired`).
5. Released detections are interleaved with the released pings in time order. A detection at τ is
   applied after the pings at τ.
6. `coverage.require_detector: false` (development only) releases every ping immediately as
   covered, and `model_params_json.negative_evidence` = `"ungated"`.

## 9. Contacts: existence (Bernoulli-inspired)

Implemented by `contacts.ContactTable`; checked in [7]. π = `detector.base_rate`.

1. **Association.** Map-frame detection `z`, covariance `R`. Candidates are the contacts with
   `‖z − x_c‖ ≤ association.gate_m` (2 m) **and**
   `(z − x_c)ᵀ(R + P_c)⁻¹(z − x_c) ≤ association.chi2` (9.21). Pick the smallest Mahalanobis
   distance, and break ties by the smaller uid. With no candidate, a contact is created with the
   next uid (monotonic, never reused).
2. **Contact looks** follow §5: sector, timeout, epoch.
   - A contact is *observed* by a covered ping whose swath contains the contact's cell.
   - `PD_c*` is the best covered PD at that cell in the look.
3. **Creation look**: `L = logit q*`, with q* the best score in the look (§5 replacement).
4. **Later detected look**: `L = L_open + ρ_c·(logit q* − logit π)`, with
   `ρ_c = ρ(k | M_c)` against the contact's sectors that had a look.
5. **Missed look** (closed, covered, observed, no associated detection):
   `L ← L + ln((1 − ρ_c·PD_c*)/(1 − PFA))`. It is applied **once**, when the look closes.
6. A look closes on a sector change, a timeout, or an epoch change. These are evaluated on event
   times only (`ContactTable.close_due`).
7. `L` is clamped to `[logit p_min, logit p_max]` after each update. `p_exists = σ(L)`.
8. **Calibration priority:** while `detector.calibrated` is false, `q` is a raw score and
   `p_exists` is not a probability.
9. **State**: confirmed when there are ≥ `contact.confirm_looks` (2) detected looks, from distinct
   sectors when `contact.confirm_distinct_sectors` is set. A resolvable shadow would also confirm,
   but no input carries it yet. Otherwise the state is candidate.
10. **Pruning**: a candidate with `p_exists < contact.prune_p` is dropped. The table is capped at
    `contact.max_contacts`, lowest `p_exists` first. Uids are retired.

## 10. Contacts: position and class

Implemented in `contacts.Contact.covariance`; checked in [7] and [13].

1. **Covariance validation.** A detection covariance must be finite, symmetric (|xy − yx| ≤ 1e-9),
   positive semi-definite, and of positive trace. Otherwise it is replaced by the propagation model
   (brief §6.6):

       Σ = σ_pose²·I + (J_ψ b)(J_ψ b)ᵀ σ_ψ² + R(ψ) diag(σ_along², (σ_across + f·r)²) R(ψ)ᵀ

   where `J_ψ b = r·(−sin b, cos b)`. The sigmas are `uncertainty.*`.
2. **Common mode.** `σ_cm` = `contact.common_mode_sigma_m` (Perception's assumed σ_nav, 1 m) is
   shared by all detections. Each observation's independent part is
   `R_ind = R − σ_cm²·I`, floored at `(0.05 m)²·I` (eigenvalue floor).
3. **Fusion**: one observation per look (the best-q detection),
   `Λ = Σ_looks R_ind⁻¹` and `x_c = Λ⁻¹ Σ R_ind⁻¹ z`.
4. `P_c = Λ⁻¹ + (σ_cm² + σ_floor² + σ_align²)·I`, where σ_floor = `contact.position_floor_sigma_m`
   and σ_align = `odom_to_map_sigma_m` (0 if NaN).
5. **Class**: `α_{c,k} = α₀ + Σ_looks q*_look·[class = k]`, and `class_probabilities = α/Σα`.
   This is confidence-weighted Dirichlet smoothing, not a Bayes update; there is no confusion
   matrix.

## 11. Rendering

Implemented by `belief.render_p_target`; checked in [8].

`p_target_j = 1 − (1 − p_bg,j)·∏_c (1 − p_c·w_{c,j})`.
- `w_{c,j} = N(cell centre_j; x_c, P_c)·Δ²`, over the cells within 4σ (the largest eigenvalue)
  of x_c.
- Background and contact are treated as independent: an approximation (brief §17.8).

## 12. Disc query

Implemented by `belief.query_disc`; checked in [8].

For the disc `D(x0, R)` and the in-area cells J_D whose centres lie in D:
- `p_any = 1 − ∏_{j∈J_D}(1 − p_bg,j)·∏_c(1 − p_c·m_c)`;
- `m_c = ∫_D N(·; x_c, P_c)`, by midpoint quadrature on a 0.05 m lattice over D ∩ (x_c ± 5σ);
- expected count = `Σ_{J_D} p_bg + Σ_c p_c·m_c`.

The contact's **own mass inside the disc** is used, never the rendered map: a cell-by-cell product
under-counts a spread contact (brief §7, v1 bug 2). For an isotropic contact at the centre,
`m = 1 − exp(−R²/2σ²)` (checked in [8]).

## 13. What the primary sources say, and how Belief uses them (read 2026-11-04)

Paull et al. 2013 and Bourgault et al. 2006 were read in full from PDFs the owner supplied;
Yordanova & Gips 2020 from its arXiv preprint. Exact citations and read status:
`docs/REFERENCES.md`.

1. **Paull's "confidence" is not a detection probability.** `c_ij = P(m_ij = M_ij)`: the
   probability that the belief equals the truth (Eq. 5).
   - Eq. 9, `P(M = 1 | m = 1) = c·p/(2cp − c − p + 1)`, follows only if
     `P(m = 1 | M = 0) = 1 − c`, i.e. symmetric errors.
   - Its P(y) curves run from 0.5 (no information) to 1 (Fig. 2).
   - Belief applies the Eqs. 22–24 combination rule to **PD**, the probability of detecting a
     present target, with PFA separate (0 by default). There, Eq. 23 is exactly the cumulative
     detection probability of independent looks (Bourgault Eqs. 6–7; Yordanova & Gips Eq. 2).
   - Consequence: **Paull's Eq. 9 is not a consistency check for Belief.** It would imply
     PFA = 1 − PD. The superproject note "Paull's Eq. 9 is the consistency check"
     (`literature_conclusions.md`) holds only under Paull's symmetric-error model.
2. **One observation per cell per pass.** Paull's expected information gain sums one term per
   ping line, "Given that there is no overlap between subsequent sonar pings" (Eqs. 15–16).
   Belief's look rule (§5) is the dense-ping analogue: many pings hit a 0.5 m cell in one pass,
   and they count once.
3. **Unknown sonar / environment parameters** (Paull Table I, Eqs. 11–13). Paull takes the
   expectation over the parameter set Z (uniform when unknown), refined as parameters are
   measured. A fitted PD for Belief could follow the same route: a seabed-class mixture
   (`TODO.md` [P2]).
4. **Bourgault's posterior is one normalised target PDF** (Eqs. 2–3). `q_k` is the scalar
   probability of missing it on step k (Eq. 5). Belief's grid instead keeps independent per-cell
   occupancy (Elfes; Thrun):
   - the per-cell update `p(1−PD)/(p(1−PD) + 1 − p)` is Bourgault's miss update with the target
     conditioned in that cell;
   - the per-cell `Q_i = ∏(1 − PD_ik)` is Bourgault's `Q_k` given the target in cell i.
5. **Mission-level cumulative detection probability** (Bourgault Eq. 7) for one target placed
   uniformly in the survey area: `P = 1 − Σ_i π_i Q_i = mean in-area coverage`, with π uniform.
   - It is published as `summary.area_cumulative_pd` on `/diagnostics` and in each snapshot
     manifest (`Belief.area_cumulative_pd`, checked in [20]).
   - It is model-conditional (placeholder PD).
6. **Independence and discrimination are assumptions there too.** Bourgault assumes independent
   observations (§2.2) and perfect discrimination (§4.1). An accurate `P_k` needs "an accurate
   observation model obtained through extensive in situ experimental testing" (§4.3). The same
   caveat applies to every coverage number here.
7. **Smooth observation models** (Bourgault §4.3): "the function [should] be smooth, and …
   decrease progressively to zero without any steps". The PD placeholder has continuous ramps,
   with no steps. ρ is stepped by sectors (§6.6).
8. **For the planner** (informational, not Belief's job):
   - Bourgault's greedy one-step search "spirals" and suffers the "scattering effect": flying
     over a probability peak raises entropy. Lookahead gained +29 % (static) and +22 % (drifting
     target) cumulative detection.
   - Paull's information gain alone was worse than a lawnmower; it needed the branch-entropy term
     (Table III).
