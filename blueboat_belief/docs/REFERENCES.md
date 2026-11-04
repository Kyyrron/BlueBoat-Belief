# REFERENCES: what each source supports, and whether it was read

**Rule:** cite nothing from memory. Every non-software choice in `MATH.md` / `DESIGN.md` traces to
a row here. A source that is relevant to an idea is not evidence for a numerical parameter.

Status values:
- **PRIMARY**: the full text was read in a module session; claims point to sections and equations.
- **NOTES**: known only through the project's `literature_conclusions.md` or the brief.
- **NOT READ**.

| Source | Status | Supports in Belief | Does NOT support |
|---|---|---|---|
| Bourgault, F., Furukawa, T., Durrant-Whyte, H. F. (2006). *Optimal Search for a Lost Target in a Bayesian World.* Field and Service Robotics, STAR 24, Springer, pp. 209–222. (PDF supplied by the owner) | **PRIMARY** (2026-11-04) | Miss likelihood `p(D̄|x) = 1 − p(D|x)` (Eq. 4); `q_k` (Eq. 5); `Q_k = ∏ q_i` (Eq. 6); `P_k = 1 − Q_k` (Eq. 7); first detection `p_k = Q_{k−1}(1 − q_k)` (Eq. 8); MTTD (Eq. 10). Observations assumed independent (§2.2). Perfect discrimination, no false alarms (§4.1). An accurate `P_k` needs an in-situ observation model (§4.3). Observation models should be smooth, without steps (§4.3). Greedy search and the "scattering effect"; +29 % / +22 % for lookahead (§4.4) | Belief's per-cell independent occupancy grid: Bourgault keeps **one normalised target PDF**. Any number for SSS |
| Paull, L., Saeedi, S., Seto, M., Li, H. (2013). *Sensor-Driven Online Coverage Planning for Autonomous Underwater Vehicles.* IEEE/ASME Trans. Mechatronics 18(6), 1827–1838, doi:10.1109/TMECH.2012.2213607. (PDF supplied by the owner) | **PRIMARY** (2026-11-04) | Angle-dependent fusion of two looks (§III-C, Eqs. 22–24), with c¹ ≥ c² and α = the acute angle between the two insonification **lines** (Fig. 7), stated as an assumption. Lateral-range P(y) curves from ESPRESSO depend on seabed, target, sonar and vehicle (Table I, Fig. 2). Expected confidence over unknown parameters (Eqs. 11–13). One observation per cell per ping line, "no overlap between subsequent sonar pings" (Eqs. 15–16). Turns: outer side missed, inner side "completely distorted" for ATR (§II-C). Eq. 9 (`P(M=1|m=1)`) under symmetric errors | That the linear rule is measured. Paull: "the purpose of this work is not to verify the ESPRESSO model"; Fawcett is cited for the value of multiple views, not for linearity. Paull's confidence (`P(m = M)`, range 0.5–1) is not PD, so Eq. 9 is not a check for Belief (`MATH.md` §13.1) |
| Yordanova, V., Gips, B. (2020). *Coverage Path Planning With Track Spacing Adaptation for Autonomous Underwater Vehicles.* IEEE RA-L 5(3), 4774–4780, doi:10.1109/LRA.2020.3003886 (arXiv 2006.12896 preprint read) | **PRIMARY** (2026-11-04) | Residual risk `RR = ∏(1 − Pd_i)` per grid point over looks (Eq. 2). Pd from through-the-sensor features for the ATR, reduced to range (§III-B) | SSS numbers: the platform is a side-looking **SAS** (MUSCLE, Bluefin-21) |
| Myers, V., Williams, D. P. (2012). IEEE JOE 37(1), 45–55 | NOTES | Value of additional views (SAS, POMDP) | A measured same-aspect SSS dependence |
| Fawcett, J. et al. (2010). IEEE JOE 35(4), 863–876 | NOTES | Two-look SSS classification and fusion; 30° aspect increments, the source of S = 12 | Paull's linear rule; a PD model |
| Elfes, A. (1989). Computer 22(6), 46–57; Thrun, Burgard, Fox (2005). *Probabilistic Robotics*, ch. 9 | NOTES | Occupancy grids; log-odds update with clamping | — |
| Thrun, S. (2003). Autonomous Robots 15, 111–127 | NOTES (brief §17.8) | The independent-cell caveat (L7) | — |
| Mahler, R. (2007). *Statistical Multisource-Multitarget Information Fusion* | NOTES | The Bernoulli-contact idea (existence + Gaussian position) | That the contact table is an RFS filter (it is not) |
| Guo, C. et al. (2017). ICML, PMLR 70 | NOTES | Calibration needed (the calibration priority) | That Perception's score is calibrated |
| Nguyen, P., Murdoch, D. (2012). Naval Res. Logistics 59(8) | NOTES (brief §17.3) | A precedent for fitting a PD(range) curve with confidence intervals | — |
| Cobra, D. T. et al. (1992) | NOT READ (cited by Perception for `backscan_from_m`) | The back-scan radius `r* = v/|ω|` as Perception uses it | — |
| Berlenko & Krinkin (2026); Bar-Shalom et al.; Vo & Vo | NOTES (brief §15, §18) | Context only: Bayesian vs DS grids; data association in clutter (L6) | — |
