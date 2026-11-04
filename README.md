# BlueBoat-Belief

> **PRIORITY: detector calibration.**
> - Perception's detector scores are **uncalibrated** today (calibration = identity), and
>   `/sss_ai/detections` carries no calibration flag.
> - Belief runs with `detector.calibrated: false` and publishes that flag in every `BeliefGrid`
>   (`model_params_json`).
> - **Until a calibrated detector is deployed, `p_exists` and `p_target` are score-derived
>   indices, not probabilities.** See `blueboat_belief/docs/REQUESTS.md` R-1 and `.claude/TODO.md` [P0].

The probabilistic belief map (thesis deliverable D4) of the BlueBoat aspect-aware side-scan sonar
survey. From every processed ping and every detection it keeps a recursive estimate of three things:
- where targets probably are;
- where the seabed has really been searched;
- **from which aspects**.

It publishes that estimate as `BeliefGrid` (contract `belief_grid/0`), the replanner's only
input.

| Package | Type | Content |
|---|---|---|
| `blueboat_belief_interfaces/` | ament_cmake (rosidl) | `msg/BeliefGrid`, `msg/BeliefContact`, `srv/QueryBelief` |
| `blueboat_belief/` | ament_python | ROS-free deterministic core, adapters, the `belief_map` node, offline replay CLI |

```bash
colcon build --packages-up-to blueboat_belief          # from ~/ros2_ws
ros2 launch blueboat_belief belief_launch.py area_file:=<world folder | survey-area yaml> [use_sim_time:=true] [record_inputs:=true]
blueboat-belief-replay <bag> --area <file>             # offline, deterministic
blueboat-belief-make-area --out area.yaml --anchor LAT LON --local XMIN YMIN XMAX YMAX   # survey-area file
cd blueboat_belief && python3 -m test.smoke_test       # the gate, no ROS needed
```

Documentation, in `blueboat_belief/docs/`:
- `topics.md`: interfaces, layout, conventions;
- `MATH.md`: the model specification (§13: what Paull 2013 and Bourgault 2006 say, read in the
  primary text);
- `REFERENCES.md`: what each source supports, and whether it was read;
- `DESIGN.md`: decisions and limitations;
- `SURVEY_AREA_FORMAT.md`;
- `REQUESTS.md`;
- `CONTRACT_PROVIDED.md` / `CONTRACT_DEVIATIONS.md`;
- `LOCAL_CHECKLIST.md`: what still needs a ROS machine.

All results are model-conditional. The PD curve is a placeholder, and the aspect-dependence rule
is Paull et al.'s stated assumption (Eqs. 22–24), not a measurement; both stay so until fitted.
