# BlueBoat-Belief — development guide

How to work on the belief map: the AI coding assistant setup, the tests, the rules and what to do
next. For using Belief, see the [README](../../README.md) and the [user guide](USER_GUIDE.md).

## 1. Install Claude Code

The project is developed with [Claude Code](https://docs.claude.com/en/docs/claude-code/overview),
an AI coding assistant that runs in the terminal.

```bash
curl -fsSL https://claude.ai/install.sh | bash        # native installer
# or, with Node.js 18 or newer:
npm install -g @anthropic-ai/claude-code
```

Then start it and log in with your Anthropic account:

```bash
cd ~/ros2_ws/src/BlueBoat-SideScanSonar               # the whole project: cross-module rules included
claude
# or only this module:
cd ~/ros2_ws/src/BlueBoat-SideScanSonar/BlueBoat-Belief && claude
```

Starting from the superproject is better: Claude then also reads the project rules, the interface
contract between modules and the shared skills (test commands, file formats).

## 2. What Claude reads in this module

| File | Role |
|---|---|
| `.claude/CLAUDE.md` | Loaded at start: the layout, the interface, the non-negotiable rules, the commands, the tests and the status |
| `.claude/TODO.md` | Open work, by priority |
| `blueboat_belief/docs/MATH.md` | The specification: a change to the model is written here first |
| `blueboat_belief/docs/topics.md` | The interface contract |
| `blueboat_belief/docs/DESIGN.md` | The decisions in force (the D- and Q- numbers cited in the code) |

The documents describe the current state only: no session logs or change histories. Update a
document in place and delete what is no longer true.

## 3. What to set up after a fresh clone

Nothing in `.claude/` is ignored by git, and Belief has no Claude hook, so a fresh clone is complete.
You need:

- **Both Python environments**: the workspace venv (`~/ros2_ws/.venv`, with numpy 2 and `rosbags`) and
  the system Python (numpy 1, the one the ROS nodes use). The test gate runs in both.
- **The neighbouring checkouts** (BlueBoat-MCS and BlueBoat-Replanner beside this repository) for
  two of the gate's sections; without them those sections cannot run.

## 4. Tests

```bash
cd ~/ros2_ws/src/BlueBoat-SideScanSonar/BlueBoat-Belief/blueboat_belief
test/gate.sh                                   # offline gate in both Pythons: must end ALL CHECKS PASSED, 0 skipped
/usr/bin/python3 -B -m test.ros_graph_check    # the ROS runtime check
```

- **Do not source `~/ros2_ws/install/setup.bash` before `test/gate.sh`.**
- **The ROS check needs `ROS_DOMAIN_ID` set to something other than 0**, Belief sourced, and no other
  node running. It refuses otherwise. Never run it on a live graph.
- There is no linter or type checker.
- The Replanner reads the line `ALL CHECKS PASSED (N checks, M skipped)` of
  `docs/VALIDATION_REPORT.md`: keep its format.

## 5. Rules for changes

- **Never edit outside BlueBoat-Belief.** Needs from other modules go to `docs/REQUESTS.md`.
- **New message types only in `blueboat_belief_interfaces`.** Never rename or retype a topic; delivery
  settings match the other end.
- **Online code never reads simulation ground truth.**
- **`core/` imports no ROS and no Qt**; ROS lives only in `ros/belief_node.py`.
- **The core is deterministic**: no wall clock, no randomness, sorted iteration. Snapshots must be
  byte-identical on replay.
- **Looks, not pings**: the miss rules in `.claude/CLAUDE.md` are exact and tested.
- **Cite nothing from memory**, and never tune a placeholder number to improve a result.
- **Model first**: a change to `docs/MATH.md` comes before the code change, with a closed-form test.
- **Outputs are never overwritten.**
- **The Replanner imports `core/`.** Do not edit `core/*.py` while a Replanner development run is
  going (`pgrep -f blueboat_replanner.eval.runner`).
- AI sessions never commit, push or tag: the owner does.

## 6. Next steps

From `.claude/TODO.md`, in plain words:

1. **Calibrate the detector**, so that `p_exists` and `p_target` become probabilities: a calibrated
   detector on held-out simulated worlds, then `detector.calibrated: true` and its base rate.
2. **Run Belief with a real detector** once YOLO weights exist: set the class names, run the live
   simulation check, and compare the contacts with the world's objects offline.
3. **Check on real-boat data** that the odometry heading is absolute east-north-up, how often the
   timestamps are zero, and how far the compass and odometry headings differ. Some of this can use
   the run bags already on disk.
4. **Check the turn gate on straight lines** with a real run's own timing: in Gazebo it also cut about
   one ping in ten on a straight return leg.
5. **Replace the placeholder detection model** with values fitted on a real detector.
