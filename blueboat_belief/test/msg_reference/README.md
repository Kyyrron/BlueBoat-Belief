# msg_reference: verbatim message definitions used by the ROS-free layout tests

Never imported at runtime. `test/smoke_test.py [10]` parses these with `rosbags` to check that
the adapters read the real field layouts.

| File | Source | Copied |
|---|---|---|
| `blueboat_interfaces/msg/ProcessedSSSPing.msg` | `BlueBoat-Control/blueboat_interfaces/msg/ProcessedSSSPing.msg` at Control `1fadbb2` | 2026-11-04 |
| `vision_msgs/msg/*.msg` | https://raw.githubusercontent.com/ros-perception/vision_msgs/ros2/vision_msgs/msg/ (branch `ros2`) | 2026-11-04 |

The standard types (`std_msgs`, `geometry_msgs`, `nav_msgs`, `sensor_msgs`, `diagnostic_msgs`,
`visualization_msgs`) come from the `rosbags` ROS 2 Jazzy type store.
