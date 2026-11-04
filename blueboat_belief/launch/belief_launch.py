"""Launch the BlueBoat-Belief node (belief_map).

    ros2 launch blueboat_belief belief_launch.py area_file:=<world folder | survey-area yaml> \
        [use_sim_time:=true] [config:=<yaml>] [record_inputs:=true] [data_dir:=<root>]

- ``use_sim_time``: true under Gazebo, false on the real boat. Zero stamps there are replaced by
  the node clock (docs/topics.md).
- ``record_inputs:=true`` (owner decision Q-12) also runs ``ros2 bag record`` of Belief's four
  input topics into ``<root>/data/Belief_data/<stamp>-inputs``. Control's run bag excludes the
  sonar topics, so this recording is what an offline replay (``blueboat-belief-replay``) needs.

ROS imports are optional (the SSS-Sim pattern), so the offline smoke test can import the
module-scope helpers without ``launch`` installed.
"""

from __future__ import annotations

import datetime as _dt
import os

try:  # pragma: no cover - exercised only with ROS
    from launch import LaunchDescription
    from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
    from launch.substitutions import LaunchConfiguration
    from launch_ros.actions import Node
    from ament_index_python.packages import get_package_share_directory
except ImportError:  # offline import (smoke test)
    LaunchDescription = None

INPUT_TOPICS = (
    "/sss_processor/processed",
    "/sss_ai/detections",
    "/blueboat/odom",
    "/mavros/global_position/global",
)


def as_bool(text: str) -> bool:
    return str(text).strip().lower() in ("1", "true", "yes", "on")


def record_command(out_dir: str) -> list:
    """The ``ros2 bag record`` command line for Belief's inputs (mcap)."""
    return ["ros2", "bag", "record", "-s", "mcap", "-o", out_dir, *INPUT_TOPICS]


def record_dir(data_dir: str, now: _dt.datetime) -> str:
    """``<root>/data/Belief_data/<YYYY_MM_DD-HH_MM_SS>-inputs`` (+ ``-2``… if taken; never reused)."""
    from blueboat_belief.io.dataroot import belief_dir  # noqa: PLC0415 - installed package

    base = belief_dir(data_dir)
    base.mkdir(parents=True, exist_ok=True)
    stem = now.strftime("%Y_%m_%d-%H_%M_%S") + "-inputs"
    k = 1
    while True:
        p = base / (stem if k == 1 else f"{stem}-{k}")
        if not p.exists():
            return str(p)
        k += 1


def _setup(context, *args, **kwargs):  # pragma: no cover - needs ROS
    cfg = LaunchConfiguration("config").perform(context)
    area = LaunchConfiguration("area_file").perform(context)
    sim = as_bool(LaunchConfiguration("use_sim_time").perform(context))
    data_dir = LaunchConfiguration("data_dir").perform(context)
    params = [{"area.file": area, "io.data_dir": data_dir, "use_sim_time": sim}]
    if cfg:
        params.insert(0, cfg)
    actions = [Node(package="blueboat_belief", executable="belief_node", name="belief_map",
                    parameters=params, output="screen")]
    if as_bool(LaunchConfiguration("record_inputs").perform(context)):
        actions.append(ExecuteProcess(cmd=record_command(record_dir(data_dir, _dt.datetime.now())),
                                      output="screen"))
    return actions


def generate_launch_description():  # pragma: no cover - needs ROS
    share = get_package_share_directory("blueboat_belief")
    return LaunchDescription([
        DeclareLaunchArgument("area_file", description="world folder, metadata.yaml or survey-area file"),
        DeclareLaunchArgument("config", default_value=os.path.join(share, "config", "belief.yaml")),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("record_inputs", default_value="false"),
        DeclareLaunchArgument("data_dir", default_value=""),
        OpaqueFunction(function=_setup),
    ])
