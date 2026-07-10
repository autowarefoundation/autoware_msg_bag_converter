# Copyright (c) 2024 TIER IV.inc
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from pathlib import Path
import re

from autoware_internal_planning_msgs.msg import CandidateTrajectories
from autoware_internal_planning_msgs.msg import ScoredCandidateTrajectories
from autoware_internal_planning_msgs_v1_13.msg import (
    CandidateTrajectories as CandidateTrajectoriesV1_13,
)
from autoware_internal_planning_msgs_v1_13.msg import (
    CandidateTrajectory as CandidateTrajectoryV1_13,
)
from autoware_internal_planning_msgs_v1_13.msg import (
    ScoredCandidateTrajectories as ScoredCandidateTrajectoriesV1_13,
)
from autoware_internal_planning_msgs_v1_13.msg import (
    ScoredCandidateTrajectory as ScoredCandidateTrajectoryV1_13,
)
from autoware_vehicle_msgs.msg import TurnIndicatorsCommand
from autoware_vehicle_msgs.msg import TurnIndicatorsReport
from builtin_interfaces.msg import Time
from rclpy.serialization import deserialize_message
from rosbag2_py import TopicMetadata
from std_msgs.msg import Header
from unique_identifier_msgs.msg import UUID

from autoware_msg_bag_converter.converter import change_topic_type
from autoware_msg_bag_converter.converter import convert_candidate_trajectories_v1_13
from autoware_msg_bag_converter.converter import convert_scored_candidate_trajectories_v1_13
from autoware_msg_bag_converter.converter import find_nearest_turn_indicators_command
from autoware_msg_bag_converter.converter import turn_indicators_report_to_command


def test_change_topic_type() -> None:
    auto_type = TopicMetadata(
        name="/vehicle/status/control_mode",
        type="autoware_auto_vehicle_msgs/msg/ControlModeReport",
        serialization_format="cdr",
    )
    new_type = change_topic_type(auto_type)
    assert new_type.name == "/vehicle/status/control_mode"
    assert new_type.type == "autoware_vehicle_msgs/msg/ControlModeReport"
    assert new_type.serialization_format == "cdr"

    not_auto_type = TopicMetadata(
        name="/tf",
        type="tf2_msgs/msg/TFMessage",
        serialization_format="cdr",
    )
    not_changed_type = change_topic_type(not_auto_type)
    assert not_auto_type.name == not_changed_type.name
    assert not_auto_type.type == not_changed_type.type
    assert not_auto_type.serialization_format == not_changed_type.serialization_format


def test_get_rosbag_path() -> None:
    # Test to confirm bag path acquisition in directory mode
    input_root = Path(__file__).resolve().parent.joinpath("resource")
    output_root = Path(__file__).resolve().parent.joinpath("converted")
    pattern = re.compile(r".*\.(db3|mcap)$")
    bag_paths = [p for p in input_root.rglob("*") if pattern.match(str(p))]
    assert len(bag_paths) == 3  # noqa
    for db3_path in bag_paths:
        input_bag_dir = db3_path.parent
        rel_path = input_bag_dir.relative_to(input_root)
        output_bag_dir = output_root.joinpath(rel_path)
        print(output_bag_dir)  # noqa


def test_turn_indicators_report_to_command() -> None:
    report = TurnIndicatorsReport(
        stamp=Time(sec=1, nanosec=2),
        report=TurnIndicatorsReport.ENABLE_LEFT,
    )
    command = turn_indicators_report_to_command(report)
    assert command.command == TurnIndicatorsCommand.ENABLE_LEFT
    assert command.stamp.sec == report.stamp.sec
    assert command.stamp.nanosec == report.stamp.nanosec


def test_find_nearest_turn_indicators_command() -> None:
    timeline = [
        (10, TurnIndicatorsReport(stamp=Time(sec=0), report=TurnIndicatorsReport.DISABLE)),
        (20, TurnIndicatorsReport(stamp=Time(sec=0), report=TurnIndicatorsReport.ENABLE_LEFT)),
        (30, TurnIndicatorsReport(stamp=Time(sec=0), report=TurnIndicatorsReport.ENABLE_RIGHT)),
    ]
    assert find_nearest_turn_indicators_command([], 15).command == TurnIndicatorsCommand.NO_COMMAND
    assert (
        find_nearest_turn_indicators_command(timeline, 5).command
        == TurnIndicatorsCommand.NO_COMMAND
    )
    assert (
        find_nearest_turn_indicators_command(timeline, 20).command
        == TurnIndicatorsCommand.ENABLE_LEFT
    )
    assert (
        find_nearest_turn_indicators_command(timeline, 25).command
        == TurnIndicatorsCommand.ENABLE_LEFT
    )
    assert (
        find_nearest_turn_indicators_command(timeline, 30).command
        == TurnIndicatorsCommand.ENABLE_RIGHT
    )


def test_convert_candidate_trajectories_v1_13_injects_turn_indicators() -> None:
    old_traj = CandidateTrajectoryV1_13(
        header=Header(frame_id="map"),
        generator_id=UUID(uuid=[1] + [0] * 15),
        points=[],
    )
    old_msg = CandidateTrajectoriesV1_13(candidate_trajectories=[old_traj], generator_info=[])
    turn_cmd = TurnIndicatorsCommand(
        stamp=Time(sec=3, nanosec=4),
        command=TurnIndicatorsCommand.ENABLE_RIGHT,
    )

    new_bytes = convert_candidate_trajectories_v1_13(old_msg, turn_cmd)
    new_msg = deserialize_message(new_bytes, CandidateTrajectories)

    assert len(new_msg.candidate_trajectories) == 1
    injected = new_msg.candidate_trajectories[0].turn_indicators_command
    assert injected.command == TurnIndicatorsCommand.ENABLE_RIGHT
    assert injected.stamp.sec == turn_cmd.stamp.sec
    assert injected.stamp.nanosec == turn_cmd.stamp.nanosec
    assert new_msg.candidate_trajectories[0].header.frame_id == "map"


def test_convert_scored_candidate_trajectories_v1_13_injects_turn_indicators() -> None:
    old_traj = CandidateTrajectoryV1_13(
        header=Header(frame_id="map"),
        generator_id=UUID(uuid=[2] + [0] * 15),
        points=[],
    )
    expected_score = 0.5
    old_msg = ScoredCandidateTrajectoriesV1_13(
        scored_candidate_trajectories=[
            ScoredCandidateTrajectoryV1_13(candidate_trajectory=old_traj, score=expected_score),
        ],
        generator_info=[],
    )
    turn_cmd = TurnIndicatorsCommand(command=TurnIndicatorsCommand.DISABLE)

    new_bytes = convert_scored_candidate_trajectories_v1_13(old_msg, turn_cmd)
    new_msg = deserialize_message(new_bytes, ScoredCandidateTrajectories)

    assert len(new_msg.scored_candidate_trajectories) == 1
    scored = new_msg.scored_candidate_trajectories[0]
    assert scored.score == expected_score
    assert scored.candidate_trajectory.turn_indicators_command.command == (
        TurnIndicatorsCommand.DISABLE
    )
