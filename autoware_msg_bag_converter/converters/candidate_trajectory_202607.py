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

"""2026/07: CandidateTrajectory turn_indicators_command injection."""

from bisect import bisect_right
from typing import TYPE_CHECKING

from autoware_internal_planning_msgs.msg import CandidateTrajectories
from autoware_internal_planning_msgs.msg import CandidateTrajectory
from autoware_internal_planning_msgs.msg import GeneratorInfo
from autoware_internal_planning_msgs.msg import ScoredCandidateTrajectories
from autoware_internal_planning_msgs.msg import ScoredCandidateTrajectory
from autoware_internal_planning_msgs_v1_13.msg import (
    CandidateTrajectories as CandidateTrajectoriesV1_13,
)
from autoware_internal_planning_msgs_v1_13.msg import (
    CandidateTrajectory as CandidateTrajectoryV1_13,
)
from autoware_internal_planning_msgs_v1_13.msg import GeneratorInfo as GeneratorInfoV1_13
from autoware_internal_planning_msgs_v1_13.msg import (
    ScoredCandidateTrajectories as ScoredCandidateTrajectoriesV1_13,
)
from autoware_vehicle_msgs.msg import TurnIndicatorsCommand
from autoware_vehicle_msgs.msg import TurnIndicatorsReport
from rclpy.serialization import deserialize_message
from rclpy.serialization import serialize_message

from autoware_msg_bag_converter.bag import create_reader

if TYPE_CHECKING:
    from autoware_msg_bag_converter.converters import ConvertContext

TURN_INDICATORS_STATUS_TOPIC = "/vehicle/status/turn_indicators_status"

TYPE_VERSION_FALLBACK = {
    "autoware_internal_planning_msgs/msg/CandidateTrajectories": (
        "autoware_internal_planning_msgs_v1_13/msg/CandidateTrajectories"
    ),
    "autoware_internal_planning_msgs/msg/ScoredCandidateTrajectories": (
        "autoware_internal_planning_msgs_v1_13/msg/ScoredCandidateTrajectories"
    ),
}


def turn_indicators_report_to_command(report: TurnIndicatorsReport) -> TurnIndicatorsCommand:
    """Map vehicle TurnIndicatorsReport to planning TurnIndicatorsCommand."""
    return TurnIndicatorsCommand(stamp=report.stamp, command=report.report)


def find_nearest_turn_indicators_command(
    timeline: list[tuple[int, TurnIndicatorsReport]],
    stamp_ns: int,
) -> TurnIndicatorsCommand:
    """Return command from the latest report with bag stamp <= stamp_ns, or empty NO_COMMAND."""
    if not timeline:
        return TurnIndicatorsCommand()
    stamps = [entry[0] for entry in timeline]
    idx = bisect_right(stamps, stamp_ns) - 1
    if idx < 0:
        return TurnIndicatorsCommand()
    return turn_indicators_report_to_command(timeline[idx][1])


def preload_turn_indicators_timeline(
    input_bag_path: str,
    storage_type: str,
) -> dict[str, list[tuple[int, TurnIndicatorsReport]]]:
    """Load turn-indicator status timeline for cross-topic merge."""
    timeline: list[tuple[int, TurnIndicatorsReport]] = []
    reader = create_reader(input_bag_path, storage_type)
    try:
        while reader.has_next():
            topic_name, msg, stamp = reader.read_next()
            if topic_name != TURN_INDICATORS_STATUS_TOPIC:
                continue
            report = deserialize_message(msg, TurnIndicatorsReport)
            timeline.append((stamp, report))
    finally:
        del reader
    timeline.sort(key=lambda item: item[0])
    return {"turn_indicators_timeline": timeline}


def _convert_generator_info(old_info: GeneratorInfoV1_13) -> GeneratorInfo:
    return GeneratorInfo(
        generator_id=old_info.generator_id,
        generator_name=old_info.generator_name,
    )


def _convert_candidate_trajectory(
    old_traj: CandidateTrajectoryV1_13,
    turn_indicators_command: TurnIndicatorsCommand,
) -> CandidateTrajectory:
    return CandidateTrajectory(
        header=old_traj.header,
        generator_id=old_traj.generator_id,
        points=old_traj.points,
        turn_indicators_command=turn_indicators_command,
    )


def convert_candidate_trajectories_v1_13(
    old_msg: CandidateTrajectoriesV1_13,
    turn_indicators_command: TurnIndicatorsCommand,
) -> bytes:
    new_msg = CandidateTrajectories()
    for old_traj in old_msg.candidate_trajectories:
        new_msg.candidate_trajectories.append(
            _convert_candidate_trajectory(old_traj, turn_indicators_command)
        )
    for old_info in old_msg.generator_info:
        new_msg.generator_info.append(_convert_generator_info(old_info))
    return serialize_message(new_msg)


def convert_scored_candidate_trajectories_v1_13(
    old_msg: ScoredCandidateTrajectoriesV1_13,
    turn_indicators_command: TurnIndicatorsCommand,
) -> bytes:
    new_msg = ScoredCandidateTrajectories()
    for old_scored in old_msg.scored_candidate_trajectories:
        new_msg.scored_candidate_trajectories.append(
            ScoredCandidateTrajectory(
                candidate_trajectory=_convert_candidate_trajectory(
                    old_scored.candidate_trajectory,
                    turn_indicators_command,
                ),
                score=old_scored.score,
            )
        )
    for old_info in old_msg.generator_info:
        new_msg.generator_info.append(_convert_generator_info(old_info))
    return serialize_message(new_msg)


def convert_candidate_trajectories_v1_13_with_context(
    old_msg: CandidateTrajectoriesV1_13,
    *,
    context: "ConvertContext",
) -> bytes:
    turn_cmd = find_nearest_turn_indicators_command(
        context.turn_indicators_timeline,
        context.stamp_ns,
    )
    return convert_candidate_trajectories_v1_13(old_msg, turn_cmd)


def convert_scored_candidate_trajectories_v1_13_with_context(
    old_msg: ScoredCandidateTrajectoriesV1_13,
    *,
    context: "ConvertContext",
) -> bytes:
    turn_cmd = find_nearest_turn_indicators_command(
        context.turn_indicators_timeline,
        context.stamp_ns,
    )
    return convert_scored_candidate_trajectories_v1_13(old_msg, turn_cmd)
