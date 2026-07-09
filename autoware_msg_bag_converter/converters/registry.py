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

from collections.abc import Callable
from dataclasses import dataclass
from dataclasses import field
from typing import Any

from autoware_vehicle_msgs.msg import TurnIndicatorsReport
from rosbag2_py import TopicMetadata

from autoware_msg_bag_converter.converters import auto_msgs_202408
from autoware_msg_bag_converter.converters import candidate_trajectory_202607
from autoware_msg_bag_converter.converters import perception_202505
from autoware_msg_bag_converter.converters import planning_internal_202502
from autoware_msg_bag_converter.converters import pointcloud_202507
from autoware_msg_bag_converter.converters import route_state_202503
from autoware_msg_bag_converter.converters import topic_rename_202508

ConverterFn = Callable[..., bytes]


@dataclass
class ConvertContext:
    turn_indicators_timeline: list[tuple[int, TurnIndicatorsReport]] = field(default_factory=list)
    stamp_ns: int = 0


TYPE_NOT_SIMPLY_REPLACED = {
    **auto_msgs_202408.TYPE_NOT_SIMPLY_REPLACED,
}

PREFIX_PACKAGES = auto_msgs_202408.PREFIX_PACKAGES

TYPE_NAME_REMAPPING = {
    **planning_internal_202502.TYPE_NAME_REMAPPING,
    **route_state_202503.TYPE_NAME_REMAPPING,
    **perception_202505.TYPE_NAME_REMAPPING,
}

TYPE_VERSION_FALLBACK = {
    **perception_202505.TYPE_VERSION_FALLBACK,
    **candidate_trajectory_202607.TYPE_VERSION_FALLBACK,
}

TYPES_TO_UPDATE_DATA = pointcloud_202507.TYPES_TO_UPDATE_DATA

TOPIC_NAME_REMAPPING = topic_rename_202508.TOPIC_NAME_REMAPPING


def change_topic_type(old_type: TopicMetadata) -> TopicMetadata:
    serialization_format = "cdr"

    if old_type.type in TYPE_NOT_SIMPLY_REPLACED:
        new_topic_type = TYPE_NOT_SIMPLY_REPLACED[old_type.type]
    else:
        new_topic_type = old_type.type.replace("autoware_auto_", "autoware_")
    if any(old_type.type.startswith(prefix) for prefix in PREFIX_PACKAGES):
        new_topic_type = f"autoware_{old_type.type}"

    if old_type.type in TYPE_NAME_REMAPPING:
        new_topic_type = TYPE_NAME_REMAPPING[old_type.type]

    new_topic_name = TOPIC_NAME_REMAPPING.get(old_type.name, old_type.name)

    return TopicMetadata(
        name=new_topic_name,
        type=new_topic_type,
        serialization_format=serialization_format,
        offered_qos_profiles=old_type.offered_qos_profiles,
    )


def _without_context(converter: Callable[[Any], bytes]) -> ConverterFn:
    def wrapper(old_msg: Any, *, context: ConvertContext) -> bytes:  # noqa: ARG001
        return converter(old_msg)

    return wrapper


REGISTRY: dict[str, ConverterFn] = {
    "autoware_auto_control_msgs/msg/AckermannControlCommand": _without_context(
        auto_msgs_202408.convert_ackermann_control_command
    ),
    "autoware_auto_planning_msgs/msg/PathWithLaneId": _without_context(
        auto_msgs_202408.convert_path_with_lane_id
    ),
    "autoware_auto_planning_msgs/msg/HADMapRoute": _without_context(
        auto_msgs_202408.convert_hadmap_route
    ),
    "autoware_auto_perception_msgs/msg/TrafficSignalArray": _without_context(
        auto_msgs_202408.convert_auto_traffic_signal_array
    ),
    "autoware_perception_msgs/msg/TrafficSignalArray": _without_context(
        auto_msgs_202408.convert_traffic_signal_array
    ),
    "autoware_perception_msgs_v1_7/msg/TrafficLightGroupArray": _without_context(
        perception_202505.convert_traffic_light_group_array_v1_7
    ),
    "sensor_msgs/msg/PointCloud2": _without_context(pointcloud_202507.convert_pointcloud2),
    "autoware_internal_planning_msgs_v1_13/msg/CandidateTrajectories": (
        candidate_trajectory_202607.convert_candidate_trajectories_v1_13_with_context
    ),
    "autoware_internal_planning_msgs_v1_13/msg/ScoredCandidateTrajectories": (
        candidate_trajectory_202607.convert_scored_candidate_trajectories_v1_13_with_context
    ),
}

PRELOADERS = [
    candidate_trajectory_202607.preload_turn_indicators_timeline,
]


def convert_by_type(old_type: str, old_msg: Any, *, context: ConvertContext) -> bytes:
    converter = REGISTRY.get(old_type)
    if converter is None:
        raise KeyError(f"No converter registered for type {old_type}")
    return converter(old_msg, context=context)


def build_convert_context(input_bag_path: str, storage_type: str) -> ConvertContext:
    context_data: dict[str, Any] = {}
    for preloader in PRELOADERS:
        context_data.update(preloader(input_bag_path, storage_type))
    return ConvertContext(
        turn_indicators_timeline=context_data.get("turn_indicators_timeline", []),
    )
