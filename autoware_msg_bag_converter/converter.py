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

# refer the test code of rosbag2_py
# https://github.com/ros2/rosbag2/blob/rolling/rosbag2_py/test/test_sequential_writer.py
# https://github.com/ros2/rosbag2/blob/rolling/rosbag2_py/test/test_reindexer.py

from pathlib import Path
from typing import Any
from typing import TYPE_CHECKING

from autoware_auto_control_msgs.msg import AckermannControlCommand
from autoware_auto_perception_msgs.msg import TrafficSignalArray as AutoTrafficSignalArray
from autoware_auto_planning_msgs.msg import HADMapRoute
from autoware_auto_planning_msgs.msg import PathWithLaneId as AutoPathWithLaneId
from autoware_control_msgs.msg import Control
from autoware_control_msgs.msg import Lateral
from autoware_control_msgs.msg import Longitudinal
from autoware_perception_msgs.msg import TrafficLightElement
from autoware_perception_msgs.msg import TrafficLightGroup
from autoware_perception_msgs.msg import TrafficLightGroupArray
from autoware_perception_msgs.msg import TrafficSignalArray
from autoware_perception_msgs_v1_7.msg import TrafficLightGroupArray as TrafficLightGroupArrayV1_7
from autoware_planning_msgs.msg import LaneletPrimitive
from autoware_planning_msgs.msg import LaneletRoute
from autoware_planning_msgs.msg import LaneletSegment
from autoware_planning_msgs.msg import PathPoint
from rclpy.serialization import deserialize_message
from rclpy.serialization import serialize_message
from rosbag2_py import Reindexer
from rosbag2_py import TopicMetadata
from rosidl_runtime_py.utilities import get_message
from tier4_planning_msgs.msg import PathPointWithLaneId
from tier4_planning_msgs.msg import PathWithLaneId as T4PathWithLaneId
import yaml

from autoware_msg_bag_converter.bag import create_reader
from autoware_msg_bag_converter.bag import create_writer
from autoware_msg_bag_converter.bag import get_storage_options
from autoware_msg_bag_converter.convert_pointcloud_types import convert_pointcloud2

if TYPE_CHECKING:
    from autoware_auto_perception_msgs.msg import TrafficLight as AutoTrafficLight
    from autoware_auto_perception_msgs.msg import TrafficSignal as AutoTrafficSignal
    from autoware_perception_msgs.msg import TrafficSignal
    from autoware_perception_msgs.msg import TrafficSignalElement

TYPES_NOT_SIMPLY_REPLACED = {
    # 2024/08 add autoware_ prefix and remove auto_ prefix
    "autoware_auto_control_msgs/msg/AckermannControlCommand": "autoware_control_msgs/msg/Control",
    "autoware_auto_planning_msgs/msg/PathWithLaneId": "tier4_planning_msgs/msg/PathWithLaneId",
    "autoware_auto_planning_msgs/msg/HADMapRoute": "autoware_planning_msgs/msg/LaneletRoute",
    "autoware_auto_perception_msgs/msg/TrafficSignalArray": "autoware_perception_msgs/msg/TrafficLightGroupArray",
    "autoware_perception_msgs/msg/TrafficSignalArray": "autoware_perception_msgs/msg/TrafficLightGroupArray",
}

TYPES_TO_ADD_AUTOWARE_PREFIX = [
    # 2024/08 add autoware_ prefix and remove auto_ prefix
    "control_validator/msg",
    "planning_validator/msg",
    "vehicle_cmd_gate/msg",
]

TYPE_NAME_REMAPPING = {
    # 2025/02 https://github.com/autowarefoundation/autoware_universe/pull/10180
    "tier4_planning_msgs/msg/Scenario": "autoware_internal_planning_msgs/msg/Scenario",
    # 2025/02 https://github.com/autowarefoundation/autoware_universe/pull/10023
    "tier4_planning_msgs/msg/PathWithLaneId": "autoware_internal_planning_msgs/msg/PathWithLaneId",
    # 2025/03 https://github.com/autowarefoundation/autoware_internal_msgs/pull/55
    "tier4_planning_msgs/msg/RouteState": "autoware_internal_planning_msgs/msg/RouteState",
    # 2025/05 https://github.com/autowarefoundation/autoware_universe/pull/10273
    "tier4_planning_msgs/msg/VelocityLimit": "autoware_internal_planning_msgs/msg/VelocityLimit",
    "tier4_planning_msgs/msg/ClearVelocityLimit": "autoware_internal_planning_msgs/msg/ClearVelocityLimit",
}

TYPES_TO_UPDATE_VERSION = {
    # 2025/5
    "autoware_perception_msgs/msg/TrafficLightGroupArray": "autoware_perception_msgs_v1_7/msg/TrafficLightGroupArray",
}
TYPES_TO_UPDATE_DATA = [
    # 2025/7
    "sensor_msgs/msg/PointCloud2",
]

TOPIC_NAME_REMAPPING = {
    # 2025/08
    "/planning/scenario_planning/trajectory": "/planning/trajectory",
}


def change_topic_type(old_type: TopicMetadata) -> TopicMetadata:
    serialization_format = "cdr"

    if old_type.type in TYPES_NOT_SIMPLY_REPLACED:
        new_topic_type = TYPES_NOT_SIMPLY_REPLACED[old_type.type]
    else:
        # If old_type is not in the conversion rules, simply remove "auto_" and use that as the new type.
        new_topic_type = old_type.type.replace("autoware_auto_", "autoware_")
    if any(old_type.type.startswith(prefix) for prefix in TYPES_TO_ADD_AUTOWARE_PREFIX):
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


def convert_ackermann_control_command(old_msg: AckermannControlCommand) -> bytes:
    lateral = Lateral(
        stamp=old_msg.lateral.stamp,
        steering_tire_angle=old_msg.lateral.steering_tire_angle,
        steering_tire_rotation_rate=old_msg.lateral.steering_tire_rotation_rate,
        is_defined_steering_tire_rotation_rate=True,
    )
    longitudinal = Longitudinal(
        stamp=old_msg.longitudinal.stamp,
        velocity=old_msg.longitudinal.speed,
        acceleration=old_msg.longitudinal.acceleration,
        jerk=old_msg.longitudinal.jerk,
        is_defined_acceleration=True,
        is_defined_jerk=False,
    )
    return serialize_message(
        Control(
            stamp=old_msg.stamp,
            lateral=lateral,
            longitudinal=longitudinal,
        ),
    )


def convert_path_with_lane_id(old_msg: AutoPathWithLaneId) -> bytes:
    points: list[PathPointWithLaneId] = []
    for old_point in old_msg.points:
        point = PathPoint(
            pose=old_point.point.pose,
            longitudinal_velocity_mps=old_point.point.longitudinal_velocity_mps,
            lateral_velocity_mps=old_point.point.lateral_velocity_mps,
            heading_rate_rps=old_point.point.heading_rate_rps,
            is_final=old_point.point.is_final,
        )
        points.append(PathPointWithLaneId(point=point, lane_ids=old_point.lane_ids))
    return serialize_message(
        T4PathWithLaneId(
            header=old_msg.header,
            points=points,
            left_bound=old_msg.left_bound,
            right_bound=old_msg.right_bound,
        ),
    )


def convert_auto_traffic_signal_array(old_msg: AutoTrafficSignalArray) -> bytes:
    new_msg = TrafficLightGroupArray(stamp=old_msg.header.stamp)
    for old_signal in old_msg.signals:
        old_signal: AutoTrafficSignal
        traffic_light_group = TrafficLightGroup(traffic_light_group_id=old_signal.map_primitive_id)
        for old_light in old_signal.lights:
            old_light: AutoTrafficLight
            traffic_light_element = TrafficLightElement(
                color=old_light.color,
                shape=old_light.shape,
                status=old_light.status,
                confidence=old_light.confidence,
            )
            traffic_light_group.elements.append(traffic_light_element)
        new_msg.traffic_light_groups.append(traffic_light_group)
    return serialize_message(new_msg)


def convert_traffic_signal_array(old_msg: TrafficSignalArray) -> bytes:
    new_msg = TrafficLightGroupArray(stamp=old_msg.stamp)
    for old_signal in old_msg.signals:
        old_signal: TrafficSignal
        traffic_light_group = TrafficLightGroup(traffic_light_group_id=old_signal.traffic_signal_id)
        for old_element in old_signal.elements:
            old_element: TrafficSignalElement
            traffic_light_element = TrafficLightElement(
                color=old_element.color,
                shape=old_element.shape,
                status=old_element.status,
                confidence=old_element.confidence,
            )
            traffic_light_group.elements.append(traffic_light_element)
        new_msg.traffic_light_groups.append(traffic_light_group)
    return serialize_message(new_msg)


def convert_traffic_light_group_array_v1_7(
    old_msg: TrafficLightGroupArrayV1_7,
) -> bytes:
    new_msg = TrafficLightGroupArray(stamp=old_msg.stamp)
    for old_group in old_msg.traffic_light_groups:
        new_group = TrafficLightGroup(
            traffic_light_group_id=old_group.traffic_light_group_id,
        )
        for old_element in old_group.elements:
            new_element = TrafficLightElement(
                color=old_element.color,
                shape=old_element.shape,
                status=old_element.status,
                confidence=old_element.confidence,
            )
            new_group.elements.append(new_element)
        new_msg.traffic_light_groups.append(new_group)
    return serialize_message(new_msg)


def convert_hadmap_route(old_msg: HADMapRoute) -> bytes:
    new_msg = LaneletRoute(
        header=old_msg.header,
        start_pose=old_msg.start_pose,
        goal_pose=old_msg.goal_pose,
        allow_modification=False,
    )
    for old_segment in old_msg.segments:
        new_segment = LaneletSegment(
            preferred_primitive=LaneletPrimitive(
                id=old_segment.preferred_primitive_id, primitive_type="lane"
            )
        )
        for old_primitive in old_segment.primitives:
            new_primitive = LaneletPrimitive(id=old_primitive.id, primitive_type="lane")
            new_segment.primitives.append(new_primitive)
        new_msg.segments.append(new_segment)
    return serialize_message(new_msg)


def deserialize_message_recursive(msg: bytes, type_name: str) -> tuple[Any, str]:
    try:
        return deserialize_message(msg, get_message(type_name)), type_name
    except Exception as e:  # noqa
        if type_name in TYPES_TO_UPDATE_VERSION:
            original_type_name = TYPES_TO_UPDATE_VERSION[type_name]
            return deserialize_message_recursive(msg, original_type_name)
        print(f"Failed to deserialize message of type {type_name}: {e}")  # noqa
        return msg, "unknown_type"


def convert_msg(topic_name: str, msg: bytes, type_map: dict) -> bytes:  # noqa
    # get old msg type
    old_type: str = type_map[topic_name]
    if (
        old_type not in TYPES_NOT_SIMPLY_REPLACED
        and old_type not in TYPES_TO_UPDATE_VERSION
        and old_type not in TYPES_TO_UPDATE_DATA
    ):
        return msg

    old_msg, old_type = deserialize_message_recursive(msg, old_type)

    if old_type == "autoware_perception_msgs_v1_7/msg/TrafficLightGroupArray":
        return convert_traffic_light_group_array_v1_7(old_msg)
    if old_type == "autoware_auto_control_msgs/msg/AckermannControlCommand":
        return convert_ackermann_control_command(old_msg)
    if old_type == "autoware_auto_planning_msgs/msg/PathWithLaneId":
        return convert_path_with_lane_id(old_msg)
    if old_type == "autoware_auto_planning_msgs/msg/HADMapRoute":
        return convert_hadmap_route(old_msg)
    if old_type == "autoware_auto_perception_msgs/msg/TrafficSignalArray":
        return convert_auto_traffic_signal_array(old_msg)
    if old_type == "autoware_perception_msgs/msg/TrafficSignalArray":
        return convert_traffic_signal_array(old_msg)
    if old_type == "sensor_msgs/msg/PointCloud2":
        return convert_pointcloud2(old_msg)
    if old_type == "unknown_type":
        return msg
    return serialize_message(old_msg)


def convert_metadata(input_metadata_path: str, output_metadata_path: str) -> None:
    with input_metadata_path.open() as f:
        input_metadata = yaml.safe_load(f)

    # key: topic_name, value: offered_qos_profiles
    qos_profiles = {
        topic["topic_metadata"]["name"]: topic["topic_metadata"]["offered_qos_profiles"]
        for topic in input_metadata["rosbag2_bagfile_information"]["topics_with_message_count"]
    }

    with output_metadata_path.open() as f:
        output_metadata = yaml.safe_load(f)

    for topic in output_metadata["rosbag2_bagfile_information"]["topics_with_message_count"]:
        topic_name = topic["topic_metadata"]["name"]
        if topic_name in qos_profiles:
            topic["topic_metadata"]["offered_qos_profiles"] = qos_profiles[topic_name]

    with output_metadata_path.open("w") as f:
        yaml.dump(output_metadata, f, default_flow_style=False)


def convert_bag(input_bag_path: str, output_bag_path: str) -> None:
    p_input = Path(input_bag_path)
    storage_type = "mcap"
    for _ in p_input.glob("*.db3"):
        storage_type = "sqlite3"
        break
    # open reader
    reader = create_reader(input_bag_path, storage_type)
    # open writer
    writer = create_writer(output_bag_path, storage_type)

    # create topic
    old_type_map = {}  # key: topic_name value: old_type's msg type
    for topic_type in reader.get_all_topics_and_types():
        old_type_map[topic_type.name] = topic_type.type
        new_topic_type = change_topic_type(
            topic_type,
        )
        writer.create_topic(new_topic_type)

    # copy data from input bag to output bag
    while reader.has_next():
        topic_name, msg, stamp = reader.read_next()
        new_msg = convert_msg(topic_name, msg, old_type_map)
        new_topic_name = TOPIC_NAME_REMAPPING.get(topic_name, topic_name)
        writer.write(new_topic_name, new_msg, stamp)

    # reindex to update metadata.yaml
    del writer
    Reindexer().reindex(get_storage_options(output_bag_path, storage_type))

    # rewrite qos_profiles to metadata.yaml
    input_metadata_path = Path(input_bag_path) / "metadata.yaml"
    output_metadata_path = Path(output_bag_path) / "metadata.yaml"
    if input_metadata_path.exists() and output_metadata_path.exists():
        convert_metadata(input_metadata_path, output_metadata_path)
