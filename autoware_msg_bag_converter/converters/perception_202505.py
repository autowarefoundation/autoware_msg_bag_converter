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

"""2025/05: velocity-limit passthrough + TrafficLightGroupArray IDL upgrade."""

from autoware_perception_msgs.msg import TrafficLightElement
from autoware_perception_msgs.msg import TrafficLightGroup
from autoware_perception_msgs.msg import TrafficLightGroupArray
from autoware_perception_msgs_v1_7.msg import TrafficLightGroupArray as TrafficLightGroupArrayV1_7
from rclpy.serialization import serialize_message

TYPE_NAME_REMAPPING = {
    "tier4_planning_msgs/msg/VelocityLimit": "autoware_internal_planning_msgs/msg/VelocityLimit",
    "tier4_planning_msgs/msg/ClearVelocityLimit": (
        "autoware_internal_planning_msgs/msg/ClearVelocityLimit"
    ),
}

TYPE_VERSION_FALLBACK = {
    "autoware_perception_msgs/msg/TrafficLightGroupArray": (
        "autoware_perception_msgs_v1_7/msg/TrafficLightGroupArray"
    ),
}


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
