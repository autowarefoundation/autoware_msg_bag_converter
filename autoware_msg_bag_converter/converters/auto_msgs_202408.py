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

"""2024/08: autoware_auto_* migration with payload rewrites."""

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
from autoware_planning_msgs.msg import LaneletPrimitive
from autoware_planning_msgs.msg import LaneletRoute
from autoware_planning_msgs.msg import LaneletSegment
from autoware_planning_msgs.msg import PathPoint
from rclpy.serialization import serialize_message
from tier4_planning_msgs.msg import PathPointWithLaneId
from tier4_planning_msgs.msg import PathWithLaneId as T4PathWithLaneId

if TYPE_CHECKING:
    from autoware_auto_perception_msgs.msg import TrafficLight as AutoTrafficLight
    from autoware_auto_perception_msgs.msg import TrafficSignal as AutoTrafficSignal
    from autoware_perception_msgs.msg import TrafficSignal
    from autoware_perception_msgs.msg import TrafficSignalElement

TYPE_NOT_SIMPLY_REPLACED = {
    "autoware_auto_control_msgs/msg/AckermannControlCommand": "autoware_control_msgs/msg/Control",
    "autoware_auto_planning_msgs/msg/PathWithLaneId": "tier4_planning_msgs/msg/PathWithLaneId",
    "autoware_auto_planning_msgs/msg/HADMapRoute": "autoware_planning_msgs/msg/LaneletRoute",
    "autoware_auto_perception_msgs/msg/TrafficSignalArray": (
        "autoware_perception_msgs/msg/TrafficLightGroupArray"
    ),
    "autoware_perception_msgs/msg/TrafficSignalArray": (
        "autoware_perception_msgs/msg/TrafficLightGroupArray"
    ),
}

PREFIX_PACKAGES = [
    "control_validator/msg",
    "planning_validator/msg",
    "vehicle_cmd_gate/msg",
]


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
