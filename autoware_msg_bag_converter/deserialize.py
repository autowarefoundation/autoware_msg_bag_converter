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

from typing import Any

from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

from autoware_msg_bag_converter.converters import TYPE_VERSION_FALLBACK


def deserialize_message_recursive(msg: bytes, type_name: str) -> tuple[Any, str]:
    try:
        return deserialize_message(msg, get_message(type_name)), type_name
    except Exception as e:  # noqa: BLE001
        if type_name in TYPE_VERSION_FALLBACK:
            original_type_name = TYPE_VERSION_FALLBACK[type_name]
            return deserialize_message_recursive(msg, original_type_name)
        print(f"Failed to deserialize message of type {type_name}: {e}")  # noqa: T201
        return msg, "unknown_type"
