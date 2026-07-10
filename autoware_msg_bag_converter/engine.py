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

from rclpy.serialization import serialize_message
from rosbag2_py import Reindexer
import yaml

from autoware_msg_bag_converter.bag import create_reader
from autoware_msg_bag_converter.bag import create_writer
from autoware_msg_bag_converter.bag import get_storage_options
from autoware_msg_bag_converter.converters import ConvertContext
from autoware_msg_bag_converter.converters import REGISTRY
from autoware_msg_bag_converter.converters import TOPIC_NAME_REMAPPING
from autoware_msg_bag_converter.converters import TYPE_NOT_SIMPLY_REPLACED
from autoware_msg_bag_converter.converters import TYPE_VERSION_FALLBACK
from autoware_msg_bag_converter.converters import TYPES_TO_UPDATE_DATA
from autoware_msg_bag_converter.converters import build_convert_context
from autoware_msg_bag_converter.converters import change_topic_type
from autoware_msg_bag_converter.converters import convert_by_type
from autoware_msg_bag_converter.deserialize import deserialize_message_recursive


def _needs_conversion(old_type: str) -> bool:
    return (
        old_type in TYPE_NOT_SIMPLY_REPLACED
        or old_type in TYPE_VERSION_FALLBACK
        or old_type in TYPES_TO_UPDATE_DATA
    )


def convert_msg(
    topic_name: str,
    msg: bytes,
    type_map: dict,
    context: ConvertContext,
) -> bytes:
    old_type: str = type_map[topic_name]
    if not _needs_conversion(old_type):
        return msg

    old_msg, resolved_type = deserialize_message_recursive(msg, old_type)

    if resolved_type == "unknown_type":
        return msg
    if resolved_type not in REGISTRY:
        return serialize_message(old_msg)

    return convert_by_type(resolved_type, old_msg, context=context)


def convert_metadata(input_metadata_path: Path, output_metadata_path: Path) -> None:
    with input_metadata_path.open() as f:
        input_metadata = yaml.safe_load(f)

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

    base_context = build_convert_context(input_bag_path, storage_type)

    reader = create_reader(input_bag_path, storage_type)
    writer = create_writer(output_bag_path, storage_type)

    old_type_map: dict[str, str] = {}
    for topic_type in reader.get_all_topics_and_types():
        old_type_map[topic_type.name] = topic_type.type
        writer.create_topic(change_topic_type(topic_type))

    while reader.has_next():
        topic_name, msg, stamp = reader.read_next()
        context = ConvertContext(
            turn_indicators_timeline=base_context.turn_indicators_timeline,
            stamp_ns=stamp,
        )
        new_msg = convert_msg(topic_name, msg, old_type_map, context)
        new_topic_name = TOPIC_NAME_REMAPPING.get(topic_name, topic_name)
        writer.write(new_topic_name, new_msg, stamp)

    del reader
    del writer
    Reindexer().reindex(get_storage_options(output_bag_path, storage_type))

    input_metadata_path = Path(input_bag_path) / "metadata.yaml"
    output_metadata_path = Path(output_bag_path) / "metadata.yaml"
    if input_metadata_path.exists() and output_metadata_path.exists():
        convert_metadata(input_metadata_path, output_metadata_path)
