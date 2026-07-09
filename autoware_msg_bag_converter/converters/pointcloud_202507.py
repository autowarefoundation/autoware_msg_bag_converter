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

"""2025/07: PointCloud2 field layout upgrade.

Copied and modified from:
https://github.com/Shin-kyoto/tier4_perception_dataset_tools/tree/main
"""

import os
import sys

import numpy as np
from rclpy.serialization import serialize_message
import ros2_numpy
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointField

TYPES_TO_UPDATE_DATA = [
    "sensor_msgs/msg/PointCloud2",
]

# import debugpy
# debugpy.listen(5678)
# debugpy.wait_for_client()

if os.environ.get("ROSBAG2_PY_TEST_WITH_RTLD_GLOBAL", None) is not None:
    # This is needed on Linux when compiling with clang/libc++.
    # TL;DR This makes class_loader work when using a python extension compiled with libc++.
    #
    # For the fun RTTI ABI details, see https://whatofhow.wordpress.com/2015/03/17/odr-rtti-dso/.
    sys.setdlopenflags(os.RTLD_GLOBAL | os.RTLD_LAZY)


def is_xyz_layout(msg: PointCloud2):
    if len(msg.fields) != 3:
        return False

    x_field: PointField = msg.fields[0]
    y_field: PointField = msg.fields[1]
    z_field: PointField = msg.fields[2]

    if (
        x_field.name != "x"
        or x_field.offset != 0
        or x_field.datatype != PointField.FLOAT32
        or x_field.count != 1
    ):
        return False

    if (
        y_field.name != "y"
        or y_field.offset != 4
        or y_field.datatype != PointField.FLOAT32
        or y_field.count != 1
    ):
        return False

    if (
        z_field.name != "z"
        or z_field.offset != 8
        or z_field.datatype != PointField.FLOAT32
        or z_field.count != 1
    ):
        return False

    return True


def is_old_xyzi_layout(msg: PointCloud2):
    if len(msg.fields) != 4:
        return False

    x_field: PointField = msg.fields[0]
    y_field: PointField = msg.fields[1]
    z_field: PointField = msg.fields[2]
    i_field: PointField = msg.fields[3]

    if (
        x_field.name != "x"
        or x_field.offset != 0
        or x_field.datatype != PointField.FLOAT32
        or x_field.count != 1
    ):
        return False

    if (
        y_field.name != "y"
        or y_field.offset != 4
        or y_field.datatype != PointField.FLOAT32
        or y_field.count != 1
    ):
        return False

    if (
        z_field.name != "z"
        or z_field.offset != 8
        or z_field.datatype != PointField.FLOAT32
        or z_field.count != 1
    ):
        return False

    if (
        i_field.name != "intensity"
        or i_field.offset != 12
        or i_field.datatype != PointField.FLOAT32
        or i_field.count != 1
    ) and (
        i_field.name != "intensity"
        or i_field.offset != 16
        or i_field.datatype != PointField.FLOAT32
        or i_field.count != 1
    ):
        return False

    return True


def is_old_xyzir_layout(msg: PointCloud2):
    if len(msg.fields) != 5:
        return False

    x_field: PointField = msg.fields[0]
    y_field: PointField = msg.fields[1]
    z_field: PointField = msg.fields[2]
    i_field: PointField = msg.fields[3]
    ring_field: PointField = msg.fields[4]

    if (
        x_field.name != "x"
        or x_field.offset != 0
        or x_field.datatype != PointField.FLOAT32
        or x_field.count != 1
    ):
        return False

    if (
        y_field.name != "y"
        or y_field.offset != 4
        or y_field.datatype != PointField.FLOAT32
        or y_field.count != 1
    ):
        return False

    if (
        z_field.name != "z"
        or z_field.offset != 8
        or z_field.datatype != PointField.FLOAT32
        or z_field.count != 1
    ):
        return False

    if (
        i_field.name != "intensity"
        or i_field.offset != 16
        or i_field.datatype != PointField.FLOAT32
        or i_field.count != 1
    ):
        return False

    if (
        ring_field.name != "ring"
        or ring_field.offset != 20
        or ring_field.datatype != PointField.UINT16
        or ring_field.count != 1
    ):
        return False

    return True


def is_old_xyziradrt_layout(msg: PointCloud2):
    if len(msg.fields) != 9:
        return False

    x_field: PointField = msg.fields[0]
    y_field: PointField = msg.fields[1]
    z_field: PointField = msg.fields[2]
    i_field: PointField = msg.fields[3]
    ring_field: PointField = msg.fields[4]
    azimuth_field: PointField = msg.fields[5]
    distance_field: PointField = msg.fields[6]
    return_type_field: PointField = msg.fields[7]
    time_stamp_field: PointField = msg.fields[8]

    if (
        x_field.name != "x"
        or x_field.offset != 0
        or x_field.datatype != PointField.FLOAT32
        or x_field.count != 1
    ):
        return False

    if (
        y_field.name != "y"
        or y_field.offset != 4
        or y_field.datatype != PointField.FLOAT32
        or y_field.count != 1
    ):
        return False

    if (
        z_field.name != "z"
        or z_field.offset != 8
        or z_field.datatype != PointField.FLOAT32
        or z_field.count != 1
    ):
        return False

    if (
        i_field.name != "intensity"
        or i_field.offset != 16
        or i_field.datatype != PointField.FLOAT32
        or i_field.count != 1
    ):
        return False

    if (
        ring_field.name != "ring"
        or ring_field.offset != 20
        or ring_field.datatype != PointField.UINT16
        or ring_field.count != 1
    ):
        return False

    if (
        azimuth_field.name != "azimuth"
        or azimuth_field.offset != 24
        or azimuth_field.datatype != PointField.FLOAT32
        or azimuth_field.count != 1
    ):
        return False

    if (
        distance_field.name != "distance"
        or distance_field.offset != 28
        or distance_field.datatype != PointField.FLOAT32
        or distance_field.count != 1
    ):
        return False

    if (
        return_type_field.name != "return_type"
        or return_type_field.offset != 32
        or return_type_field.datatype != PointField.UINT8
        or return_type_field.count != 1
    ) and (
        return_type_field.name != "return_type"
        or return_type_field.offset != 32
        or return_type_field.datatype != PointField.INT8
        or return_type_field.count != 1
    ):
        return False

    if (
        time_stamp_field.name != "time_stamp"
        or time_stamp_field.offset != 40
        or time_stamp_field.datatype != PointField.FLOAT64
        or time_stamp_field.count != 1
    ):
        return False

    return True


def is_new_xyzirc_layout(msg: PointCloud2):
    if len(msg.fields) != 6:
        return False

    x_field: PointField = msg.fields[0]
    y_field: PointField = msg.fields[1]
    z_field: PointField = msg.fields[2]
    i_field: PointField = msg.fields[3]
    return_type_field: PointField = msg.fields[4]
    channel_field: PointField = msg.fields[5]

    if (
        x_field.name != "x"
        or x_field.offset != 0
        or x_field.datatype != PointField.FLOAT32
        or x_field.count != 1
    ):
        return False

    if (
        y_field.name != "y"
        or y_field.offset != 4
        or y_field.datatype != PointField.FLOAT32
        or y_field.count != 1
    ):
        return False

    if (
        z_field.name != "z"
        or z_field.offset != 8
        or z_field.datatype != PointField.FLOAT32
        or z_field.count != 1
    ):
        return False

    if (
        i_field.name != "intensity"
        or i_field.offset != 12
        or i_field.datatype != PointField.UINT8
        or i_field.count != 1
    ):
        return False

    if (
        return_type_field.name != "return_type"
        or return_type_field.offset != 13
        or return_type_field.datatype != PointField.UINT8
        or return_type_field.count != 1
    ):
        return False

    if (
        channel_field.name != "channel"
        or channel_field.offset != 14
        or channel_field.datatype != PointField.UINT16
        or channel_field.count != 1
    ):
        return False

    return True


def is_new_xyzircaedt_layout(msg: PointCloud2):
    if len(msg.fields) != 10:
        return False

    x_field: PointField = msg.fields[0]
    y_field: PointField = msg.fields[1]
    z_field: PointField = msg.fields[2]
    i_field: PointField = msg.fields[3]
    return_type_field: PointField = msg.fields[4]
    channel_field: PointField = msg.fields[5]
    azimuth_field: PointField = msg.fields[6]
    elevation_field: PointField = msg.fields[7]
    distance_field: PointField = msg.fields[8]
    time_stamp_field: PointField = msg.fields[9]

    if (
        x_field.name != "x"
        or x_field.offset != 0
        or x_field.datatype != PointField.FLOAT32
        or x_field.count != 1
    ):
        return False

    if (
        y_field.name != "y"
        or y_field.offset != 4
        or y_field.datatype != PointField.FLOAT32
        or y_field.count != 1
    ):
        return False

    if (
        z_field.name != "z"
        or z_field.offset != 8
        or z_field.datatype != PointField.FLOAT32
        or z_field.count != 1
    ):
        return False

    if (
        i_field.name != "intensity"
        or i_field.offset != 12
        or i_field.datatype != PointField.UINT8
        or i_field.count != 1
    ):
        return False

    if (
        return_type_field.name != "return_type"
        or return_type_field.offset != 13
        or return_type_field.datatype != PointField.UINT8
        or return_type_field.count != 1
    ):
        return False

    if (
        channel_field.name != "channel"
        or channel_field.offset != 14
        or channel_field.datatype != PointField.UINT16
        or channel_field.count != 1
    ):
        return False

    if (
        azimuth_field.name != "azimuth"
        or azimuth_field.offset != 16
        or azimuth_field.datatype != PointField.FLOAT32
        or azimuth_field.count != 1
    ):
        return False

    if (
        elevation_field.name != "elevation"
        or elevation_field.offset != 20
        or elevation_field.datatype != PointField.FLOAT32
        or elevation_field.count != 1
    ):
        return False

    if (
        distance_field.name != "distance"
        or distance_field.offset != 24
        or distance_field.datatype != PointField.FLOAT32
        or distance_field.count != 1
    ):
        return False

    if (
        time_stamp_field.name != "time_stamp"
        or time_stamp_field.offset != 28
        or time_stamp_field.datatype != PointField.UINT32
        or time_stamp_field.count != 1
    ):
        return False

    return True


def convert_xyz_to_xyzirc(msg: PointCloud2):
    input_array: np.ndarray = ros2_numpy.numpify(msg)
    num_points = len(input_array["x"])
    converted_array = np.zeros(
        (num_points,),
        dtype=[
            ("x", np.float32),
            ("y", np.float32),
            ("z", np.float32),
            ("intensity", np.uint8),
            ("return_type", np.uint8),
            ("channel", np.uint16),
        ],
    )
    converted_array["x"] = input_array["x"]
    converted_array["y"] = input_array["y"]
    converted_array["z"] = input_array["z"]

    converted_msg = ros2_numpy.msgify(PointCloud2, converted_array)
    converted_msg.header = msg.header

    assert is_new_xyzirc_layout(converted_msg)

    return converted_msg


def convert_xyzi_to_xyzirc(msg: PointCloud2):
    input_array: np.ndarray = ros2_numpy.numpify(msg)
    num_points = len(input_array["intensity"])
    converted_array = np.zeros(
        (num_points,),
        dtype=[
            ("x", np.float32),
            ("y", np.float32),
            ("z", np.float32),
            ("intensity", np.uint8),
            ("return_type", np.uint8),
            ("channel", np.uint16),
        ],
    )
    converted_array["x"] = input_array["x"]
    converted_array["y"] = input_array["y"]
    converted_array["z"] = input_array["z"]
    # NOTE(Shin-kyoto): astypeをcasting="unsafe"で使用している
    converted_array["intensity"] = input_array["intensity"].astype(np.uint8)

    converted_msg = ros2_numpy.msgify(PointCloud2, converted_array)
    converted_msg.header = msg.header

    assert is_new_xyzirc_layout(converted_msg)

    return converted_msg


def convert_xyzir_to_xyzirc(msg: PointCloud2):
    input_array = ros2_numpy.numpify(msg)
    num_points = len(input_array)

    converted_array = np.zeros(
        (num_points,),
        dtype=[
            ("x", np.float32),
            ("y", np.float32),
            ("z", np.float32),
            ("intensity", np.uint8),
            ("return_type", np.uint8),
            ("channel", np.uint16),
        ],
    )

    converted_array["x"] = input_array["x"]
    converted_array["y"] = input_array["y"]
    converted_array["z"] = input_array["z"]
    converted_array["intensity"] = input_array["intensity"].astype(np.uint8)
    converted_array["channel"] = input_array["ring"]

    converted_msg = ros2_numpy.msgify(PointCloud2, converted_array)
    converted_msg.header = msg.header

    assert is_new_xyzirc_layout(converted_msg)

    return converted_msg


def convert_xyziradrt_to_xyzircaedt(msg: PointCloud2):
    input_array = ros2_numpy.numpify(msg)
    num_points = len(input_array)

    converted_array = np.zeros(
        (num_points,),
        dtype=[
            ("x", np.float32),
            ("y", np.float32),
            ("z", np.float32),
            ("intensity", np.uint8),
            ("return_type", np.uint8),
            ("channel", np.uint16),
            ("azimuth", np.float32),
            ("elevation", np.float32),
            ("distance", np.float32),
            ("time_stamp", np.uint32),
        ],
    )

    converted_array["x"] = input_array["x"]
    converted_array["y"] = input_array["y"]
    converted_array["z"] = input_array["z"]
    converted_array["intensity"] = input_array["intensity"].astype(np.uint8)
    converted_array["channel"] = input_array["ring"]

    converted_array["azimuth"] = input_array["azimuth"] * (np.pi / 18000.0)
    converted_array["distance"] = input_array["distance"]

    # Old timestamps are a double !
    msg_stamp_seconds = msg.header.stamp.sec + 1e-9 * msg.header.stamp.nanosec
    point_rel_seconds = input_array["time_stamp"] - msg_stamp_seconds
    point_rel_nanoseconds = (1e9 * point_rel_seconds).astype(np.uint64)
    assert (point_rel_nanoseconds < np.iinfo(np.uint32).max).all()
    converted_array["time_stamp"] = point_rel_nanoseconds.astype(np.uint32)

    converted_msg = ros2_numpy.msgify(PointCloud2, converted_array)
    converted_msg.header = msg.header

    assert is_new_xyzircaedt_layout(converted_msg)

    return converted_msg


def convert_pointcloud2(msg: PointCloud2) -> bytes:
    if is_old_xyzi_layout(msg):
        converted_msg = convert_xyzi_to_xyzirc(msg)
    elif is_old_xyzir_layout(msg):
        converted_msg = convert_xyzir_to_xyzirc(msg)
    elif is_old_xyziradrt_layout(msg):
        converted_msg = convert_xyziradrt_to_xyzircaedt(msg)
    elif is_xyz_layout(msg):
        converted_msg = convert_xyz_to_xyzirc(msg)
    elif is_new_xyzirc_layout(msg):
        converted_msg = msg
    else:
        print("Unsupported layout for PointCloud2 message. Skipping point cloud conversion.")
        converted_msg = msg

    return serialize_message(converted_msg)
