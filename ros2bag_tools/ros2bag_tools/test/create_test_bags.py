# Copyright 2025 AIT Austrian Institute of Technology GmbH
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
from contextlib import closing

from typing import Sequence
from typing import Tuple

from diagnostic_msgs.msg import DiagnosticArray
from example_interfaces.msg import String

from rclpy.qos import QoSProfile
from rclpy.serialization import serialize_message

from rclpy.time import S_TO_NS
from rclpy.time import Time

from ros2bag_tools.filter import BagMessageTuple
from ros2bag_tools.topics import convert_rclpy_qos_to_rclcpp_qos
from ros2bag_tools.topics import create_metadata

from rosbag2_py import SequentialWriter
from rosbag2_py import TopicMetadata

from rosbag2_tools import default_rosbag_options

from sensor_msgs.msg import Image


def create_string_bag(path, qos_profiles: list[QoSProfile] = []):
    with closing(SequentialWriter()) as writer:
        storage_options, converter_options = default_rosbag_options(path)
        qos = convert_rclpy_qos_to_rclcpp_qos(qos_profiles)
        writer.open(storage_options, converter_options)

        topic = create_metadata('/data', 'example_interfaces/msg/String', 'cdr',
                                offered_qos_profiles=qos)

        writer.create_topic(topic)

        msg = String()
        msg.data = 'test_start'
        writer.write('/data', serialize_message(msg), 1000)
        msg.data = 'test_end'
        writer.write('/data', serialize_message(msg), S_TO_NS + 2000)


def create_diagnostics_bag(path, qos_profiles: list[QoSProfile] = []):
    with closing(SequentialWriter()) as writer:
        storage_options, converter_options = default_rosbag_options(path)
        qos = convert_rclpy_qos_to_rclcpp_qos(qos_profiles)
        writer.open(storage_options, converter_options)

        topic = create_metadata(
            '/diagnostics', 'diagnostic_msgs/msg/DiagnosticArray', 'cdr',
            offered_qos_profiles=qos)
        writer.create_topic(topic)

        msg = DiagnosticArray()
        writer.write('/diagnostics', serialize_message(msg), 1000)


def create_day_time_bag(path, qos_profiles: list[QoSProfile] = []):
    with closing(SequentialWriter()) as writer:
        storage_options, converter_options = default_rosbag_options(path)
        qos = convert_rclpy_qos_to_rclcpp_qos(qos_profiles)

        writer.open(storage_options, converter_options)

        topic = create_metadata('/data', 'example_interfaces/msg/String', 'cdr',
                                offered_qos_profiles=qos)
        writer.create_topic(topic)

        HOUR_TO_NS = 60 * 60 * S_TO_NS

        msg = String()
        msg.data = 'msg0'
        writer.write('/data', serialize_message(msg), 13 * HOUR_TO_NS - 1000)
        msg.data = 'msg1'
        writer.write('/data', serialize_message(msg), 13 * HOUR_TO_NS)
        msg.data = 'msg2'
        writer.write('/data', serialize_message(msg), 14 * HOUR_TO_NS)
        msg.data = 'msg2'
        writer.write('/data', serialize_message(msg), 14 * HOUR_TO_NS + 1000)


def create_images_bag(path, qos_profiles: list[QoSProfile] = []):
    with closing(SequentialWriter()) as writer:
        storage_options, converter_options = default_rosbag_options(path)
        qos = convert_rclpy_qos_to_rclcpp_qos(qos_profiles)
        writer.open(storage_options, converter_options)

        topic = create_metadata('/image', 'sensor_msgs/msg/Image', 'cdr',
                                offered_qos_profiles=qos)
        writer.create_topic(topic)
        for i in range(3):
            msg = Image()
            t = 1000 * 1000 * 1000 * i
            msg.header.frame_id = 'camera_optical_frame'
            msg.header.stamp.nanosec = t
            msg.width = 2
            msg.height = 2
            msg.step = 2
            msg.encoding = 'mono8'
            msg.data = [0, 128, 128, 255]
            writer.write('/image', serialize_message(msg), t)


def create_synced_bag(
        path,
        qos_profiles: list[QoSProfile] = []) -> Tuple[TopicMetadata, Sequence[BagMessageTuple]]:
    with closing(SequentialWriter()) as writer:
        storage_options, converter_options = default_rosbag_options(path)
        qos = convert_rclpy_qos_to_rclcpp_qos(qos_profiles)
        writer.open(storage_options, converter_options)

        topics = [create_metadata(topic, 'diagnostic_msgs/msg/DiagnosticArray', 'cdr',
                                  offered_qos_profiles=qos)
                  for topic in ['/sync0', '/sync1', '/offsync0']]
        for topic in topics:
            writer.create_topic(topic)
        entries = {
            # times in ms (match between /syncN = m, no match = x)
            # assumes 10ms match tolerance (exclusive)
            # True means should produce a match/sync.
            '/sync0':    [(0, True), (50, False), (80, False), (100, True), (200, False),
                          (300, False)],
            '/sync1':    [(0, True), (109, True), (210, False), (311, False)],
            '/offsync0': [(20, True), (120, True), (220, True), (320, True)],  # all pass through
        }
        for topic, ts in entries.items():
            for ms, _ in ts:
                ns = int(ms*1e6)
                t = Time(nanoseconds=ns)
                msg = DiagnosticArray()
                msg.header.stamp = t.to_msg()
                writer.write(topic, serialize_message(msg), ns)
        return entries
