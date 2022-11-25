#!/usr/bin/python3
# Copyright 2021 AIT Austrian Institute of Technology GmbH
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
"""Create the test.bag file."""
import sys
from rclpy.time import CONVERSION_CONSTANT
from rclpy.serialization import serialize_message
from rosbag2_tools import default_rosbag_options
from rosbag2_py import SequentialWriter, TopicMetadata
from example_interfaces.msg import String
from diagnostic_msgs.msg import DiagnosticArray


def create_test_bag(path):
    writer = SequentialWriter()
    storage_options, converter_options = default_rosbag_options(path)
    writer.open(storage_options, converter_options)

    topic = TopicMetadata('/data', 'example_interfaces/msg/String', 'cdr')
    writer.create_topic(topic)

    msg = String()
    msg.data = 'test_start'
    writer.write('/data', serialize_message(msg), 1000)
    msg.data = 'test_end'
    writer.write('/data', serialize_message(msg), CONVERSION_CONSTANT + 2000)


def create_diagnostics_bag(path):
    writer = SequentialWriter()
    storage_options, converter_options = default_rosbag_options(path)
    writer.open(storage_options, converter_options)

    topic = TopicMetadata('/diagnostics', 'diagnostic_msgs/msg/DiagnosticArray', 'cdr')
    writer.create_topic(topic)

    msg = DiagnosticArray()
    writer.write('/diagnostics', serialize_message(msg), 1000)


def create_day_time_bag(path):
    writer = SequentialWriter()
    storage_options, converter_options = default_rosbag_options(path)
    writer.open(storage_options, converter_options)

    topic = TopicMetadata('/data', 'example_interfaces/msg/String', 'cdr')
    writer.create_topic(topic)

    HOUR_TO_NS = 60 * 60 * CONVERSION_CONSTANT

    msg = String()
    msg.data = 'msg0'
    writer.write('/data', serialize_message(msg), 13 * HOUR_TO_NS - 1000)
    msg.data = 'msg1'
    writer.write('/data', serialize_message(msg), 13 * HOUR_TO_NS)
    msg.data = 'msg2'
    writer.write('/data', serialize_message(msg), 14 * HOUR_TO_NS)
    msg.data = 'msg2'
    writer.write('/data', serialize_message(msg), 14 * HOUR_TO_NS + 1000)


create_day_time_bag(sys.argv[1])
