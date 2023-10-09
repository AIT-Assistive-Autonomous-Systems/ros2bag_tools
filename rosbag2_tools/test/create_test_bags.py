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
"""Create test bag files."""
from rclpy.serialization import serialize_message
from rosbag2_py import SequentialWriter, StorageOptions, ConverterOptions, TopicMetadata
from sensor_msgs.msg import Range
from diagnostic_msgs.msg import KeyValue


def create_range_bag(path):
    writer = SequentialWriter()
    storage_options = StorageOptions(uri=path)
    converter_options = ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr')
    writer.open(storage_options, converter_options)

    topic = TopicMetadata('/range', 'sensor_msgs/msg/Range', 'cdr')
    writer.create_topic(topic)

    msg = Range()
    msg.header.stamp.sec = 0
    msg.header.stamp.nanosec = 90
    msg.range = 10.0
    writer.write('/range', serialize_message(msg), 100)
    msg.header.stamp.sec = 0
    msg.header.stamp.nanosec = 190
    msg.range = 20.0
    writer.write('/range', serialize_message(msg), 200)


def create_multi_topic_bag(path):
    writer = SequentialWriter()
    storage_options = StorageOptions(uri=path)
    converter_options = ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr')
    writer.open(storage_options, converter_options)

    topic = TopicMetadata('/range', 'sensor_msgs/msg/Range', 'cdr')
    writer.create_topic(topic)

    topic = TopicMetadata('/diagnostics', 'diagnostic_msgs/msg/KeyValue', 'cdr')
    writer.create_topic(topic)

    msg = Range()
    msg.header.stamp.sec = 0
    msg.header.stamp.nanosec = 90
    msg.range = 10.0
    writer.write('/range', serialize_message(msg), 100)

    msg = KeyValue()
    msg.key = 'cpu'
    msg.value = 'warn'
    writer.write('/diagnostics', serialize_message(msg), 200)
