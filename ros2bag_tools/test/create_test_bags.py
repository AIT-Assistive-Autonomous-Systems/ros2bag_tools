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
from typing import Tuple, Sequence
from random import Random
from rclpy.time import CONVERSION_CONSTANT, Time
from rclpy.serialization import serialize_message
from rosbag2_tools import default_rosbag_options
from rosbag2_py import SequentialWriter, TopicMetadata
from example_interfaces.msg import String
from diagnostic_msgs.msg import DiagnosticArray
from sensor_msgs.msg import Image
from ros2bag_tools.filter import BagMessageTuple


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


def create_images_bag(path):
    writer = SequentialWriter()
    storage_options, converter_options = default_rosbag_options(path)
    writer.open(storage_options, converter_options)

    topic = TopicMetadata('/image', 'sensor_msgs/msg/Image', 'cdr')
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

def create_synced_bag(path) -> Tuple[TopicMetadata, Sequence[BagMessageTuple]]:
    writer = SequentialWriter()
    storage_options, converter_options = default_rosbag_options(path)
    writer.open(storage_options, converter_options)

    topics = [TopicMetadata('/sync1', 'sensor_msgs/msg/Image', 'cdr'),
        TopicMetadata('/sync2', 'sensor_msgs/msg/Image', 'cdr'),
        TopicMetadata('/sync3', 'sensor_msgs/msg/Image', 'cdr'),
        TopicMetadata('/offsync1', 'sensor_msgs/msg/Image', 'cdr')]

    synced_topics = [topic.name for topic in topics[0:3]]

    slop_ns = 10*int(1e6)
    dropped = []
    msgs = []

    gen = Random(42)

    t_offsets_ns = [0, 0, 0, 3*slop_ns]

    for i, topic in enumerate(topics):
        writer.create_topic(topic)
        for j in range(5):
            msg = Image()

            stamp = (Time(seconds=j+1, nanoseconds=0).nanoseconds +
                gen.randint(-slop_ns//2, slop_ns//2)) + t_offsets_ns[i]
            stamp = Time(nanoseconds=stamp)
            t = Time(seconds=j+1, nanoseconds=gen.randint(int(0), int(100*1e6)))
            msg.header.frame_id = f'camera{i}_optical_frame'
            msg.header.stamp = stamp.to_msg()
            msg.width = 2
            msg.height = 1
            msg.step = 2
            msg.encoding = 'mono8'
            msg.data = [i, j]

            bag_tuple = (topic.name, msg, t.nanoseconds)
            if i==1 and j in [1, 3]:
                dropped.append(bag_tuple)
                continue
            elif j != 1 or i == 3:
                msgs.append(bag_tuple)
            bag_tuple = (topic.name, serialize_message(msg), int(t.nanoseconds))
            writer.write(*bag_tuple)

    # write late message
    bag_tuple = (dropped[1][0], dropped[1][1], dropped[1][2] + int(400e9))
    msgs.append(bag_tuple)
    bag_tuple = (dropped[1][0], serialize_message(dropped[1][1]), dropped[1][2] + int(400e9))
    writer.write(*bag_tuple)
    return topics, synced_topics, msgs
        

#create_images_bag(sys.argv[1])
