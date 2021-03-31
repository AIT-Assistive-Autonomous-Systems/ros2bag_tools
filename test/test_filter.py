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

import argparse
from rclpy.serialization import serialize_message, deserialize_message
from rosbag2_py import TopicMetadata
from ros2bag_tools.filter import FilterResult
from ros2bag_tools.filter.composite import CompositeFilter
from ros2bag_tools.filter.cut import CutFilter
from ros2bag_tools.filter.extract import ExtractFilter
from ros2bag_tools.filter.reframe import ReframeFilter
from ros2bag_tools.filter.rename import RenameFilter
from ros2bag_tools.filter.replace import ReplaceFilter
from ros2bag_tools.filter.restamp import RestampFilter
from example_interfaces.msg import String
from diagnostic_msgs.msg import DiagnosticArray


import pytest


def test_composite_filter():
    filter = CompositeFilter({
        'extract': ExtractFilter
    })

    parser = argparse.ArgumentParser('composite')
    filter.add_arguments(parser)
    args = parser.parse_args(['-c', 'test/composite.config'])

    in_files = ['/dev/null']
    out_file = '/dev/null'
    filter.set_args(in_files, out_file, args)
    assert(filter.filter_msg(('/data', None, 0)) == FilterResult.DROP_MESSAGE)


def test_extract_filter():
    filter = ExtractFilter()

    parser = argparse.ArgumentParser('extract')
    filter.add_arguments(parser)
    args = parser.parse_args(['-t', '/data'])

    in_file = '/dev/null'
    out_file = '/dev/null'
    filter.set_args(in_file, out_file, args)
    msg = ('/data', None, 0)
    assert(filter.filter_msg(msg) == msg)


def test_replace_filter():
    filter = ReplaceFilter()

    parser = argparse.ArgumentParser('replace')
    filter.add_arguments(parser)
    args = parser.parse_args(['-t', '/data', '-v', 'test/data.yaml'])

    in_files = ['/dev/null']
    out_file = '/dev/null'
    filter.set_args(in_files, out_file, args)

    string_msg = String()
    string_msg.data = 'in'
    msg = ('/data', serialize_message(string_msg), 0)

    with pytest.raises(RuntimeError):
        # if topic hasn't been found, filter_msg fails with error
        filter.filter_msg(msg)

    topic_metadata = TopicMetadata('/data', 'example_interfaces/msg/String', 'cdr')
    assert(filter.filter_topic(topic_metadata) == topic_metadata)
    (topic, result_data, t) = filter.filter_msg(msg)
    assert(topic == '/data')
    result_msg = deserialize_message(result_data, String)
    assert(result_msg.data == 'out')
    assert(t == 0)


def test_cut_filter():
    filter = CutFilter()

    parser = argparse.ArgumentParser('cut')
    filter.add_arguments(parser)
    args = parser.parse_args(['--duration', '0.5'])

    in_files = ['test/test.bag']
    out_file = '/dev/null'
    filter.set_args(in_files, out_file, args)

    string_msg = String()
    string_msg.data = 'in'

    # timestamp within the bag duration
    msg = ('/data', serialize_message(string_msg), 0)
    assert(filter.filter_msg(msg) == FilterResult.DROP_MESSAGE)

    # exact start timestamp
    msg = ('/data', serialize_message(string_msg), 100)
    assert(filter.filter_msg(msg) == msg)

    # timestamp within the bag and cut duration
    msg = ('/data', serialize_message(string_msg), 500 * 1000 * 1000 - 200)
    assert(filter.filter_msg(msg) == msg)

    # timestamp after the requested duration, but before the last message in the test bag
    msg = ('/data', serialize_message(string_msg), 1000 * 1000 * 1000 + 200)
    assert(filter.filter_msg(msg) == FilterResult.STOP_CURRENT_BAG)


def test_reframe_filter():
    filter = ReframeFilter()

    parser = argparse.ArgumentParser('reframe')
    filter.add_arguments(parser)
    args = parser.parse_args(['-t', '/data', '--frame', 'frame1'])

    in_files = ['/dev/null']
    out_file = '/dev/null'
    filter.set_args(in_files, out_file, args)

    topic_metadata = TopicMetadata('/data', 'diagnostic_msgs/msg/DiagnosticArray', 'cdr')
    assert(filter.filter_topic(topic_metadata) == topic_metadata)

    msg = DiagnosticArray()
    msg.header.frame_id = 'frame0'
    msg.header.stamp.sec = 0
    msg.header.stamp.nanosec = 1

    # timestamp within the bag and cut duration
    bag_msg = ('/data', serialize_message(msg), 1)
    (_, data, _) = filter.filter_msg(bag_msg)
    new_msg = deserialize_message(data, DiagnosticArray)
    assert(new_msg.header.frame_id == 'frame1')


def test_rename_filter():
    filter = RenameFilter()

    parser = argparse.ArgumentParser('rename')
    filter.add_arguments(parser)
    args = parser.parse_args(['-t', '/data', '--name', '/renamed'])

    in_files = ['/dev/null']
    out_file = '/dev/null'
    filter.set_args(in_files, out_file, args)

    topic_metadata = TopicMetadata('/data', 'example_interfaces/msg/String', 'cdr')
    assert(filter.filter_topic(topic_metadata).name == '/renamed')

    msg = String()
    msg.data = 'test'

    # timestamp within the bag and cut duration
    bag_msg = ('/data', serialize_message(msg), 1)
    (topic, _, _) = filter.filter_msg(bag_msg)
    assert(topic == '/renamed')


def test_restamp_filter():
    filter = RestampFilter()

    parser = argparse.ArgumentParser('restamp')
    filter.add_arguments(parser)
    args = parser.parse_args([])

    in_files = ['/dev/null']
    out_file = '/dev/null'
    filter.set_args(in_files, out_file, args)

    topic_metadata = TopicMetadata('/data', 'diagnostic_msgs/msg/DiagnosticArray', 'cdr')
    assert(filter.filter_topic(topic_metadata) == topic_metadata)

    ns_stamp = 500 * 1000 * 1000 - 100
    msg = DiagnosticArray()
    msg.header.stamp.sec = 0
    msg.header.stamp.nanosec = ns_stamp

    # timestamp within the bag and cut duration
    bag_msg = ('/data', serialize_message(msg), 500 * 1000 * 1000)
    (_, _, t) = filter.filter_msg(bag_msg)
    assert(t == ns_stamp)
