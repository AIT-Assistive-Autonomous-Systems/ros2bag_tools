# Copyright 2020 AIT Austrian Institute of Technology GmbH
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
from ros2bag_tools.filter.replace import ReplaceFilter
from ros2bag_tools.filter.extract import ExtractFilter
from ros2bag_tools.filter.composite import CompositeFilter
from example_interfaces.msg import String

import pytest


def test_composite_filter():
    filter = CompositeFilter({
        'extract': ExtractFilter
    })

    parser = argparse.ArgumentParser('composite')
    filter.add_arguments(parser)
    args = parser.parse_args(['-c', 'test/composite.config'])

    in_file = '/dev/null'
    out_file = '/dev/null'
    filter.set_args(in_file, out_file, args)
    assert(filter.filter_msg(('/data', None, 0)) is None)


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

    in_file = '/dev/null'
    out_file = '/dev/null'
    filter.set_args(in_file, out_file, args)

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
