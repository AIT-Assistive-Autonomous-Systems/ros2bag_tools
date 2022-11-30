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
from typing import Sequence, Tuple
from rclpy.serialization import serialize_message, deserialize_message
from rosbag2_py import Info, TopicMetadata
from ros2bag_tools.filter import FilterResult, BagMessageTuple
from ros2bag_tools.filter.add import AddFilter
from ros2bag_tools.filter.cut import CutFilter
from ros2bag_tools.filter.extract import ExtractFilter
from ros2bag_tools.filter.reframe import ReframeFilter
from ros2bag_tools.filter.rename import RenameFilter
from ros2bag_tools.filter.replace import ReplaceFilter
from ros2bag_tools.filter.restamp import RestampFilter
from ros2bag_tools.filter.sync import SyncFilter
from ros2bag_tools.reader import FilteredReader
from example_interfaces.msg import String
from diagnostic_msgs.msg import DiagnosticArray

import pytest


from .create_test_bags import create_synced_bag

@pytest.fixture(scope="session")
def dummy_synced_bag(tmp_path_factory) -> Tuple[
        str, Sequence[TopicMetadata], Sequence[BagMessageTuple]]:
    bag_path = tmp_path_factory.mktemp('bags')

    synced_bag = str(bag_path / 'synced_bag')

    topics, synced_topics, synced_msgs = create_synced_bag(synced_bag)
    
    return synced_bag, synced_topics, topics, synced_msgs


def read_metadata(path: str):
    info = Info()
    return info.read_metadata(path, '')


def test_add_filter():
    filter = AddFilter()

    parser = argparse.ArgumentParser('add')
    filter.add_arguments(parser)
    args = parser.parse_args(
        ['-t', '/data', '--type', 'example_interfaces/msg/String', '-v', 'test/data.yaml',
         '--align-to', '/align'])
    filter.set_args(None, args)

    topic_metadata = TopicMetadata(
        '/align', 'example_interfaces/msg/String', 'cdr')
    topics = filter.filter_topic(topic_metadata)
    assert(len(topics) == 2)
    assert(topics[0].name == '/align')
    assert(topics[0].type == 'example_interfaces/msg/String')
    assert(topics[1].name == '/data')
    assert(topics[1].type == 'example_interfaces/msg/String')

    msg = String()
    msg.data = 'align'
    msgs = filter.filter_msg(('/align', serialize_message(msg), 1))

    assert(len(msgs) == 2)
    (topic0, data0, t0) = msgs[0]
    (topic1, data1, t1) = msgs[1]
    assert(topic0 == '/align')
    assert(topic1 == '/data')
    assert(t0 == t1)
    assert(deserialize_message(data0, String).data == 'align')
    assert(deserialize_message(data1, String).data == 'out')


def test_extract_filter():
    filter = ExtractFilter()

    parser = argparse.ArgumentParser('extract')
    filter.add_arguments(parser)
    args = parser.parse_args(['-t', '/data'])
    filter.set_args(None, args)

    msg = ('/data', None, 0)
    assert(filter.filter_msg(msg) == msg)


def test_replace_filter():
    filter = ReplaceFilter()

    parser = argparse.ArgumentParser('replace')
    filter.add_arguments(parser)
    args = parser.parse_args(['-t', '/data', '-v', 'test/data.yaml'])
    filter.set_args(None, args)

    string_msg = String()
    string_msg.data = 'in'
    msg = ('/data', serialize_message(string_msg), 0)

    with pytest.raises(RuntimeError):
        # if topic hasn't been found, filter_msg fails with error
        filter.filter_msg(msg)

    topic_metadata = TopicMetadata(
        '/data', 'example_interfaces/msg/String', 'cdr')
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
    filter.set_args([read_metadata('test/test.bag')], args)

    string_msg = String()
    string_msg.data = 'in'

    # timestamp before bag start
    msg = ('/data', serialize_message(string_msg), 0)
    assert(filter.filter_msg(msg) == FilterResult.DROP_MESSAGE)

    # exact start timestamp
    msg = ('/data', serialize_message(string_msg), 1000)
    assert(filter.filter_msg(msg) == msg)

    # timestamp within the bag and cut duration
    msg = ('/data', serialize_message(string_msg), 500 * 1000 * 1000 - 1000)
    assert(filter.filter_msg(msg) == msg)

    # timestamp after the requested duration, but before the last message in the test bag
    msg = ('/data', serialize_message(string_msg), 1000 * 1000 * 1000 + 1000)
    assert(filter.filter_msg(msg) == FilterResult.STOP_CURRENT_BAG)


def test_cut_filter_args():
    filter = CutFilter()

    parser = argparse.ArgumentParser('cut')
    filter.add_arguments(parser)

    args = parser.parse_args(['--duration', '3603'])
    with pytest.raises(argparse.ArgumentError):
        # duration is too long for bag
        filter.set_args([read_metadata('test/day_time.bag')], args)

    args = parser.parse_args(['--start', '13:15', '--end', '13:10'])
    with pytest.raises(argparse.ArgumentError):
        # end before start
        filter.set_args([read_metadata('test/day_time.bag')], args)

    args = parser.parse_args(['--start', '12:00', '--end', '12:59'])
    with pytest.raises(argparse.ArgumentError):
        # error since time bounds are not covered by bag
        filter.set_args([read_metadata('test/day_time.bag')], args)


def test_reframe_filter():
    filter = ReframeFilter()

    parser = argparse.ArgumentParser('reframe')
    filter.add_arguments(parser)
    args = parser.parse_args(['-t', '/diagnostics', '--frame', 'frame1'])
    filter.set_args(None, args)

    topic_metadata = TopicMetadata(
        '/diagnostics', 'diagnostic_msgs/msg/DiagnosticArray', 'cdr')
    assert(filter.filter_topic(topic_metadata) == topic_metadata)

    msg = DiagnosticArray()
    msg.header.frame_id = 'frame0'
    msg.header.stamp.sec = 0
    msg.header.stamp.nanosec = 1

    # timestamp within the bag and cut duration
    bag_msg = ('/diagnostics', serialize_message(msg), 1)
    (_, data, _) = filter.filter_msg(bag_msg)
    new_msg = deserialize_message(data, DiagnosticArray)
    assert(new_msg.header.frame_id == 'frame1')


def test_rename_filter():
    filter = RenameFilter()

    parser = argparse.ArgumentParser('rename')
    filter.add_arguments(parser)
    args = parser.parse_args(['-t', '/data', '--name', '/renamed'])
    filter.set_args(None, args)

    topic_metadata = TopicMetadata(
        '/data', 'example_interfaces/msg/String', 'cdr')
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
    filter.set_args([read_metadata('test/diagnostics.bag')], args)

    topic_metadata = TopicMetadata(
        '/diagnostics', 'diagnostic_msgs/msg/DiagnosticArray', 'cdr')
    assert(filter.filter_topic(topic_metadata) == topic_metadata)

    ns_stamp = 500 * 1000 * 1000 - 100
    msg = DiagnosticArray()
    msg.header.stamp.sec = 0
    msg.header.stamp.nanosec = ns_stamp

    # timestamp within the bag and cut duration
    bag_msg = ('/diagnostics', serialize_message(msg), 500 * 1000 * 1000)
    (_, _, t) = filter.filter_msg(bag_msg)
    assert(t == ns_stamp)

def test_sync_filter(dummy_synced_bag):
    bag_path, synced_topics, topics, synced_msgs = dummy_synced_bag
    assert(len(synced_msgs) > 0)

    test_filter = SyncFilter()
    reader = FilteredReader(bag_paths=[bag_path], filter=test_filter)
    parser = argparse.ArgumentParser('sync')
    test_filter.add_arguments(parser)
    args = parser.parse_args(["-t", *synced_topics, "--slop", "0.01"])
    test_filter.set_args([read_metadata(bag_path)], args)

    for meta in topics:
        assert(test_filter.filter_topic(meta) == meta)

    msg_type = type(synced_msgs[0][1])
    for msg in reader:
        # serializing the same message and comparing it returns False
        # thus compare deserialized message
        topic, msg, t = msg
        msg = (topic, deserialize_message(msg, msg_type), t)
        assert(msg in synced_msgs)
        synced_msgs.remove(msg)
    assert(len(synced_msgs) == 0)
