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
import logging
from pathlib import Path
from rclpy.serialization import serialize_message, deserialize_message
from rosbag2_py import Info, TopicMetadata
from ros2bag_tools.filter import FilterResult
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
from std_msgs.msg import String as RosString
from ros2bag_tools.verb.export import ExportVerb

from .logutils import capture_at_level

import pytest


pkg_prefix = Path(__file__).parents[1]


def read_metadata(path: str):
    info = Info()
    return info.read_metadata(path, '')


def test_add_filter():
    filter = AddFilter()

    parser = argparse.ArgumentParser('add')
    filter.add_arguments(parser)
    args = parser.parse_args(
        ['-t', '/data', '--type', 'example_interfaces/msg/String', '-v',
         str(pkg_prefix/'test'/'data.yaml'),
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
    args = parser.parse_args(['-t', '/data', '-v', str(pkg_prefix/'test'/'data.yaml')])
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


def test_cut_filter(tmp_string_bag):
    filter = CutFilter()

    parser = argparse.ArgumentParser('cut')
    filter.add_arguments(parser)
    args = parser.parse_args(['--duration', '0.5'])
    filter.set_args([read_metadata(tmp_string_bag)], args)

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


def test_cut_filter_args(tmp_day_time_bag):
    filter = CutFilter()

    parser = argparse.ArgumentParser('cut')
    filter.add_arguments(parser)

    args = parser.parse_args(['--duration', '3603'])
    with pytest.raises(argparse.ArgumentError):
        # duration is too long for bag
        filter.set_args([read_metadata(tmp_day_time_bag)], args)

    args = parser.parse_args(['--start', '13:15', '--end', '13:10'])
    with pytest.raises(argparse.ArgumentError):
        # end before start
        filter.set_args([read_metadata(tmp_day_time_bag)], args)

    args = parser.parse_args(['--start', '12:00', '--end', '12:59'])
    with pytest.raises(argparse.ArgumentError):
        # error since time bounds are not covered by bag
        filter.set_args([read_metadata(tmp_day_time_bag)], args)


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


def test_restamp_filter(tmp_diagnostics_bag: Path):
    filter = RestampFilter()
    info = Info()
    parser = argparse.ArgumentParser('restamp')
    filter.add_arguments(parser)
    args = parser.parse_args([])
    filter.set_args([info.read_metadata(tmp_diagnostics_bag, '')], args)

    topic_metadata = TopicMetadata(
        '/diagnostics', 'diagnostic_msgs/msg/DiagnosticArray', 'cdr')
    assert(filter.filter_topic(topic_metadata) == topic_metadata)

    msg = DiagnosticArray()
    msg.header.stamp.sec = 0
    msg.header.stamp.nanosec = 0

    bag_msg = ('/diagnostics', serialize_message(msg), 1)
    (_, data, t) = filter.filter_msg(bag_msg)
    msg_filtered = deserialize_message(data, DiagnosticArray)
    assert(t == 0)
    assert(msg_filtered.header.stamp.nanosec == 0)

    args = parser.parse_args(['--invert'])
    filter.set_args([info.read_metadata(tmp_diagnostics_bag, '')], args)
    (_, data, _) = filter.filter_msg(bag_msg)
    msg_filtered = deserialize_message(data, DiagnosticArray)
    assert(msg_filtered.header.stamp.nanosec == 1)

    args = parser.parse_args(
        ['--offset', '2', '--offset-topic', '/diagnostics'])
    filter.set_args([info.read_metadata(tmp_diagnostics_bag, '')], args)
    (_, data, t) = filter.filter_msg(bag_msg)
    msg_filtered = deserialize_message(data, DiagnosticArray)
    assert(t == 2)
    assert(msg_filtered.header.stamp.nanosec == 0)

    args = parser.parse_args(
        ['--offset-header', '--offset', '1', '--offset-topic', '/diagnostics'])
    filter.set_args([info.read_metadata(tmp_diagnostics_bag, '')], args)
    (_, data, t) = filter.filter_msg(bag_msg)
    msg_filtered = deserialize_message(data, DiagnosticArray)
    assert(t == 1)
    assert(msg_filtered.header.stamp.nanosec == 1)

    # No header tests
    topic_metadata = TopicMetadata(
        '/string', 'std_msgs/msg/String', 'cdr')
    assert(filter.filter_topic(topic_metadata) == topic_metadata)

    msg = RosString()
    msg.data = 'TestString'

    bag_msg = ('/string', serialize_message(msg), 1)
    (_, data, t) = filter.filter_msg(bag_msg)
    msg_filtered = deserialize_message(data, RosString)
    assert(t == 1)


def test_sync_filter(tmp_synced_bag):
    test_filter = SyncFilter()
    parser = argparse.ArgumentParser('sync')
    test_filter.add_arguments(parser)
    args = parser.parse_args(['-t', '/sync0', '/sync1', '--slop', '0.01'])
    test_filter.set_args([read_metadata(tmp_synced_bag)], args)

    reader = FilteredReader(bag_paths=[tmp_synced_bag], filter=test_filter)

    topics = [TopicMetadata(topic, 'diagnostic_msgs/msg/DiagnosticArray', 'cdr')
              for topic in ['/sync0', '/sync1', '/offsync0']]
    for meta in topics:
        assert(test_filter.filter_topic(meta) == meta)

    expected_counts = {
        # syncN should only be counted if they match within 10ms
        '/sync0': 2,
        '/sync1': 2,
        # offsync should pass through filter untouched
        '/offsync0': 1,
    }
    counts = {}
    for (topic, _, _) in reader:
        if topic in counts:
            counts[topic] += 1
        else:
            counts[topic] = 1
    assert(expected_counts == counts)


def test_export_sync_selected(caplog: pytest.LogCaptureFixture,
                              tmp_synced_bag,
                              synced_export_conf):
    filter_conf_path, export_conf_path = synced_export_conf

    parser = argparse.ArgumentParser('export')
    verb = ExportVerb()

    with capture_at_level(caplog, logging.INFO, 'pytest.export.sync(0)'):
        verb.add_arguments(parser, 'pytest.export')
        args = parser.parse_args(['-f', filter_conf_path, '-c', export_conf_path,
                                  '-i', tmp_synced_bag])
        verb.main(args=args)
    assert "total #synced-bundles: 2" in caplog.messages
    assert "total #off-sync msgs on '/sync0': 2" in caplog.messages
    assert "total #off-sync msgs on '/sync1': 2" in caplog.messages
