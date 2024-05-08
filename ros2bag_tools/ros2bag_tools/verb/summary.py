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

from array import array

import numpy as np
from rclpy.serialization import deserialize_message
from rclpy.time import Time
from ros2bag.api import add_standard_reader_args
from ros2bag.verb import VerbExtension

from ros2bag_tools.progress import ProgressTracker
from ros2bag_tools.time import metadelta_to_timedelta
from ros2bag_tools.verb import get_reader_options

from rosbag2_py import Info
from rosbag2_py import SequentialReader
from rosbag2_py import StorageFilter
from rosidl_runtime_py.utilities import get_message


def generic_equals(ref_val, val):
    """Equals which handles the case of array values, which all have to match."""
    if ref_val is None:
        return True
    cmp = ref_val == val
    if isinstance(cmp, np.ndarray) or isinstance(cmp, array):
        return cmp.all()
    return cmp


class ConstantFieldSummaryOutput:
    """Get constant value of a specific field and add to summary."""

    def __init__(self, field_name):
        self._field_name = field_name
        self._value = None

    def update(self, message):
        assert(hasattr(message, self._field_name))
        value = getattr(message, self._field_name)
        # ensure the value is actually constant across the bag
        assert generic_equals(self._value, value)
        self._value = value

    def write(self):
        print(f'\t{self._field_name}: {self._value}')


class ValueRangeSummaryOutput:
    """Accumulate values of a field and print statistics."""

    def __init__(self, field_name):
        self._field_name = field_name
        self._values = []

    def update(self, message):
        assert(hasattr(message, self._field_name))
        value = getattr(message, self._field_name)
        self._values.append(value)

    def write(self):
        values = np.array(self._values)
        mean = np.mean(values)
        stddev = np.std(values)
        print(f'\t{self._field_name}: mean {mean:.3f} (stddev {stddev:.3f})')


def default_summary_output(message_type):
    if message_type == 'sensor_msgs/msg/Image':
        return [ConstantFieldSummaryOutput(field) for field in ['width', 'height', 'encoding']]
    if message_type == 'sensor_msgs/msg/CameraInfo':
        return [ConstantFieldSummaryOutput(field) for field
                in ['distortion_model', 'roi', 'width', 'height', 'd', 'k', 'r', 'p']]
    if message_type == 'sensor_msgs/msg/NavSatFix':
        return [ValueRangeSummaryOutput(field) for field in ['latitude', 'longitude', 'altitude']]
    return []


class SummaryVerb(VerbExtension):
    """Print a summary of the contents of a bag."""

    def add_arguments(self, parser, cli_name):  # noqa: D102
        add_standard_reader_args(parser)
        parser.add_argument(
            '--progress', default=False, action='store_true',
            help='display reader progress in terminal')
        parser.add_argument('-t', '--topic', nargs='*', type=str,
                            help='topics to summarize, summarize all if empty')

    def main(self, *, args):  # noqa: D102
        if not args.topic:
            args.topic = []

        reader = SequentialReader()
        in_storage_options, in_converter_options = get_reader_options(args)
        reader.open(in_storage_options, in_converter_options)

        info = Info()
        metadata = info.read_metadata(args.bag_path, args.storage)
        message_counts = {}
        for entry in metadata.topics_with_message_count:
            message_counts[entry.topic_metadata.name] = entry.message_count
        bag_duration_s = metadelta_to_timedelta(metadata.duration).total_seconds()

        type_name_to_type_map = {}
        topic_to_type_map = {}
        summaries = {}

        for topic_metadata in reader.get_all_topics_and_types():
            if args.topic and topic_metadata.name not in args.topic:
                continue
            if topic_metadata.type not in type_name_to_type_map:
                try:
                    type_name_to_type_map[topic_metadata.type] = get_message(
                        topic_metadata.type)
                except (AttributeError, ModuleNotFoundError, ValueError):
                    raise RuntimeError(
                        f"Cannot load message type '{topic_metadata.type}'")
            topic_to_type_map[topic_metadata.name] = type_name_to_type_map[topic_metadata.type]
            summaries[topic_metadata.name] = {
                'frame_ids': set(),
                'write_delays_ns': [],
                'custom': default_summary_output(topic_metadata.type)
            }

        reader.set_filter(StorageFilter(topics=args.topic))

        progress = ProgressTracker()
        if args.progress:
            progress.add_estimated_work(metadata, 1.0)

        while reader.has_next():
            (topic, data, t) = reader.read_next()
            msg_type = topic_to_type_map[topic]
            msg = deserialize_message(data, msg_type)
            for custom in summaries[topic]['custom']:
                custom.update(msg)
            if hasattr(msg, 'header'):
                summaries[topic]['frame_ids'].add(msg.header.frame_id)
                delay = t - Time.from_msg(msg.header.stamp).nanoseconds
                summaries[topic]['write_delays_ns'].append(delay)
            if args.progress:
                progress.print_update(progress.update(topic), every=100)

        if args.progress:
            progress.print_finish()

        for topic, summary in summaries.items():
            print(topic)
            if not message_counts[topic]:
                print('\tNo messages')
                continue
            frame_id_str = ', '.join(summary['frame_ids'])
            print(f'\tframe_id: {frame_id_str}')
            freq = message_counts[topic] / bag_duration_s
            print(f'\tfrequency: {freq:.2f} hz')
            if summary['write_delays_ns']:
                # only messages with header.stamp have delays
                write_delays = np.array(summary['write_delays_ns'])
                delay_ms_mean = np.mean(write_delays) / 1000 / 1000
                delay_ms_stddev = np.std(write_delays) / 1000 / 1000
                print(
                    f'\twrite delay: {delay_ms_mean:.2f}ms (stddev {delay_ms_stddev:.2f})')
                for custom in summaries[topic]['custom']:
                    custom.write()
