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
import os
from datetime import datetime, timezone
from rosbag2_py import (
    SequentialReader,
    SequentialWriter,
    StorageOptions,
    ConverterOptions,
)
from ros2bag.api import print_error
from ros2bag.verb import VerbExtension
from ros2bag_tools.filter import FilterResult
from ros2bag_tools.time import ros_time_from_nanoseconds, ros_to_datetime_utc


def get_rosbag_options(path, serialization_format='cdr'):
    storage_options = StorageOptions(uri=path, storage_id='sqlite3')
    converter_options = ConverterOptions(
        input_serialization_format=serialization_format,
        output_serialization_format=serialization_format)
    return storage_options, converter_options


class ProgressTracker:

    def __init__(self):
        self._i = 0
        self._expected_topics = set()
        self._no_of_expected_messages = 0

    def add_estimated_work(self, reader, storage_filter=None):
        metadata = reader.get_metadata()
        start = metadata.starting_time.astimezone(timezone.utc)
        end = start + metadata.duration
        filter_start = start
        filter_end = end
        if storage_filter:
            if hasattr(storage_filter, 'start_time'):
                filter_start = ros_to_datetime_utc(
                    ros_time_from_nanoseconds(storage_filter.start_time))
            if hasattr(storage_filter, 'stop_time'):
                filter_end = ros_to_datetime_utc(
                    ros_time_from_nanoseconds(storage_filter.stop_time))
        if start < filter_start:
            start = filter_start
        if filter_end < end:
            end = filter_end
        filter_factor = min(1, max(0, (end - start) / metadata.duration))
        for topic in metadata.topics_with_message_count:
            n = topic.message_count
            topic_name = topic.topic_metadata.name
            if not storage_filter or (not storage_filter.topics or topic_name in storage_filter.topics):
                self._expected_topics.add(topic_name)

                # assume that messages are spread uniformly across filtered timespan
                # this might not be true, but gives a good enough estimation for a
                # progress bar
                self._no_of_expected_messages += int(n * filter_factor)

    @property
    def n_processed(self):
        return self._i

    @property
    def n_expected(self):
        return self._no_of_expected_messages

    def update(self, topic) -> float:
        """
        Call when message of topic was processed.

        Return progress as number between 0.0 and 1.0.
        """
        if self._no_of_expected_messages <= 0:
            return 1.0
        if topic in self._expected_topics:
            self._i += 1
        return min(1, (self._i + 1) / self._no_of_expected_messages)

    def print_update(self, update, every=1):
        if self._i % every != 0:
            return
        values = (update, self.n_processed, self.n_expected)
        print("{0[0]:.2%} {0[1]}/{0[2]} ...".format(values), end='\r')

    def print_finish(self):
        # print done and clear to end of line
        print("100% Done\033[K")


class FilterVerb(VerbExtension):
    """Abstract base class for bag message processing verbs."""

    def __init__(self, filter):
        self._filter = filter

    def add_arguments(self, parser, _cli_name):  # noqa: D102
        parser.add_argument(
            'bag_files', nargs='+', help='input bag files')
        parser.add_argument(
            '-o', '--output',
            help='destination of the bagfile to create, \
            defaults to a timestamped folder in the current directory')
        parser.add_argument(
            '-s', '--in-storage',
            help='storage identifier to be used for the input bag, defaults to "sqlite3"')
        parser.add_argument(
            '-b', '--max-bag-size', type=int, default=0,
            help='maximum size in bytes before the bagfile will be split. '
                  'Default it is zero, resulting in a single bagfile and disabled '
                  'splitting.'
        )
        parser.add_argument(
            '--out-storage', default='sqlite3',
            help='storage identifier to be used for the output bag, defaults to "sqlite3"')
        parser.add_argument(
            '-f', '--serialization-format', default='',
            help='rmw serialization format in which the messages are saved, defaults to the'
                 ' rmw currently in use')
        parser.add_argument(
            '--progress', action='store_true',
            help='show progress bar')
        self._filter.add_arguments(parser)

    def main(self, *, args):  # noqa: D102
        for bag_file in args.bag_files:
            if not os.path.exists(bag_file):
                return print_error("bag file '{}' does not exist!".format(bag_file))

        uri = args.output or datetime.now().strftime('rosbag2_%Y_%m_%d-%H_%M_%S')

        if os.path.isdir(uri):
            return print_error("Output folder '{}' already exists.".format(uri))

        try:
            self._filter.set_args(args.bag_files, uri, args)
        except argparse.ArgumentError as e:
            return print_error(str(e))

        storage_filter = self._filter.get_storage_filter()

        progress = ProgressTracker()
        readers = []
        for bag_file in args.bag_files:
            reader = SequentialReader()
            in_storage_options, in_converter_options = get_rosbag_options(
                bag_file)
            if args.in_storage:
                in_storage_options.storage = args.in_storage
            reader.open(in_storage_options, in_converter_options)
            if storage_filter:
                reader.set_filter(storage_filter)
            if args.progress:
                progress.add_estimated_work(reader, storage_filter)
            readers.append(reader)

        writer = SequentialWriter()
        out_storage_options = StorageOptions(
            uri=uri, storage_id=args.out_storage, max_bagfile_size=args.max_bag_size)
        out_converter_options = ConverterOptions(
            input_serialization_format=args.serialization_format,
            output_serialization_format=args.serialization_format)
        writer.open(out_storage_options, out_converter_options)

        for reader in readers:
            for topic_metadata in reader.get_all_topics_and_types():
                result = self._filter.filter_topic(topic_metadata)
                if result:
                    if not isinstance(result, list):
                        result = [result]
                    for item in result:
                        writer.create_topic(item)

        for reader in readers:
            while reader.has_next():
                msg = reader.read_next()
                result = self._filter.filter_msg(msg)
                if args.progress:
                    prog_perc = progress.update(msg[0])
                    progress.print_update(prog_perc)
                if result == FilterResult.STOP_CURRENT_BAG:
                    break
                elif result == FilterResult.DROP_MESSAGE:
                    continue
                elif isinstance(result, list):
                    for item in result:
                        writer.write(item[0], item[1], item[2])
                elif isinstance(result, tuple):
                    writer.write(result[0], result[1], result[2])
                else:
                    return print_error("Filter returned invalid result: '{}'.".format(result))
        if args.progress:
            progress.print_finish()
