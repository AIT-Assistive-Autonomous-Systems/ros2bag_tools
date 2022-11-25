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
from datetime import datetime
from rosbag2_py import (
    Info,
    SequentialReader,
    SequentialWriter,
    StorageOptions,
    ConverterOptions,
    get_registered_readers,
    get_registered_writers
)
from ros2bag_tools.progress import ProgressTracker

ReadOrder = None
ReadOrderSortBy = None

try:
    from rosbag2_py import (
        ReadOrder,
        ReadOrderSortBy
    )
except:
    pass

from ros2bag.api import print_error
from ros2bag.verb import VerbExtension
from ros2bag_tools.filter import FilterResult

def get_rosbag_options(args):
    """Get rosbag options from args matching the ros2bag.api.add_standard_reader_args names"""
    storage_id = args.storage if hasattr(args,'storage') else 'sqlite3'
    serialization_format = (
        args.serialization_format if hasattr(args,'serialization_format') else 'cdr'
    )
    max_bagfile_size = args.max_bag_size if hasattr(args, 'max_bag_size') else 0
    storage_options = StorageOptions(
        uri=args.bag_path,
        storage_id=storage_id,
        max_bagfile_size=max_bagfile_size)
    converter_options = ConverterOptions(
        input_serialization_format=serialization_format,
        output_serialization_format=serialization_format)
    return storage_options, converter_options


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
            '-s', '--in-storage', default='sqlite3',
            choices = get_registered_readers(),
            help='storage identifier to be used for the input bag, defaults to "sqlite3"')
        parser.add_argument(
            '-b', '--max-bag-size', type=int, default=0,
            help='maximum size in bytes before the bagfile will be split. '
                  'Default it is zero, resulting in a single bagfile and disabled '
                  'splitting.'
        )
        parser.add_argument(
            '--out-storage', default='sqlite3',
            choices = get_registered_writers(),
            help='storage identifier to be used for the output bag, defaults to "sqlite3"')
        parser.add_argument(
            '-f', '--serialization-format', default='cdr',
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

        info = Info()
        metadatas = [info.read_metadata(f, args.in_storage or '') for f in args.bag_files]
        try:
            self._filter.set_args(metadatas, args)
        except argparse.ArgumentError as e:
            return print_error(str(e))

        storage_filter = self._filter.get_storage_filter()

        args_in_bag = vars(args).copy()
        args_in_bag.pop('bag_files')
        args_out_bag = args_in_bag.copy()
        args_in_bag['storage'] = args.in_storage
        args_in_bag['max_bag_size'] = 0
        args_out_bag['storage'] = args.out_storage
        args_out_bag['bag_path'] = uri

        progress = ProgressTracker()
        readers = []
        for bag_file, metadata in zip(args.bag_files, metadatas):
            reader = SequentialReader()
            args_in_bag['bag_path'] = bag_file
            if ReadOrder:
                reader.set_read_order(ReadOrder(ReadOrderSortBy.ReceivedTimestamp))
            in_storage_options, in_converter_options = get_rosbag_options(
                argparse.Namespace(**args_in_bag))
            reader.open(in_storage_options, in_converter_options)
            if storage_filter:
                reader.set_filter(storage_filter)
            if args.progress:
                progress.add_estimated_work(metadata, self._filter.output_size_factor(metadata))
            readers.append(reader)

        writer = SequentialWriter()
        out_storage_options, out_converter_options = get_rosbag_options(
            argparse.Namespace(**args_out_bag))
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
