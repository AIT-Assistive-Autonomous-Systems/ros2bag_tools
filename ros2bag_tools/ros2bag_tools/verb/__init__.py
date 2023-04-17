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
    SequentialWriter,
    StorageOptions,
    ConverterOptions,
    get_registered_readers,
    get_registered_writers
)
from ros2bag_tools.progress import ProgressTracker

from ros2bag_tools.reader import FilteredReader
from ros2bag_tools.logging import getLogger
from ros2bag.api import print_error
from ros2bag.verb import VerbExtension


def get_reader_options(args):
    """Get rosbag options from args matching the ros2bag.api.add_standard_reader_args names."""
    serialization_format = (
        args.serialization_format if hasattr(
            args, 'serialization_format') else 'cdr'
    )
    storage_options = StorageOptions(uri=args.bag_path)
    storage_id = args.storage if hasattr(args, 'storage') else ''
    if storage_id:
        storage_options.storage_id = storage_id
    converter_options = ConverterOptions(
        input_serialization_format=serialization_format,
        output_serialization_format=serialization_format)
    return storage_options, converter_options


class FilterVerb(VerbExtension):
    """Abstract base class for bag message processing verbs."""

    def __init__(self, filter):
        self._filter = filter

    def add_arguments(self, parser, cli_name):  # noqa: D102
        self._logger = getLogger(cli_name)
        parser.add_argument(
            'bag_files', nargs='+', help='input bag files')
        parser.add_argument(
            '-s', '--in-storage', default='',
            choices=get_registered_readers(),
            help='storage identifier to be used for the input bag')
        parser.add_argument(
            '-o', '--output',
            help='destination of the bagfile to create, \
            defaults to a timestamped folder in the current directory')
        parser.add_argument(
            '-b', '--max-bag-size', type=int, default=0,
            help='maximum size in bytes before the bagfile will be split. '
                  'Default it is zero, resulting in a single bagfile and disabled '
                  'splitting.'
        )
        parser.add_argument(
            '--out-storage', default='sqlite3',
            choices=get_registered_writers(),
            help='storage identifier to be used for the output bag')
        parser.add_argument(
            '-f', '--serialization-format', default='cdr',
            help='rmw serialization format in which the messages are saved, defaults to \'cdr\'')
        parser.add_argument('--progress', action='store_true',
                            help='show progress bar')
        self._filter.add_arguments(parser)
        self._filter.set_logger(self._logger)

    def main(self, *, args):  # noqa: D102
        for bag_file in args.bag_files:
            if not os.path.exists(bag_file):
                return print_error("bag file '{}' does not exist!".format(bag_file))

        uri = args.output or datetime.now().strftime('rosbag2_%Y_%m_%d-%H_%M_%S')

        if os.path.isdir(uri):
            return print_error("Output folder '{}' already exists.".format(uri))

        info = Info()
        metadatas = [info.read_metadata(
            f, args.in_storage or '') for f in args.bag_files]
        try:
            self._filter.set_args(metadatas, args)
        except argparse.ArgumentError as e:
            return print_error(str(e))

        progress = None
        if args.progress:
            progress = ProgressTracker()

        reader = FilteredReader(args.bag_files, self._filter, args.in_storage)

        if progress:
            for metadata in metadatas:
                progress.add_estimated_work(
                    metadata, self._filter.output_size_factor(metadata))

        writer = SequentialWriter()
        args_out_bag = vars(args).copy()
        args_out_bag['storage'] = args.out_storage
        args_out_bag['bag_path'] = uri
        out_storage_options, out_converter_options = get_reader_options(
            argparse.Namespace(**args_out_bag))
        writer.open(out_storage_options, out_converter_options)

        for topic in reader.get_all_topics_and_types():
            writer.create_topic(topic)

        for item in reader:
            if progress:
                prog_perc = progress.update(item[0])
                progress.print_update(prog_perc)
            writer.write(*item)

        if progress:
            progress.print_finish()
