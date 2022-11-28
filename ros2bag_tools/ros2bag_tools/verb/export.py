# Copyright 2022 AIT Austrian Institute of Technology GmbH
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

import operator
import logging
import rosbag2_py
from rosbag2_py import Info
from typing import NamedTuple
from ros2bag_tools.reader import TopicDeserializer
from ros2bag_tools.exporter import ExporterError
from ros2bag.api import print_error, check_path_exists
from ros2bag.verb import VerbExtension
from ros2bag_tools.filter import FilterExtension
from ros2bag_tools.filter.composite import CompositeFilter
from ros2bag_tools.verb import FilteredReader
from ros2cli.entry_points import load_entry_points
from ros2bag_tools.extension import ExtensionLoader, readargs


logger = logging.getLogger(__name__)


class CompositeFilterArgs(NamedTuple):
    config: str


class ExportVerb(VerbExtension):
    """Export bag data to other formats."""

    def __init__(self):
        VerbExtension.__init__(self)
        self._exporters = load_entry_points('ros2bag_tools.exporter')
        self._exporter_parsers = []
        self._filter = FilterExtension()

    def add_arguments(self, parser, cli_name):  # noqa: D102
        parser.add_argument('--in', '-i', required=True,
                            help='bag to export', dest='bag_path', type=check_path_exists)
        reader_choices = rosbag2_py.get_registered_readers()
        parser.add_argument(
            '-s', '--storage', default='', choices=reader_choices,
            help='Storage implementation of bag. '
            'By default attempts to detect automatically - use this argument to override.')

        target_arg_group = parser.add_mutually_exclusive_group(required=True)
        target_arg_group.add_argument(
            '-t', '--topic', type=str, help='topics to export')
        target_arg_group.add_argument(
            '-c', '--config', type=str, help='config of multiple exporters')
        parser.add_argument(
            '-f', '--filter', type=str, help='config of filters to apply before export')

        exporter_parsers = parser.add_subparsers(
            dest='exporter', help='choice of exporter')
        for exporter_name, exporter in self._exporters.items():
            exporter_parser = exporter_parsers.add_parser(exporter_name)
            exporter.add_arguments(exporter_parser)
            self._exporter_parsers.append(exporter_parser)

    def main(self, *, args):  # noqa: D102
        exporters = []
        if args.exporter:
            assert not args.config
            exporter = self._exporters[args.exporter]()
            exporter.open(args)
            exporters = [(args.topic, exporter)]
        else:
            assert args.config
            loader = ExtensionLoader('ros2bag_tools.exporter', logger)
            with open(args.config, 'r') as f:
                for argv in readargs(f):
                    assert(len(argv) >= 2)
                    topic = argv[0]
                    exporter_name = argv[1]
                    exporter_args = argv[2:]
                    exporter, exporter_args = loader.load(exporter_name, exporter_args)
                    exporter.open(exporter_args)
                    exporters.append((topic, exporter))

        assert(len(exporters) > 0)

        info = Info()
        metadatas = [info.read_metadata(args.bag_path, args.storage)]
        if args.filter:
            self._filter = CompositeFilter()
            self._filter.set_args(metadatas, CompositeFilterArgs(args.filter))

        exported_topics = list(map(operator.itemgetter(0), exporters))
        reader = FilteredReader(
            [args.bag_path], self._filter, args.storage, topics=exported_topics)

        serializer = TopicDeserializer()
        for topic_metadata in reader.get_all_topics_and_types():
            serializer.add_topic(topic_metadata)

        for topic, data, t in reader:
            msg = serializer.deserialize(topic, data)
            for exporter_topic, exporter in exporters:
                if exporter_topic == topic:
                    try:
                        exporter.write(topic, msg, t)
                    except ExporterError as e:
                        return print_error(f'exporter write failed: {str(e)}')
        for _, exporter in exporters:
            try:
                exporter.close()
            except ExporterError as e:
                return print_error(f'exporter flush failed: {str(e)}')
