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

import os
from ros2bag_tools.exporter import ExporterError
from rosbag2_tools.bag_view import BagView
from ros2bag.api import print_error
from ros2bag.verb import VerbExtension
from rosbag2_py import StorageFilter
from ros2cli.entry_points import load_entry_points


class ExportVerb(VerbExtension):
    """Export bag data to other formats."""

    def __init__(self):
        VerbExtension.__init__(self)
        self._exporters = load_entry_points('ros2bag_tools.exporter')
        self._exporter_parsers = []

    def add_arguments(self, parser, cli_name):  # noqa: D102
        parser.add_argument('--in', '-i', required=True,
                            help='bag file to read data from', dest='bag_file')
        parser.add_argument('-t', '--topic', type=str, help='topics to export')

        exporter_parsers = parser.add_subparsers(dest='exporter', help='choice of exporter')
        for exporter_name, exporter in self._exporters.items():
            exporter_parser = exporter_parsers.add_parser(exporter_name)
            exporter.add_arguments(exporter_parser)
            self._exporter_parsers.append(exporter_parser)

    def main(self, *, args):  # noqa: D102
        if not args.topic:
            return print_error("topic to export is required")

        if not os.path.exists(args.bag_file):
            return print_error("bag file '{}' does not exist!".format(args.bag_file))

        filter = StorageFilter(topics=[args.topic])
        view = BagView(args.bag_file, filter)
        exporter = self._exporters[args.exporter]()
        try:
            exporter.process(args, view)
        except ExporterError as e:
            return print_error(f'export failed: {str(e)}')
