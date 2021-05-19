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
import matplotlib.pyplot as plt
from rosbag2_tools.bag_view import BagView
from rosbag2_tools.data_frame import read_data_frames
from ros2bag.api import print_error
from ros2bag.verb import VerbExtension


class PlotVerb(VerbExtension):
    """Display a plot of bag data."""

    def add_arguments(self, parser, cli_name):  # noqa: D102
        parser.add_argument(
            'bag_file', help='bag file to read data from')
        parser.add_argument(
            '-s', '--storage', default='sqlite3',
            help='storage identifier to be used for the input bag, defaults to "sqlite3"')
        parser.add_argument(
            '-f', '--serialization-format', default='',
            help='rmw serialization format in which the messages are read, defaults to the'
                 ' rmw currently in use')
        parser.add_argument('-t', '--topic', nargs='+', type=str,
                            help='topics with field name to visualize')

    def main(self, *, args):  # noqa: D102
        if not os.path.exists(args.bag_file):
            return print_error("bag file '{}' does not exist!".format(args.bag_file))

        # NOTE(hidmic): in merged install workspaces on Windows, Python entrypoint lookups
        #               combined with constrained environments (as imposed by colcon test)
        #               may result in DLL loading failures when attempting to import a C
        #               extension. Therefore, do not import rosbag2_transport at the module
        #               level but on demand, right before first use.
        from rosbag2_py import (
            SequentialReader,
            StorageOptions,
            ConverterOptions,
            StorageFilter
        )

        reader = SequentialReader()
        in_storage_options = StorageOptions(
            uri=args.bag_file, storage_id=args.storage)
        in_converter_options = ConverterOptions(
            input_serialization_format=args.serialization_format,
            output_serialization_format=args.serialization_format)
        reader.open(in_storage_options, in_converter_options)

        topics_with_field = [tuple(t.split('.', 1)) for t in args.topic]

        filter = StorageFilter()
        filter.topics = [twf[0] for twf in topics_with_field]

        fields_by_topic = {}
        for topic, field in topics_with_field:
            if topic not in fields_by_topic:
                fields_by_topic[topic] = []
            fields_by_topic[topic].append(field)

        bag_view = BagView(reader, filter)
        dfs = read_data_frames(bag_view, fields_by_topic)
        _, ax = plt.subplots()
        for topic, fields in fields_by_topic.items():
            for field in fields:
                dfs[topic].plot(x='header.stamp', y=field, ax=ax, label=f'{topic} {field}')
        ax.axhline(y=0, color='black', linestyle='--', linewidth=1.0)
        ax.grid(True)
        ax.set_xlabel('t (UTC header.stamp)')
        plt.show()
