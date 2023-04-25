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

import matplotlib.pyplot as plt
from rosbag2_tools.bag_view import BagView
from rosbag2_tools.data_frame import read_data_frames
from ros2bag.api import add_standard_reader_args
from ros2bag.verb import VerbExtension, get_reader_options
from rosbag2_py import SequentialReader, StorageFilter


class PlotVerb(VerbExtension):
    """Display a plot of bag data."""

    def add_arguments(self, parser, cli_name):  # noqa: D102
        add_standard_reader_args(parser)
        parser.add_argument('-t', '--topic', nargs='+', required=True,
                            help='topics with field name to visualize')

    def main(self, *, args):
        in_storage_options, in_converter_options = get_reader_options(args)
        reader = SequentialReader()
        reader.open(in_storage_options, in_converter_options)

        topics_with_field = [tuple(t.split('.', 1)) for t in args.topic]

        filter = StorageFilter(topics=[twf[0] for twf in topics_with_field])
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
