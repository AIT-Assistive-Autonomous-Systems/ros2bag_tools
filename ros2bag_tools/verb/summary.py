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

from typing import NamedTuple, List
import os
import numpy as np
from rclpy.time import Time
from ros2bag.api import print_error
from ros2bag.verb import VerbExtension
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


class Summary(NamedTuple):
    count: int
    frame_id: str
    delays: List[float]


class SummaryVerb(VerbExtension):
    """Print a summary of the contents of a specific bag."""

    def add_arguments(self, parser, cli_name):  # noqa: D102
        parser.add_argument(
            'bag_file', help='bag file summarize')
        parser.add_argument(
            '-s', '--storage', default='sqlite3',
            help='storage identifier to be used for the input bag, defaults to "sqlite3"')
        parser.add_argument(
            '-f', '--serialization-format', default='',
            help='rmw serialization format in which the messages are read, defaults to the'
                 ' rmw currently in use')

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
        )

        reader = SequentialReader()
        in_storage_options = StorageOptions(uri=args.bag_file, storage_id=args.storage)
        in_converter_options = ConverterOptions(
            input_serialization_format=args.serialization_format,
            output_serialization_format=args.serialization_format)
        reader.open(in_storage_options, in_converter_options)

        type_name_to_type_map = {}
        topic_to_type_map = {}
        summaries = {}

        for topic_metadata in reader.get_all_topics_and_types():
            if topic_metadata.type not in type_name_to_type_map:
                try:
                    type_name_to_type_map[topic_metadata.type] = get_message(topic_metadata.type)
                except (AttributeError, ModuleNotFoundError, ValueError):
                    raise RuntimeError(f"Cannot load message type '{topic_metadata.type}'")
            topic_to_type_map[topic_metadata.name] = type_name_to_type_map[topic_metadata.type]
            summaries[topic_metadata.name] = {
                'count': 0,
                'frame_id': '',
                'delays': []
            }

        while reader.has_next():
            (topic, data, t) = reader.read_next()
            msg_type = topic_to_type_map[topic]
            msg = deserialize_message(data, msg_type)
            if hasattr(msg, 'header'):
                summaries[topic]['count'] += 1
                if summaries[topic]['frame_id']:
                    assert(summaries[topic]['frame_id'] == msg.header.frame_id)
                summaries[topic]['frame_id'] = msg.header.frame_id
                delay = t - Time.from_msg(msg.header.stamp).nanoseconds
                summaries[topic]['delays'].append(delay)

        for topic, summary in summaries.items():
            print(topic)
            frame_id = summary['frame_id']
            mean_delay = np.mean(np.array(summary['delays']))
            print(f'\tframe_id: {frame_id}')
            print(f'\tmean delay: {int(mean_delay / 1000 / 1000)}ms')
