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

from rosidl_runtime_py import message_to_yaml
from ros2bag.api import add_standard_reader_args
from ros2bag.verb import VerbExtension
from rosbag2_py import SequentialReader, StorageFilter
from ros2bag_tools.verb import get_reader_options
from rosbag2_tools.bag_view import BagView


class EchoVerb(VerbExtension):
    """Print message data as yaml."""

    def add_arguments(self, parser, cli_name):
        add_standard_reader_args(parser)
        parser.add_argument('topic_name', help='topic to echo')
        parser.add_argument('--no-arr', default=False, action='store_true')
        parser.add_argument('--no-pager', default=False, action='store_true')

    def main(self, *, args):
        storage_options, converter_options = get_reader_options(args)
        reader = SequentialReader()
        reader.open(storage_options, converter_options)
        for _, msg, _ in BagView(reader, StorageFilter(topics=[args.topic_name])):
            print(message_to_yaml(msg, no_arr=args.no_arr), end='---\n')
            if not args.no_pager:
                key = input('Press Enter to continue: ')
                if key == 'q':
                    break
