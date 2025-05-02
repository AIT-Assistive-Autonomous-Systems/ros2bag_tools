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

from ros2bag_tools.filter import FilterExtension


class RenameFilter(FilterExtension):

    def __init__(self):
        self._rename_map = {}

    def add_arguments(self, parser):
        parser.add_argument(
            '-t', '--topic',
            action='append',
            dest='topics',
            required=True,
            help='Topic to rename. Can be specified multiple times with corresponding --name '
                 'arguments.'
        )
        parser.add_argument(
            '--name',
            action='append',
            dest='names',
            required=True,
            help='New name to set. Can be specified multiple times with corresponding --topic '
                 'arguments.'
        )

    def set_args(self, _metadata, args):
        if len(args.topics) != len(args.names):
            raise ValueError('Number of topics must match number of names')

        self._rename_map = dict(zip(args.topics, args.names))

    def filter_topic(self, topic_metadata):
        if topic_metadata.name in self._rename_map:
            topic_metadata.name = self._rename_map[topic_metadata.name]
        return topic_metadata

    def filter_msg(self, msg):
        (topic, data, t) = msg
        if topic in self._rename_map:
            return (self._rename_map[topic], data, t)
        return msg
