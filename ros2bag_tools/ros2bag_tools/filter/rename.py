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
        self._topic = None
        self._new_name = None

    def add_arguments(self, parser):
        parser.add_argument('-t', '--topic', help='topic to rename')
        parser.add_argument('--name', required=True, help='new name to set')

    def set_args(self, _metadata, args):
        self._topic = args.topic
        self._new_name = args.name

    def filter_topic(self, topic_metadata):
        if topic_metadata.name == self._topic:
            topic_metadata.name = self._new_name
        return topic_metadata

    def filter_msg(self, msg):
        (topic, data, t) = msg
        if topic == self._topic:
            return (self._new_name, data, t)
        return msg
