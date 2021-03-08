# Copyright 2020 AIT Austrian Institute of Technology GmbH
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

from ros2bag_tools.filter import BagMessageFilter


class ExtractFilter(BagMessageFilter):

    def __init__(self):
        self._args = None

    def add_arguments(self, parser):
        parser.add_argument(
            '-t', '--topic', nargs='+',
            help='topics to extract into the output bag')
        parser.add_argument(
            '-i', '--invert', default=False, action='store_true',
            help='invert the filter, i.e. specified topics are NOT copied to output bag')

    def set_args(self, _in_file, _out_file, args):
        self._args = args

    def filter_topic(self, topic_metadata):
        topic = topic_metadata.name
        if self._args.invert:
            if topic in self._args.topic:
                return None
        elif topic not in self._args.topic:
            return None
        return topic_metadata

    def filter_msg(self, msg):
        (topic, _, _) = msg
        if self._args.invert:
            if topic in self._args.topic:
                return None
        elif topic not in self._args.topic:
            return None
        return msg
