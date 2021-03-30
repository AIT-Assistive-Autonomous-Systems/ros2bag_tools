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
from ros2bag_tools.filter import BagMessageFilter, FilterResult


class CompositeFilter(BagMessageFilter):

    def __init__(self, filters):
        self._available_filters = filters
        self._filters = []

    def add_arguments(self, parser):
        parser.add_argument(
            '-c', '--config', required=True,
            help='Path to configuration file of filters')

    def set_args(self, in_files, out_file, args):
        with open(args.config, 'r') as f:
            for line in f.readlines():
                args_line = [word.strip() for word in line.split(' ')]
                tool = args_line[0]
                parser = argparse.ArgumentParser(tool)
                filter = self._available_filters[tool]()
                filter.add_arguments(parser)
                filter_args = parser.parse_args(args_line[1:])
                filter.set_args(in_files, out_file, filter_args)
                self._filters.append(filter)
        assert(len(self._filters) > 0)

    def filter_topic(self, topic_metadata):
        current_tm = topic_metadata
        for f in self._filters:
            current_tm = f.filter_topic(current_tm)
            if not current_tm:
                return None
        return current_tm

    def filter_msg(self, msg):
        current_msg = msg
        for f in self._filters:
            current_msg = f.filter_msg(current_msg)
            if current_msg == FilterResult.DROP_MESSAGE:
                return None
        return current_msg
