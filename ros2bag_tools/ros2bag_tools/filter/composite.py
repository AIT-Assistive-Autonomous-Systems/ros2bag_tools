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

import logging
from rosbag2_py import BagMetadata
from ros2bag_tools.filter import FilterExtension, FilterResult
from ros2bag_tools.extension import ExtensionLoader, readargs
from operator import itemgetter
from itertools import chain

logger = logging.getLogger(__name__)


class CompositeFilter(FilterExtension):

    def __init__(self, logger=logger):
        super().__init__(logger=logger)
        self._loader = ExtensionLoader('ros2bag_tools.filter', logger)
        self._filters = []

    def add_arguments(self, parser):
        parser.add_argument(
            '-c', '--config', required=True,
            help='Path to configuration file of filters')

    def set_args(self, metadatas, args):
        with open(args.config, 'r') as f:
            for i, argv in enumerate(readargs(f)):
                assert(len(argv) >= 1)
                filter_name = argv[0]
                arg_arr = argv[1:]
                filter, filter_args = self._loader.load(filter_name, arg_arr)
                filter.set_logger(self._logger.getChild(f'{filter_name}({i})'))
                filter.set_args(metadatas, filter_args)
                self._filters.append(filter)
        assert(len(self._filters) > 0)

    def output_size_factor(self, metadata: BagMetadata):
        total = 1.0
        for filter in self._filters:
            total *= filter.output_size_factor(metadata)
        return total

    def requested_topics(self):
        return list(chain(*(f.requested_topics() for f in self._filters)))

    def filter_topic(self, topic_metadata):
        current_tm = [topic_metadata]
        for f in self._filters:
            new_tm = []
            for tm in current_tm:
                tm = f.filter_topic(tm)
                if isinstance(tm, list):
                    new_tm.extend(tm)
                elif tm:
                    new_tm.append(tm)
            current_tm = new_tm
        return current_tm

    def _filter_msg(self, msg, flush):
        current_msgs = [msg] if not flush else []
        for f in self._filters:
            new_msgs = []
            do_flush = flush
            while current_msgs or do_flush:
                if current_msgs:
                    result = f.filter_msg(current_msgs.pop(0))
                else:
                    result = f.flush()
                    do_flush = False
                if result == FilterResult.DROP_MESSAGE:
                    continue
                elif result == FilterResult.STOP_CURRENT_BAG:
                    return FilterResult.STOP_CURRENT_BAG
                elif isinstance(result, list):
                    new_msgs.extend(result)
                else:
                    new_msgs.append(result)
            current_msgs = sorted(new_msgs, key=itemgetter(2))
        return current_msgs

    def filter_msg(self, msg):
        return self._filter_msg(msg, False)

    def flush(self):
        return self._filter_msg(None, True)
