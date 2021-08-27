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
import argparse
from rosbag2_py import BagMetadata, StorageFilter
from ros2cli.plugin_system import PluginException
from ros2cli.entry_points import load_entry_points
from ros2bag_tools.filter import FilterResult

logger = logging.getLogger(__name__)


class CompositeFilter:

    def __init__(self):
        self._filter_extensions = {}
        self._filters = []

    def add_arguments(self, parser):
        parser.add_argument(
            '-c', '--config', required=True,
            help='Path to configuration file of filters')
        self._filter_extensions = load_entry_points('ros2bag_tools.filter')

    def set_args(self, metadata, args):
        with open(args.config, 'r') as f:
            for line in f.readlines():
                line = line.strip()
                if not line:
                    # empty lines in config file are acceptable
                    continue
                if line.startswith('#'):
                    # allow comment lines
                    continue
                args_line = [word.strip() for word in line.split(' ')]
                filter_name = args_line[0]
                parser = argparse.ArgumentParser(filter_name)
                try:
                    filter = self._filter_extensions[filter_name]()
                except PluginException as e:  # noqa: F841
                    logger.warning(
                        f"Failed to instantiate ros2bag_tools.filter extension "
                        f"'{filter_name}': {e}")
                    raise argparse.ArgumentError(None, 'invalid filter')
                except Exception as e:  # noqa: F841
                    logger.error(
                        f"Failed to instantiate ros2bag_tools.filter extension "
                        f"'{filter_name}': {e}")
                    raise argparse.ArgumentError(None, 'invalid filter')
                filter.add_arguments(parser)
                filter_args = parser.parse_args(args_line[1:])
                filter.set_args(metadata, filter_args)
                self._filters.append(filter)
        assert(len(self._filters) > 0)

    def output_size_factor(self, metadata: BagMetadata):
        total = 1.0
        for filter in self._filters:
            total *= filter.output_size_factor(metadata)
        return total

    def get_storage_filter(self):
        """
        Combine storage filter of inner filters by union.
        """
        composite_storage_filter = None
        for filter in self._filters:
            storage_filter = filter.get_storage_filter()
            if storage_filter:
                if not composite_storage_filter:
                    composite_storage_filter = storage_filter
                else:
                    total_topics = set(composite_storage_filter.topics).union(
                        storage_filter.topics)
                    composite_storage_filter = StorageFilter(
                        topics=total_topics)
        return composite_storage_filter

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

    def filter_msg(self, msg):
        current_msgs = [msg]
        for f in self._filters:
            new_msgs = []
            for item in current_msgs:
                result = f.filter_msg(item)
                if result == FilterResult.DROP_MESSAGE:
                    return FilterResult.DROP_MESSAGE
                elif result == FilterResult.STOP_CURRENT_BAG:
                    return FilterResult.STOP_CURRENT_BAG
                elif isinstance(result, list):
                    new_msgs.extend(result)
                else:
                    new_msgs.append(result)
            current_msgs = new_msgs
        return current_msgs
