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

from rosidl_runtime_py.utilities import get_message
from rclpy.serialization import deserialize_message
from rosbag2_tools import default_rosbag_options
from ros2bag_tools.filter import FilterExtension, FilterResult, TopicRequest
from rosbag2_py import TopicMetadata, StorageFilter, SequentialReader

ReadOrder = None
ReadOrderSortBy = None

try:
    from rosbag2_py import (
        ReadOrder,
        ReadOrderSortBy
    )
except(ImportError):
    pass


class TopicDeserializer:

    def __init__(self):
        self._topic_type_map = {}
        self._type_map = {}

    def add_topic(self, topic_metadata: TopicMetadata):
        topic_type = topic_metadata.type
        if topic_type not in self._type_map:
            try:
                self._type_map[topic_type] = get_message(topic_type)
            except (AttributeError, ModuleNotFoundError, ValueError):
                raise RuntimeError(f"Cannot load message type '{topic_type}'")
        self._topic_type_map[topic_metadata.name] = self._type_map[topic_type]

    def deserialize(self, topic, data):
        return deserialize_message(data, self._topic_type_map[topic])


def topic_requests_to_storage_filter(topic_requests):
    if len(topic_requests) == 0:
        return None
    if not any((r == TopicRequest.LIMIT for (r, _) in topic_requests)):
        # if none of the requests limit the result, don't set a storage filter at all
        return None
    topics = {t for (_, t) in topic_requests}
    return StorageFilter(topics=list(topics))


class FilteredReader:

    def __init__(self, bag_paths, filter: FilterExtension, in_storage='', topics=None):
        assert(len(bag_paths) > 0)

        requested_topics = filter.requested_topics() + \
            [(TopicRequest.LIMIT, t) for t in (topics or [])]
        storage_filter = topic_requests_to_storage_filter(requested_topics)

        self._readers = []
        for bag_path in bag_paths:
            reader = SequentialReader()
            in_s_opts, in_c_opts = default_rosbag_options(bag_path, in_storage)
            reader.open(in_s_opts, in_c_opts)
            if ReadOrder:
                reader.set_read_order(
                    ReadOrder(ReadOrderSortBy.ReceivedTimestamp))
            if storage_filter:
                reader.set_filter(storage_filter)
            self._readers.append(reader)
        self._filter = filter
        self._storage_filter_topics = storage_filter.topics if storage_filter else []
        self._queue = []
        self._flushed = False

    def get_all_topics_and_types(self):
        for reader in self._readers:
            for topic_metadata in reader.get_all_topics_and_types():
                if self._storage_filter_topics and \
                        topic_metadata.name not in self._storage_filter_topics:
                    # if some of the topics will be filtered during reading, consider them
                    # excluded from the filtered reader
                    # this helps users of the filtered reader to avoid operations on topics and
                    # messages of topics that will not be part of the output, e.g. unnecessarily
                    # loading message types for unused topics
                    continue
                result = self._filter.filter_topic(topic_metadata)
                if result:
                    if isinstance(result, list):
                        for item in result:
                            yield item
                    else:
                        yield result

    def __iter__(self):
        return self

    def __next__(self):
        if len(self._queue) > 0:
            return self._queue.pop(0)
        while not self._flushed:
            if self._readers and not self._readers[0].has_next():
                self._readers.pop(0)
                continue  # check reader again
            if self._readers:
                msg = self._readers[0].read_next()
                result = self._filter.filter_msg(msg)
            else:
                result = self._filter.flush()
                self._flushed = True
            if result == FilterResult.STOP_CURRENT_BAG:
                raise StopIteration()
            elif result == FilterResult.DROP_MESSAGE:
                continue
            elif isinstance(result, list):
                if len(result) == 0:
                    continue
                self._queue = result[1:]
                return result[0]
            elif isinstance(result, tuple):
                return result
            else:
                raise ValueError(
                    "Filter returned invalid result: '{}'.".format(result))
        raise StopIteration()
