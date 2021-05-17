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

from rosbag2_py import StorageFilter
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


class BagView:

    def __init__(self, reader, storage_filter=StorageFilter()):
        self._reader = reader
        self._topic_type_map = {}
        self._storage_filter = storage_filter
        self._reader.set_filter(storage_filter)

        type_map = {}
        tpcs = self._storage_filter.topics
        for topic_metadata in self._reader.get_all_topics_and_types():
            if not tpcs or topic_metadata.name in tpcs:
                topic_type = topic_metadata.type
                if topic_type not in type_map:
                    try:
                        type_map[topic_metadata.type] = get_message(topic_type)
                    except (AttributeError, ModuleNotFoundError, ValueError):
                        raise RuntimeError(f"Cannot load message type '{topic_type}'")
                self._topic_type_map[topic_metadata.name] = type_map[topic_type]

    @property
    def filter(self):
        return self._storage_filter

    def topics(self):
        for (topic, msg_type) in self._topic_type_map.items():
            yield topic, msg_type

    def __iter__(self):
        return self

    def __next__(self):
        if not self._reader.has_next():
            raise StopIteration()
        (topic, data, t) = self._reader.read_next()
        msg = deserialize_message(data, self._topic_type_map[topic])
        return (topic, msg, t)
