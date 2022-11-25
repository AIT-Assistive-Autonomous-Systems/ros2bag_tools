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
from rosbag2_py import TopicMetadata


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

