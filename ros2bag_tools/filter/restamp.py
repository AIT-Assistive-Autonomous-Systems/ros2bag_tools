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

from rclpy.time import Time
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from ros2bag_tools.filter import BagMessageFilter


class RestampFilter(BagMessageFilter):

    def __init__(self):
        self._args = None
        self._topic_type_map = {}
        self._type_map = {}

    def filter_topic(self, topic_metadata):
        topic_type = topic_metadata.type
        if topic_type not in self._type_map:
            try:
                self._type_map[topic_type] = get_message(topic_type)
            except (AttributeError, ModuleNotFoundError, ValueError):
                raise RuntimeError(f"Cannot load message type '{topic_type}'")
        self._topic_type_map[topic_metadata.name] = self._type_map[topic_type]
        return topic_metadata

    def filter_msg(self, serialized_msg):
        (topic, data, t) = serialized_msg
        msg = deserialize_message(data, self._topic_type_map[topic])
        if hasattr(msg, 'header'):
            t = Time.from_msg(msg.header.stamp).nanoseconds
        return (topic, data, t)
