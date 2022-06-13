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

from rclpy.time import Time
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from ros2bag_tools.filter import FilterExtension
import re

class RestampFilter(FilterExtension):

    def __init__(self):
        self._args = None
        self._topic_type_map = {}
        self._type_map = {}

    def add_arguments(self, parser):
        parser.add_argument(
            '-u', '--offset-topic', nargs='+',
            help='topics to restamp with offset as regex string')
        parser.add_argument(
            '-c', '--offset', default=0, type=int,
            help='constant offset value in seconds applied to offset topics')

    def set_args(self, metadatas, args):
        self._offset_topics = set()
        for metadata in metadatas:
            for topic in metadata.topics_with_message_count:
                topic_name = topic.topic_metadata.name
                if any([re.match(r, topic_name) for r in args.offset_topic]):
                    self._offset_topics.add(topic_name)
        self._offset = args.offset * (10 ** 9)

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
        if topic in self._offset_topics:
            t = t + self._offset
        return (topic, data, t)
