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

import yaml
from rosbag2_py import TopicMetadata
from rosidl_runtime_py import set_message_fields
from rosidl_runtime_py.utilities import get_message
from rclpy.serialization import serialize_message, deserialize_message
from ros2bag_tools.filter import FilterExtension, TopicRequest


class AddFilter(FilterExtension):
    """Add new topic aligned with an existing topic."""

    def __init__(self):
        self._args = None
        self._aligned_msg_module = None
        self._msg_module = None
        self._values_dictionary = {}

    def add_arguments(self, parser):
        parser.add_argument('--type', required=True,
                            help='type of message to add')
        parser.add_argument('--align-to', required=True,
                            help='topic to align inserts with')
        parser.add_argument('-t', '--topic', required=True,
                            help='name of added topic')
        parser.add_argument('-v', '--values', required=True,
                            help='path to yaml data to load')

    def set_args(self, _metadata, args):
        self._args = args
        self._msg_module = get_message(args.type)
        with open(args.values, 'r') as f:
            self._values_dictionary = yaml.safe_load(f)
        if not isinstance(self._values_dictionary, dict):
            raise RuntimeError(
                '--values needs to point to a file containing a dictionary in YAML format')

    def requested_filters(self):
        return [(TopicRequest.REQUIRED, self._args.align_to)]

    def filter_topic(self, topic_metadata):
        if topic_metadata.name == self._args.align_to:
            try:
                self._aligned_msg_module = get_message(topic_metadata.type)
            except (AttributeError, ModuleNotFoundError, ValueError):
                raise RuntimeError('--align-to message type is invalid')
        new_metadata = TopicMetadata(
            name=self._args.topic, type=self._args.type, serialization_format='cdr')
        return [topic_metadata, new_metadata]

    def filter_msg(self, msg):
        (topic, data, t) = msg
        if topic == self._args.align_to:
            if not self._msg_module:
                raise RuntimeError(
                    f"Could not load message type of topic '{topic}'")

            aligned_msg = deserialize_message(data, self._aligned_msg_module)
            new_msg = self._msg_module()
            try:
                set_message_fields(new_msg, self._values_dictionary)
            except Exception as e:
                raise ValueError('failed to populate field: {0}'.format(e))
            if (hasattr(new_msg, 'header') and hasattr(aligned_msg, 'header')
                    and new_msg.header.stamp.sec == 0
                    and new_msg.header.stamp.nanosec == 0):
                new_msg.header = aligned_msg.header
            return [msg, (self._args.topic, serialize_message(new_msg), t)]
        return msg
