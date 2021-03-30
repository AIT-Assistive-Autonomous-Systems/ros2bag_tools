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
from rosidl_runtime_py import set_message_fields
from rosidl_runtime_py.utilities import get_message
from rclpy.serialization import serialize_message, deserialize_message
from ros2bag_tools.filter import BagMessageFilter


class ReplaceFilter(BagMessageFilter):

    def __init__(self):
        self._args = None
        self._msg_module = None
        self._values_dictionary = {}

    def add_arguments(self, parser):
        parser.add_argument('-t', '--topic', required=True, help='topic to replace data for')
        parser.add_argument('-v', '--values', required=True, help='path to yaml data to load')

    def set_args(self, _in_files, _out_file, args):
        self._args = args
        with open(args.values, 'r') as f:
            self._values_dictionary = yaml.safe_load(f)
        if not isinstance(self._values_dictionary, dict):
            raise RuntimeError('The passed value needs to be a dictionary in YAML format')

    def filter_topic(self, topic_metadata):
        if topic_metadata.name == self._args.topic:
            topic_type = topic_metadata.type
            try:
                self._msg_module = get_message(topic_type)
            except (AttributeError, ModuleNotFoundError, ValueError):
                raise RuntimeError('The passed message type is invalid')
        return topic_metadata

    def filter_msg(self, msg):
        if not self._msg_module:
            tpc = self._args.topic
            raise RuntimeError(f"topic '{tpc}' does not exist in input bag files")

        (topic, data, t) = msg
        if topic == self._args.topic:
            msg = deserialize_message(data, self._msg_module)
            new_data = self._msg_module()
            try:
                set_message_fields(new_data, self._values_dictionary)
            except Exception as e:
                return 'Failed to populate field: {0}'.format(e)
            if (hasattr(msg, 'header') and hasattr(new_data, 'header')
                    and new_data.header.stamp.sec == 0
                    and new_data.header.stamp.nanosec == 0):
                new_data.header = msg.header
            return (topic, serialize_message(new_data), t)
        return msg
