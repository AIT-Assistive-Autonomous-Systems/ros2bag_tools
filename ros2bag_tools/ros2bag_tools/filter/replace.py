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

from rclpy.serialization import deserialize_message
from rclpy.serialization import serialize_message

from ros2bag_tools.filter import FilterExtension

from rosbag2_tools.utils import set_message_fields

from rosidl_runtime_py.utilities import get_message

import yaml


class ReplaceFilter(FilterExtension):

    def __init__(self):
        self._args = None
        self._msg_module = None
        self._values_dictionary = {}

    def add_arguments(self, parser):
        parser.add_argument('-t', '--topic', required=True, help='topic to replace data for')
        parser.add_argument('-c', '--copy', action='store_true',
                            help='Copy original values (not just header).')
        parser.add_argument('--skip-missing', action='store_true',
                            help='Skip missing fields in the message.')
        parser.add_argument('--skip-value-errors', action='store_true',
                            help='Skip errors when setting values of fields.')
        parser.add_argument('-v', '--values', required=True, help='path to yaml data to load')

    def set_args(self, _metadata, args):
        self._args = args
        with open(args.values, 'r') as f:
            self._values_dictionary = yaml.safe_load(f)
        if not isinstance(self._values_dictionary, dict):
            raise RuntimeError('The passed value needs to be a dictionary in YAML format')

    def filter_topic(self, topic_metadata):
        if topic_metadata.name == self._args.topic:
            try:
                self._msg_module = get_message(topic_metadata.type)
            except (AttributeError, ModuleNotFoundError, ValueError):
                raise RuntimeError('The passed message type is invalid')
        return topic_metadata

    def filter_msg(self, msg):
        (topic, data, t) = msg
        if topic == self._args.topic:
            if not self._msg_module:
                raise RuntimeError(f"Could not load message type of topic '{topic}'")

            msg = deserialize_message(data, self._msg_module)
            new_data = msg if self._args.copy else self._msg_module()

            try:
                if not self._args.copy and hasattr(msg, 'header') and hasattr(new_data, 'header'):
                    new_data.header = msg.header
                set_message_fields(new_data,
                                   self._values_dictionary,
                                   strict=not self._args.skip_value_errors,
                                   strict_keys=not self._args.skip_missing)
            except Exception as e:
                raise RuntimeError('Failed to populate field: {0}'.format(e))
            return (topic, serialize_message(new_data), t)
        return msg
