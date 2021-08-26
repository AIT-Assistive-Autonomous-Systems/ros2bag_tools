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
from enum import Enum
from typing import Tuple, Union, List
import argparse
from rosbag2_py import TopicMetadata
from rclpy.exceptions import InvalidTopicNameException
from rclpy.validate_topic_name import validate_topic_name
from rclpy.serialization import deserialize_message, serialize_message
from rosidl_runtime_py.utilities import get_message
from ros2cli.plugin_system import PLUGIN_SYSTEM_VERSION, satisfies_version


class FilterResult(Enum):
    DROP_MESSAGE = 1
    STOP_CURRENT_BAG = 2


class FilterExtension:
    """
    The extension point for bag filter verb extensions.

    The following properties must be defined:
    * `NAME`

    The following methods can be defined:
    * `add_arguments`
    * `set_args`
    * `get_storage_filter`
    * `filter_topic`
    * `filter_msg`
    """

    EXTENSION_POINT_VERSION = '0.1'

    def __init__(self):
        super(FilterExtension, self).__init__()
        satisfies_version(PLUGIN_SYSTEM_VERSION, '^0.1')

    def add_arguments(self, _args):
        pass

    def set_args(self, _in_files, _out_file, _args):
        pass

    def get_storage_filter(self):
        return None

    def filter_topic(self, topic) -> Union[TopicMetadata, List[TopicMetadata]]:
        return topic

    def filter_msg(self, msg) -> Union[FilterResult, Tuple[str, bytes, int], List]:
        return msg


def TopicNameArg(value):
    try:
        validate_topic_name(value)
    except InvalidTopicNameException as e:
        raise argparse.ArgumentTypeError(str(e))
    return value


class TypeAwareTopicFilter(FilterExtension):

    def __init__(self):
        super(TypeAwareTopicFilter, self).__init__()
        self._message_type = None
        self._topic = None

    def add_arguments(self, parser):
        parser.add_argument(
            '-t', '--topic', help='target topic name', required=True, type=TopicNameArg)

    @property
    def topic(self):
        return self._topic

    def set_args(self, _in_files, _out_file, args):
        self._topic = args.topic

    def filter_topic(self, topic_metadata):
        if topic_metadata.name == self._topic:
            topic_type_name = topic_metadata.type
            try:
                self._message_type = get_message(topic_type_name)
            except (AttributeError, ModuleNotFoundError, ValueError):
                raise RuntimeError(
                    f"failed to load message type '{topic_type_name}'")
        return topic_metadata

    def filter_msg(self, msg):
        (topic, data, t) = msg
        if topic == self._topic:
            msg = deserialize_message(data, self._message_type)
            result = self.filter_typed_msg((topic, msg, t))
            if isinstance(result, FilterResult):
                return result
            (topic, msg, t) = result
            return (topic, serialize_message(msg), t)
        return msg

    def filter_typed_msg(self, _msg):
        raise NotImplementedError()
