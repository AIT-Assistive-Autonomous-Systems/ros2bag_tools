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

from ros2bag_tools.filter import BagMessageFilter
from rclpy.serialization import deserialize_message, serialize_message
from rosidl_runtime_py.utilities import get_message


class ReframeFilter(BagMessageFilter):

    def __init__(self):
        self._topic = None
        self._frame_id = None
        self._message_type = None

    def add_arguments(self, parser):
        parser.add_argument(
            '-t', '--topic',
            help='topic to set frame_id for; its messages must have a header')
        parser.add_argument(
            '--frame', required=True,
            help='frame_id to set')

    def set_args(self, _in_files, _out_file, args):
        self._topic = args.topic
        self._frame_id = args.frame

    def filter_topic(self, topic_metadata):
        if topic_metadata.name == self._topic:
            topic_type_name = topic_metadata.type
            try:
                self._message_type = get_message(topic_type_name)
            except (AttributeError, ModuleNotFoundError, ValueError):
                raise RuntimeError(f"Cannot load message type '{topic_type_name}'")
        return topic_metadata

    def filter_msg(self, msg):
        (topic, data, t) = msg
        if topic == self._topic:
            msg = deserialize_message(data, self._message_type)
            assert hasattr(msg, 'header')
            msg.header.frame_id = self._frame_id
            return (topic, serialize_message(msg), t)
        return msg
