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
from message_filters import ApproximateTimeSynchronizer, SimpleFilter
from rclpy.serialization import deserialize_message, serialize_message
from rosidl_runtime_py.utilities import get_message
from rosbag2_py import TopicMetadata, BagMetadata
from typing import Iterable, Sequence

from . import FilterExtension, BagMessageTuple

class BagWrappedMessage:
    def __init__(self, t, topic, msg):
        self._topic = topic
        self._t = t
        self._msg = msg

    @property
    def header(self):
        """Message field 'header'."""
        return self._msg.header

    @property
    def msg(self):
        return self._msg

    @property
    def t(self):
        return self._t

    @property
    def topic(self):
        return self._topic


class SyncSimpleFilter(SimpleFilter):
    """Simple filter to pass bag messages to"""

    def __init__(self, topic):
        super().__init__()
        self.topic = topic
        self.num_signaled = 0

    def getTopic(self):
        return self.topic

    def signalMessage(self, msg):
        super().signalMessage(msg)
        self.num_signaled += 1


def at_least_two(i):
    if int(i) < 2:
        raise RuntimeError("Must be at least 2")
    return int(i)

def positive(numeric):
    def check_value(arg):
        arg = numeric(arg)
        if arg < 0:
            raise RuntimeError("Must be >= 0")
    return check_value

class SyncFilter(FilterExtension):
    """
    Synchronize topics using ApproximateTimeSynchronizer and only
    output synchronized tuples of messages.
    """

    def __init__(self):
        self._sync_filters = {}
        self._topic_type_map = {}
        self._type_map = {}
        self._msgs = []
        self._num_syncs = 0

    def add_arguments(self, parser):
        parser.add_argument(
            '-t', '--topic', nargs='+',
            required=True,
            help='topics to synchronize')
        parser.add_argument(
            '--slop',
            type=positive(float),
            default=0.01,
            help='Allowed synchronization slope/error'
        )
        parser.add_argument(
            '-q', '--queue-size',
            type=at_least_two,
            default=3,
            help='Queue size of synchronizer queues'
        )

    def set_args(self, metadatas: Sequence[BagMetadata], args):
        sync_topics = [topic.topic_metadata.name
                       for meta in metadatas
                       for topic in meta.topics_with_message_count
                       if topic.topic_metadata.name in args.topic]
        if len(sync_topics) != len(args.topic):
            raise RuntimeError(
                f"Not all requested sync topics were found: {sync_topics}")
        self._sync_filters = {topic: SyncSimpleFilter(
            topic) for topic in sync_topics}
        self._synchronizer = ApproximateTimeSynchronizer(
            self._sync_filters.values(), args.queue_size, args.slop)
        self._synchronizer.registerCallback(self.sync_callback)

    def filter_topic(self, topic_metadata: TopicMetadata):
        topic_type = topic_metadata.type
        topic = topic_metadata.name
        if topic_type not in self._type_map and topic in self._sync_filters:
            try:
                message = get_message(topic_type)
                self._type_map[topic_type] = message
                fields = message.get_fields_and_field_types()
                if 'header' not in fields or fields['header'] != 'std_msgs/Header':
                    raise AttributeError(
                        f"Message {topic_type} has no header field.")
            except (AttributeError, ModuleNotFoundError, ValueError):
                raise RuntimeError(f"Cannot load message type '{topic_type}'")
        self._topic_type_map[topic] = self._type_map[topic_type]
        return topic_metadata

    def sync_callback(self, *msgs: Iterable[BagWrappedMessage]):
        self._msgs = [(msg.topic, serialize_message(msg.msg), msg.t)
                      for msg in msgs]
        self._num_syncs += 1

    def filter_msg(self, msg: BagMessageTuple):
        topic, data, t = msg

        if topic not in self._sync_filters:
            return msg

        self._sync_filters[topic].signalMessage(BagWrappedMessage(
            t, topic, deserialize_message(data, self._topic_type_map[topic])))

        result = self._msgs
        self._msgs = []
        # TODO(hofstaetterm): Produce result per filter to total num syncs.
        #
        # If no message drops occurred then num syncs and num signaled are identical. 
        return result
