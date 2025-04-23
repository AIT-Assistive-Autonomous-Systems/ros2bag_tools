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

from ros2bag_tools.filter import FilterExtension
from ros2bag_tools.filter import FilterResult
from ros2bag_tools.filter import TopicRequest

from rosbag2_py import BagMetadata


class ExtractFilter(FilterExtension):

    def __init__(self):
        self._output_topics = set()

    def add_arguments(self, parser):
        parser.add_argument(
            '-t', '--topic', nargs='+',
            help='topics to extract into the output bag')
        parser.add_argument(
            '-i', '--invert', default=False, action='store_true',
            help='invert the filter, i.e. specified topics are NOT copied to output bag')

    def set_args(self, metadatas, args):
        if args.invert:
            for metadata in metadatas:
                for topic in metadata.topics_with_message_count:
                    topic_name = topic.topic_metadata.name
                    if topic_name not in args.topic:
                        self._output_topics.add(topic_name)
        else:
            self._output_topics = set(args.topic)

    def output_size_factor(self, metadata: BagMetadata):
        output_msg_count = 0
        total_msg_count = 0
        for topic in metadata.topics_with_message_count:
            if topic.topic_metadata.name in self._output_topics:
                output_msg_count += topic.message_count
            total_msg_count += topic.message_count
        # assume that messages are spread uniformly across filtered timespan
        # this might not always apply, but gives a good enough estimation for progress feedback
        return output_msg_count / total_msg_count

    def requested_topics(self):
        return [(TopicRequest.LIMIT, t) for t in self._output_topics]

    def filter_topic(self, topic_metadata):
        if topic_metadata.name not in self._output_topics:
            return None
        return topic_metadata

    def filter_msg(self, msg):
        (topic, _, _) = msg
        if topic not in self._output_topics:
            return FilterResult.DROP_MESSAGE
        return msg
