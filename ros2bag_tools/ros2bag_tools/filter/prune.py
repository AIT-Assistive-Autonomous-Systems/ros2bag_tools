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

from rosbag2_py import TopicMetadata, StorageOptions, ConverterOptions
from ros2bag_tools.filter import FilterExtension


def get_rosbag_options(path, serialization_format='cdr'):
    storage_options = StorageOptions(uri=path, storage_id='sqlite3')
    converter_options = ConverterOptions(
        input_serialization_format=serialization_format,
        output_serialization_format=serialization_format)
    return storage_options, converter_options


class PruneFilter(FilterExtension):
    """Remove empty topics."""

    def __init__(self):
        self._empty_topics = set()

    def set_args(self, metadatas, _args):
        for metadata in metadatas:
            for topic in metadata.topics_with_message_count:
                n = topic.message_count
                topic_name = topic.topic_metadata.name
                if n == 0:
                    self._empty_topics.add(topic_name)
                else:
                    try:
                        self._empty_topics.remove(topic_name)
                    except KeyError:
                        pass

    def filter_topic(self, topic_metadata: TopicMetadata):
        topic = topic_metadata.name
        if topic in self._empty_topics:
            return None
        return topic_metadata
