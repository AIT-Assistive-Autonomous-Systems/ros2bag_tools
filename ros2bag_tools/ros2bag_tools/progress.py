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

from rosbag2_py import BagMetadata


class ProgressTracker:

    def __init__(self):
        self._i = 0
        self._expected_topics = set()
        self._no_of_expected_messages = 0

    def add_estimated_work(self, metadata: BagMetadata, factor):
        n = sum(topic.message_count for topic in metadata.topics_with_message_count)
        self._no_of_expected_messages += int(n * factor)

    @property
    def n_processed(self):
        return self._i

    @property
    def n_expected(self):
        return self._no_of_expected_messages

    def update(self, topic) -> float:
        """
        Call when message of topic was processed.

        Return progress as number between 0.0 and 1.0.
        """
        if self._no_of_expected_messages <= 0:
            return 1.0
        if topic in self._expected_topics:
            self._i += 1
        return min(1, (self._i + 1) / self._no_of_expected_messages)

    def print_update(self, update, every=1):
        if self._i % every != 0:
            return
        values = (update, self.n_processed, self.n_expected)
        print('{0[0]:.2%} {0[1]}/{0[2]} ...'.format(values), end='\r')

    def print_finish(self):
        # print done and clear to end of line
        print('100% Done\033[K')
