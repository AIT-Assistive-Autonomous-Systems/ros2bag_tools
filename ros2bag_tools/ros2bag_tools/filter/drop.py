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

from typing import Sequence
from ros2bag_tools.filter import FilterExtension
from ros2bag_tools.filter import FilterResult


class DropFilter(FilterExtension):
    """Drop X out every Y message of a topic."""

    def __init__(self):
        self._topics: Sequence[str] = []
        self._x = 0
        self._y = 0
        self._i = 0

    def add_arguments(self, parser):
        parser.add_argument(
            "-t",
            "--topics",
            nargs="+",
            help=(
                "Topics to drop messages from. Use 'all' to drop messages from all "
                "topics in the rosbag"
            ),
        )
        parser.add_argument(
            "-x", type=int, required=True, help="count of messages out of ytodrop"
        )
        parser.add_argument(
            "-y", type=int, required=True, help="module of the message counter"
        )

    def set_args(self, _metadata, args):
        self._topics = args.topics
        self._x = args.x
        self._y = args.y
        assert self._x <= self._y

    def _is_drop_topic(self, topic: str) -> bool:
        """Return a boolean indicating whether we're interested in filtering this topic."""
        if len(self._topics) == 1 and self._topics[0] == "all":
            return True

        return topic in self._topics

    def filter_msg(self, msg):
        (topic, _, _) = msg
        if self._is_drop_topic(topic):
            do_drop = False
            if self._i >= self._y:
                self._i = 0
            if self._i < self._x:
                do_drop = True
            self._i += 1
            if do_drop:
                return FilterResult.DROP_MESSAGE

        return msg
