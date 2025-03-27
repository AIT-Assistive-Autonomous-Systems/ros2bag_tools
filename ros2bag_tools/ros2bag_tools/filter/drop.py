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

from typing import Dict, Sequence

from ros2bag.api import print_error
from ros2bag_tools.filter import FilterExtension, FilterResult


class DropFilter(FilterExtension):
    """Drop X out every Y message of a topic."""

    def __init__(self):
        super().__init__()
        self._topics: Sequence[str] = []
        self._ratio: float = 1.0
        self._msg_counters: Dict[str, Dict[str, int]] = {}

    def add_arguments(self, parser):
        parser.add_argument(
            '-t',
            '--topics',
            nargs='+',
            help=(
                'Topics to drop messages from. Use "all" to drop messages from all '
                'topics in the rosbag'
            ),
        )
        parser.add_argument(
            "-x",
            type=int,
            required=True,
            help="Count of messages out of y to drop. Must be > 0.",
        )
        parser.add_argument(
            "-y",
            type=int,
            required=True,
            help="Module of the message counter. Must be > 0 and => x.",
        )

    def set_args(self, _metadata, args):
        if not args.y > 0:
            print(print_error("y must be > 0"))
            exit(1)
        if not args.x > 0:
            print(print_error("x must be > 0"))
            exit(1)
        if not args.y >= args.x:
            print(print_error("y must be => x"))
            exit(1)

        self._topics = args.topics
        self._ratio = args.x / args.y
        self._sample_size = args.y

    def _is_drop_topic(self, topic: str) -> bool:
        """Return a boolean indicating whether we're interested in filtering this topic."""
        if len(self._topics) == 1 and self._topics[0] == 'all':
            return True

        return topic in self._topics

    def filter_msg(self, msg):
        (topic, _, _) = msg

        if self._is_drop_topic(topic):
            # initialize counters
            if topic not in self._msg_counters:
                self._msg_counters[topic] = {"total": 0, "dropped": 0}

            # reset counters if we have reached sample size
            if self._msg_counters[topic]["total"] == self._sample_size:
                self._msg_counters[topic] = {"total": 0, "dropped": 0}

            self._msg_counters[topic]["total"] += 1

            # calculate current dropped messages ratio
            ratio = (
                self._msg_counters[topic]["dropped"]
                / self._msg_counters[topic]["total"]
            )

            # drop message if we are currently below target ratio
            if ratio < self._ratio:
                self._msg_counters[topic]["dropped"] += 1
                return FilterResult.DROP_MESSAGE

        return msg
