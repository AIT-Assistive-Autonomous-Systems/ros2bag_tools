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

from fractions import Fraction
from typing import Dict, Sequence

from ros2bag_tools.filter import FilterExtension, FilterResult


class DropFilter(FilterExtension):
    """Drop X out every Y message of a topic."""

    def __init__(self):
        super().__init__()
        self._topics: Sequence[str] = []
        self._ratio: Fraction = Fraction(1)
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
            '-x',
            type=int,
            required=False,
            help='Count of messages out of y to drop. Must be > 0. '
            'Requires y to be set and ratio to be unset.',
            default=-1
        )
        parser.add_argument(
            '-y',
            type=int,
            required=False,
            help='Module of the message counter. Must be > 0 and >= x. '
            'Requires x to be set and ratio to be unset.',
            default=-1
        )
        parser.add_argument(
            '-r', '--ratio',
            type=Fraction,
            required=False,
            help='Ratio of messages to drop. '
            'Can be specified as x/y or as number with decimal places. '
            'Must be > 0 and => 1. Overrules x and y arguments.',
            default=None
        )

    def set_args(self, _metadata, args):
        if args.ratio is None:
            if not args.y > 0:
                self._logger.error('y must be > 0')
                exit(1)
            if not args.x > 0:
                self._logger.error('x must be > 0')
                exit(1)
            if not args.y >= args.x:
                self._logger.error('y must be >= x')
                exit(1)
            self._ratio = Fraction(args.x, args.y)
        else:
            if args.ratio <= 0 or args.ratio > 1:
                self._logger.error('ratio must be > 0 and <= 1')
                exit(1)
            self._ratio = args.ratio

        self._topics = args.topics

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
                self._msg_counters[topic] = {'total': 0, 'dropped': 0}

            self._msg_counters[topic]['total'] += 1

            # calculate current dropped messages ratio
            ratio = Fraction(
                self._msg_counters[topic]['dropped'],
                self._msg_counters[topic]['total']
            )

            # drop message if we are currently below target ratio
            if ratio < self._ratio:
                self._msg_counters[topic]['dropped'] += 1
                return FilterResult.DROP_MESSAGE

        return msg
