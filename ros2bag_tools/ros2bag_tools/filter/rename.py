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
import re


class RenameFilter(FilterExtension):

    def __init__(self):
        self._rename_map = {}
        self._glob_rules = []

    def add_arguments(self, parser):
        parser.add_argument(
            '-t', '--topic',
            action='append',
            dest='topics',
            required=True,
            help='Topic to rename. Supports exact match and * wildcard glob patterns. '
                 'Can be specified multiple times with corresponding --name arguments.'
        )
        parser.add_argument(
            '--name',
            action='append',
            dest='names',
            required=True,
            help='New name to set. Can be specified multiple times '
                 'with corresponding --topic arguments.'
        )

    def _glob_to_regex_with_star_captures(self, pat: str):
        groups = pat.count('*')

        # Escape regex special characters, then replace escaped '*' with '(.*)'
        # to capture the wildcard part
        rx = '^' + re.escape(pat).replace(r'\*', r'(.*)') + '$'
        return rx, groups

    def set_args(self, _metadata, args):
        if len(args.topics) != len(args.names):
            raise ValueError('Number of topics must match number of names')

        self._rename_map = {}
        glob_rules_unsorted = []

        for idx, (pat, new_name) in enumerate(zip(args.topics, args.names)):
            is_glob = '*' in pat

            if not is_glob:
                self._rename_map[pat] = new_name
                continue

            regex, n_groups = self._glob_to_regex_with_star_captures(pat)
            glob_rules_unsorted.append((pat, new_name, idx, re.compile(regex), n_groups))

        # Sort glob rules by pattern length (descending) and then by original index  
        # to maintain order for patterns of the same length
        glob_rules_unsorted.sort(key=lambda r: (-len(r[0]), r[2]))
        self._glob_rules = [
                {
                    "pattern": pat,
                    "repl": repl,
                    "rx": rx,
                    "n_groups": n_groups,
                }
                for (pat, repl, _, rx, n_groups) in glob_rules_unsorted
            ]


    def _apply_repl(self, repl: str, topic: str, m: re.Match) -> str:
        def sub(match):
            g = int(match.group(1))

            if g == 0:
                return topic

            if g <= m.re.groups:
                return m.group(g) if m.group(g) is not None else ""            
            return ""

        return re.sub(r"\\([0-9])", sub, repl)

    def _rename(self, topic: str) -> str:
        exact = self._rename_map.get(topic)
        if exact is not None:
            return exact

        for rule in self._glob_rules:
            m = rule["rx"].match(topic)

            if not m:
                continue

            return self._apply_repl(rule["repl"], topic, m)

        return topic

    def filter_topic(self, topic_metadata):
        topic_metadata.name = self._rename(topic_metadata.name)
        return topic_metadata

    def filter_msg(self, msg):
        topic, data, t = msg
        topic2 = self._rename(topic)

        if topic2 != topic:
            return (topic2, data, t)
        return msg
