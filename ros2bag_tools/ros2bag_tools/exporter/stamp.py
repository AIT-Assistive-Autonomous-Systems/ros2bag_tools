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

from rclpy.time import Time

from ros2bag_tools.exporter import Exporter


class StampExporter(Exporter):
    """Timestamps by message index."""

    def __init__(self):
        self._args = None
        self._f = None
        self._i = 0

    @staticmethod
    def add_arguments(parser):
        parser.add_argument('--out', '-o', required=True, type=str,
                            help='Path to output file')
        parser.add_argument('--header', action='store_true',
                            help='Use header stamp rather than bag time')

    def open(self, args):  # noqa: A003
        self._args = args
        self._f = open(self._args.out, 'w')
        self._i = 0

    def write(self, topic, msg, t):
        if self._args.header:
            t = Time.from_msg(msg.header.stamp).nanoseconds
        self._f.write(f'{str(self._i).zfill(8)},{t}\n')
        self._i += 1

    def close(self):
        self._f.close()
