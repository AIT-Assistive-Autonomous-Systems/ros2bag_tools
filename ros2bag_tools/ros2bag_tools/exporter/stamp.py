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


class StampExporter:
    """Timestamps by message index."""

    @staticmethod
    def add_arguments(parser):
        parser.add_argument('--out', '-o', required=True, type=str,
                            help='Path to output file')
        parser.add_argument('--header', action='store_true',
                            help='Use header stamp rather than bag time')

    def process(self, args, msgs):
        idx = 0
        with open(args.out, 'w') as f:
            for _, img_msg, stamp in msgs:
                if args.header:
                    stamp = Time.from_msg(img_msg.header.stamp).nanoseconds
                f.write(f'{str(idx).zfill(8)},{stamp}\n')
                idx += 1
