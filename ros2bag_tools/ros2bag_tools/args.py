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

import rosbag2_py
from ros2bag.api import check_path_exists


# copied from ros2bag.api until 0.15.3 is released to humble
def add_standard_reader_args(parser) -> None:
    parser.add_argument(
        'bag_path', type=check_path_exists, help='Bag to open')
    reader_choices = rosbag2_py.get_registered_readers()
    parser.add_argument(
        '-s', '--storage', default='', choices=reader_choices,
        help='Storage implementation of bag. '
             'By default attempts to detect automatically - use this argument to override.')
