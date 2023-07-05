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

from pathlib import Path
import argparse
from ros2bag_tools.filter.composite import CompositeFilter


pkg_prefix = Path(__file__).parents[1]


def test_composite_filter():
    filter = CompositeFilter()

    parser = argparse.ArgumentParser('composite')
    filter.add_arguments(parser)
    args = parser.parse_args(['-c', str(pkg_prefix/'test'/'composite.config')])

    filter.set_args([], args)
    assert(len(filter.filter_msg(('/data', None, 0))) == 0)
