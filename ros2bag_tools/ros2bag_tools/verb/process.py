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

from ros2bag_tools.filter.composite import CompositeFilter
from ros2bag_tools.filter.cut import CutFilter
from ros2bag_tools.filter.extract import ExtractFilter
from ros2bag_tools.filter.reframe import ReframeFilter
from ros2bag_tools.filter.rename import RenameFilter
from ros2bag_tools.filter.replace import ReplaceFilter
from ros2bag_tools.filter.restamp import RestampFilter
from ros2bag_tools.verb import BaseProcessVerb

AVAILABLE_FILTERS = {
    'cut': CutFilter,
    'extract': ExtractFilter,
    'reframe': ReframeFilter,
    'rename': RenameFilter,
    'replace': ReplaceFilter,
    'restamp': RestampFilter,
}


class ProcessVerb(BaseProcessVerb):
    """Run a set of filters on input bags and write to new bag."""

    def __init__(self):
        BaseProcessVerb.__init__(self, CompositeFilter(AVAILABLE_FILTERS))
