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

from datetime import timedelta
from typing import List, NamedTuple

import pytest

from ros2bag_tools.filter import FilterExtension
from ros2bag_tools.filter.cut import CutFilter
from ros2bag_tools.filter.extract import ExtractFilter
from ros2bag_tools.reader import FilteredReader

from rosbag2_py import Info


class CutArgs(NamedTuple):
    start: timedelta
    end: timedelta
    duration: timedelta
    transient_local_policy: str


class ExtractArgs(NamedTuple):
    topic: List[str]
    invert: bool


def test_reader_cut_filtered(tmp_string_bag):
    test_filter = CutFilter()
    info = Info()
    test_filter.set_args([info.read_metadata(tmp_string_bag, '')],
                         CutArgs(None, timedelta(microseconds=1), None, 'snap'))
    reader = FilteredReader([tmp_string_bag], test_filter)
    it = iter(reader)
    assert ('/data' == next(it)[0])
    with pytest.raises(StopIteration):
        next(it)


def test_reader_unfiltered(tmp_string_bag):
    reader = FilteredReader([tmp_string_bag], FilterExtension())
    it = iter(reader)
    assert ('/data' == next(it)[0])
    assert ('/data' == next(it)[0])
    with pytest.raises(StopIteration):
        next(it)


def test_reader_get_topics_union(tmp_string_bag, tmp_diagnostics_bag):
    reader = FilteredReader(
        [tmp_string_bag, tmp_diagnostics_bag], FilterExtension())
    it = reader.get_all_topics_and_types()
    assert ('/data' == next(it).name)
    assert ('/diagnostics' == next(it).name)
    with pytest.raises(StopIteration):
        next(it)


def test_reader_get_topics_filtered(tmp_string_bag, tmp_diagnostics_bag):
    bags = [tmp_string_bag, tmp_diagnostics_bag]
    test_filter = ExtractFilter()
    info = Info()
    test_filter.set_args([info.read_metadata(bag, '') for bag in bags],
                         ExtractArgs(['/diagnostics'], False))
    reader = FilteredReader(bags, test_filter)
    it = reader.get_all_topics_and_types()
    assert ('/diagnostics' == next(it).name)
    with pytest.raises(StopIteration):
        next(it)
