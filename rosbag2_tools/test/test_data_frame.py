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

import pandas as pd
import pytest

from rosbag2_py import StorageFilter
from rosbag2_tools.bag_view import BagView
from rosbag2_tools.data_frame import read_data_frames

pkg_prefix = Path(__file__).parents[1]


def test_data_frame_range(tmp_range_bag):
    dfs = read_data_frames(BagView(tmp_range_bag), {'/range': ['range']})
    assert ('/range' in dfs)
    df = dfs['/range']
    stamp0 = pd.Timestamp(90, unit='ns', tz='UTC')
    stamp1 = pd.Timestamp(190, unit='ns', tz='UTC')
    assert (df['header.stamp'][0] == stamp0)
    assert (df['header.stamp'][1] == stamp1)
    assert (df['range'][0] == 10.0)
    assert (df['range'][1] == 20.0)


def test_data_frame_multi_topic(tmp_multi_topic_bag):
    fields = {'/range': ['range'], '/diagnostics': ['key', 'value']}
    dfs = read_data_frames(BagView(tmp_multi_topic_bag), fields)

    assert ('/range' in dfs)
    assert ('/diagnostics' in dfs)

    df = dfs['/range']
    stamp0 = pd.Timestamp(90, unit='ns', tz='UTC')
    assert (df['header.stamp'][0] == stamp0)
    assert (df['range'][0] == 10.0)

    df = dfs['/diagnostics']
    assert (df['key'][0] == 'cpu')
    assert (df['value'][0] == 'warn')


def test_data_frame_multi_topic_fail(tmp_multi_topic_bag):
    fields = {'/range': ['range']}
    with pytest.raises(KeyError):
        read_data_frames(BagView(tmp_multi_topic_bag), fields)


def test_data_frame_multi_topic_but_just_use_one(tmp_multi_topic_bag):
    fields = {'/range': ['range']}
    storage_filter = StorageFilter(topics=list(fields.keys()))
    dfs = read_data_frames(BagView(tmp_multi_topic_bag, storage_filter), fields)

    assert ('/range' in dfs)

    with pytest.raises(KeyError):
        dfs['/diagnostics']
