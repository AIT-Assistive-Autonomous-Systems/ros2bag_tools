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
from rosbag2_tools.bag_view import BagView
from rosbag2_tools.data_frame import read_data_frames
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
import pandas as pd


def test_data_frame_range():
    reader = SequentialReader()
    storage_options = StorageOptions(uri='test/range.bag', storage_id='sqlite3')
    converter_options = ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr')
    reader.open(storage_options, converter_options)
    dfs = read_data_frames(BagView(reader), {'/range': ['range']})
    assert('/range' in dfs)
    df = dfs['/range']
    stamp0 = pd.Timestamp(90, unit='ns')
    stamp1 = pd.Timestamp(190, unit='ns')
    assert(df['header.stamp'][0] == stamp0)
    assert(df['header.stamp'][1] == stamp1)
    assert(df['range'][0] == 10.0)
    assert(df['range'][1] == 20.0)


def test_data_frame_multi_topic():
    reader = SequentialReader()
    storage_options = StorageOptions(uri='test/multi_topic.bag', storage_id='sqlite3')
    converter_options = ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr')
    reader.open(storage_options, converter_options)

    fields = {'/range': ['range'], '/diagnostics': ['key', 'value']}
    dfs = read_data_frames(BagView(reader), fields)

    assert('/range' in dfs)
    assert('/diagnostics' in dfs)

    df = dfs['/range']
    stamp0 = pd.Timestamp(90, unit='ns')
    assert(df['header.stamp'][0] == stamp0)
    assert(df['range'][0] == 10.0)

    df = dfs['/diagnostics']
    assert(df['key'][0] == 'cpu')
    assert(df['value'][0] == 'warn')
