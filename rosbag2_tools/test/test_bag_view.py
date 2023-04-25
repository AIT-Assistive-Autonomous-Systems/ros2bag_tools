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
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from sensor_msgs.msg import Range


def test_bag_view():
    reader = SequentialReader()
    storage_options = StorageOptions(uri='test/range.bag')
    converter_options = ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr')
    reader.open(storage_options, converter_options)
    items = []
    for item in BagView(reader):
        items.append(item)
    assert(len(items) == 2)
    msgs = [msg for (_, msg, _) in items]
    assert(isinstance(msgs[0], Range))
    assert(isinstance(msgs[1], Range))
    assert(msgs[0].header.stamp.sec == 0)
    assert(msgs[0].header.stamp.nanosec == 90)
    assert(msgs[1].header.stamp.sec == 0)
    assert(msgs[1].header.stamp.nanosec == 190)
    assert(msgs[0].range == 10.0)
    assert(msgs[1].range == 20.0)
    assert(items[0][0] == '/range')
    assert(items[1][0] == '/range')
