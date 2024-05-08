#!/usr/bin/python3
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
import pytest

from .create_test_bags import create_multi_topic_bag
from .create_test_bags import create_range_bag


@pytest.fixture
def tmp_range_bag(tmp_path):
    bag_path = tmp_path / 'range.bag'
    create_range_bag(str(bag_path))
    return str(bag_path)


@pytest.fixture
def tmp_multi_topic_bag(tmp_path):
    bag_path = tmp_path / 'multi_topic.bag'
    create_multi_topic_bag(str(bag_path))
    return str(bag_path)
