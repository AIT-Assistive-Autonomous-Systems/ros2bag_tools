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
from typing import Tuple, Sequence
from ros2bag_tools.filter import BagMessageTuple
from rosbag2_py import TopicMetadata
from .create_test_bags import create_synced_bag

import pytest


@pytest.fixture(scope="session")
def dummy_synced_bag(tmp_path_factory: pytest.TempPathFactory) -> Tuple[
        str, Sequence[str], Sequence[TopicMetadata], Sequence[BagMessageTuple]]:
    bag_path = tmp_path_factory.mktemp('bags')

    synced_bag = str(bag_path / 'synced_bag')

    topics, synced_topics, synced_msgs = create_synced_bag(synced_bag)

    return synced_bag, synced_topics, topics, synced_msgs
