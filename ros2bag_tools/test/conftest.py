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
from typing import Tuple
from .create_test_bags import create_string_bag, create_diagnostics_bag, create_day_time_bag,\
    create_images_bag, create_synced_bag

import pytest


@pytest.fixture
def tmp_string_bag(tmp_path):
    bag_path = tmp_path / 'string.bag'
    create_string_bag(str(bag_path))
    return str(bag_path)


@pytest.fixture
def tmp_diagnostics_bag(tmp_path):
    bag_path = tmp_path / 'diagnostics.bag'
    create_diagnostics_bag(str(bag_path))
    return str(bag_path)


@pytest.fixture
def tmp_day_time_bag(tmp_path):
    bag_path = tmp_path / 'day_time.bag'
    create_day_time_bag(str(bag_path))
    return str(bag_path)


@pytest.fixture
def tmp_images_bag(tmp_path):
    bag_path = tmp_path / 'images.bag'
    create_images_bag(str(bag_path))
    return str(bag_path)


@pytest.fixture
def tmp_synced_bag(tmp_path):
    synced_bag = str(tmp_path / 'synced.bag')
    topics, synced_topics, synced_msgs = create_synced_bag(synced_bag)
    return synced_bag, synced_topics, topics, synced_msgs


@pytest.fixture(scope="session")
def dummy_synced_export_conf(tmp_path_factory: pytest.TempPathFactory) -> Tuple[str, str]:
    conf_path = tmp_path_factory.mktemp('conf')
    result_path = tmp_path_factory.mktemp('result')

    filter_conf = conf_path / 'sync_filter.conf'
    export_conf = conf_path / 'sync_export.conf'

    with filter_conf.open("w") as f:
        f.writelines([
            'sync -t /sync1 /sync2 /sync3'
        ])
    with export_conf.open("w") as f:
        f.writelines([
            f'/sync1 stamp -o {str(result_path)}/synced_stamps.txt'
        ])
    return str(filter_conf), str(export_conf), str(result_path)
