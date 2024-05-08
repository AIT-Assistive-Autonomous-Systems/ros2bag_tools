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

import pytest

from .create_test_bags import create_day_time_bag
from .create_test_bags import create_diagnostics_bag
from .create_test_bags import create_images_bag
from .create_test_bags import create_string_bag
from .create_test_bags import create_synced_bag


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
    bag_path = str(tmp_path / 'synced.bag')
    create_synced_bag(bag_path)
    return bag_path


@pytest.fixture(scope='session')
def synced_export_conf(tmp_path_factory: pytest.TempPathFactory) -> Tuple[str, str]:
    conf_path = tmp_path_factory.mktemp('conf')
    result_path = tmp_path_factory.mktemp('result')

    filter_conf = conf_path / 'sync_filter.conf'
    export_conf = conf_path / 'sync_export.conf'

    with filter_conf.open('w') as f:
        f.writelines([
            'sync -t /sync0 /sync1'
        ])
    with export_conf.open('w') as f:
        f.writelines([
            f'/sync0 stamp -o {str(result_path)}/synced_stamps.txt'
        ])
    return str(filter_conf), str(export_conf)
