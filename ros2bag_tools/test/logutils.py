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
from contextlib import contextmanager
import logging
from typing import Generator

from pytest import LogCaptureFixture

import ros2bag_tools.logging as r2bt_logging


@contextmanager
def capture_at_level(caplog: LogCaptureFixture, level: int,
                     logger: str) -> Generator[None, None, None]:
    original_root = r2bt_logging.root

    r2bt_logging.root = logging.getLogger()
    logger_impl = r2bt_logging.getLogger(logger)
    logger_impl.addHandler(caplog.handler)

    with caplog.at_level(level, logger):
        try:
            yield
        finally:
            logger_impl.removeHandler(caplog.handler)
            r2bt_logging.root = original_root
