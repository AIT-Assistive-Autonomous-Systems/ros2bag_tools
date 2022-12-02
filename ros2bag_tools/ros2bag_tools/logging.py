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

from rclpy.logging import get_logger as rclpy_get_logger
from rclpy.impl.rcutils_logger import RcutilsLogger
from logging import Logger

from typing import Union


class RclpyAdapter:

    def __init__(self, logger: Union[Logger, RcutilsLogger, str]):
        if isinstance(logger, str):
            logger = rclpy_get_logger(logger)
        self.logger = logger

    def getChild(self, name: str):
        if hasattr(self.logger, 'getChild'):
            return self.logger.getChild(name)
        else:
            return RclpyAdapter(logger=self.logger.get_child(name))

    def __getattr__(self, item):
        return getattr(self.logger, item)
