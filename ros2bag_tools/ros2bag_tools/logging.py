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

# noqa: A005

from functools import lru_cache
import logging
from typing import Union

from rclpy.impl.rcutils_logger import RcutilsLogger
from rclpy.logging import get_logger as rclpy_get_logger


root: logging.Logger = None


def getLogger(logger: Union[str, logging.Logger, RcutilsLogger, None] = None) -> logging.Logger:
    result = None

    if isinstance(logger, str):
        result = RclpyAdapter(rclpy_get_logger(logger)) if root is None else root.getChild(logger)
    elif isinstance(logger, RcutilsLogger):
        result = RclpyAdapter(logger)
    elif logger is None:
        result = logging.getLogger() if root is None else root
    else:
        result = logger
    return result


@lru_cache(None)
def warn_once(logger: logging.Logger, msg: str):
    logger.warning(msg)


class RclpyAdapter:

    def __init__(self, logger: RcutilsLogger):
        self.logger = logger

    def getChild(self, name: str):
        if hasattr(self.logger, 'getChild'):
            return self.logger.getChild(name)
        else:
            return RclpyAdapter(logger=self.logger.get_child(name))

    def __getattr__(self, item):
        return getattr(self.logger, item)
