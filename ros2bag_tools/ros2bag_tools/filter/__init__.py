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
from enum import Enum


class FilterResult(Enum):
    DROP_MESSAGE = 1
    STOP_CURRENT_BAG = 2


class BagMessageFilter:

    def add_arguments(self, _parser):
        pass

    def set_args(self, _in_files, _out_file, _args):
        pass

    def get_storage_filter(self, _storage_filter):
        pass

    def filter_topic(self, topic):
        return topic

    def filter_msg(self, msg):
        return msg
