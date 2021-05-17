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
import functools
from typing import Dict, List

import pandas as pd
from rclpy.time import Time
from rosidl_runtime_py.utilities import get_message

from rosbag2_tools.bag_view import BagView


# https://stackoverflow.com/questions/31174295/getattr-and-setattr-on-nested-subobjects-chained-properties
def _rgetattr(obj, attr, *args):
    """Recursively access attribute of object."""
    def _getattr(obj, attr):
        return getattr(obj, attr, *args)
    return functools.reduce(_getattr, [obj] + attr.split('.'))


def _field_type(msg_type, attr):
    """Get ROS type of message field."""
    fields = attr.split('.')
    parents = fields[:-1]
    last = fields[-1:][0]
    for parent in parents:
        parent_type_name = msg_type.get_fields_and_field_types()[parent]
        try:
            msg_type = get_message(parent_type_name)
        except (AttributeError, ModuleNotFoundError, ValueError):
            raise RuntimeError(f"Cannot load message type '{parent_type_name}'")
    return msg_type.get_fields_and_field_types()[last]


def read_data_frames(bag_view: BagView, field_dict: Dict[str, List[str]], auto_stamp=True):
    data = {}
    field_types = {}
    topics = dict(bag_view.topics())
    for (topic, fields) in field_dict.items():
        data[topic] = {}
        field_types[topic] = {}
        msg_type = topics[topic]
        fields_and_types = msg_type.get_fields_and_field_types()
        if ('header.stamp' not in fields and auto_stamp
                and 'header' in fields_and_types
                and fields_and_types['header'] == 'std_msgs/Header'):
            fields = fields.copy() # ensure not to mutate the input
            fields.append('header.stamp')
        for field in fields:
            field_types[topic][field] = _field_type(msg_type, field)
            data[topic][field] = []
    for (topic, msg, _) in bag_view:
        for field, field_type in field_types[topic].items():
            value = _rgetattr(msg, field)
            if field_type == 'builtin_interfaces/Time':
                ns = Time.from_msg(value).nanoseconds
                value = pd.to_datetime(ns, origin='unix', unit='ns')
            data[topic][field].append(value)
    return {topic: pd.DataFrame(data_dict) for topic, data_dict in data.items()}
