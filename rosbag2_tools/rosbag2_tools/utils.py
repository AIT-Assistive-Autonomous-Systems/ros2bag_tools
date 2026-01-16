# Copyright 2025 AIT Austrian Institute of Technology GmbH
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
from collections.abc import Mapping
from typing import Any

from rosidl_parser.definition import NamespacedType

from rosidl_runtime_py.convert import get_message_slot_types


MsgMapping = Mapping[str, 'MsgMapping | Any']


def set_message_fields(msg: Any,
                       values: MsgMapping,
                       strict: bool = True,
                       strict_keys: bool = True) -> None:
    """
    Set message fields and raise error if fields can't be set.

    If strict is True, will raise an error if a field value can't be set and
    if strict_keys is True, will raise an error if a field is missing in the message.
    If a namespaced type is encountered and the value is a mapping, it will call itself
    recursively, otherwise it will try to set the value directly.
    """
    rosidl_fields_type = get_message_slot_types(msg)
    for key, value in values.items():
        try:
            rv = getattr(msg, key)
            rv_idl_type = rosidl_fields_type[key]
        except AttributeError:
            if strict_keys:
                raise RuntimeError(f"Missing field '{key}' in message {type(msg)}")
            continue
        if isinstance(value, Mapping):
            if not isinstance(rv_idl_type, NamespacedType):
                if strict:
                    raise RuntimeError(f"Field '{key}' is not a namespaced type (mapping)\
                                         in message {type(msg)}")
                continue
            set_message_fields(rv, value, strict, strict_keys)
        else:
            try:
                setattr(msg, key, value)
            except Exception as e:
                if strict:
                    raise RuntimeError(f"Failed to set field '{key}' in message \
                                        {type(msg)}: {e}")
