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

import argparse
from ros2cli.plugin_system import PluginException
from ros2cli.entry_points import load_entry_points


def readargs(f):
    for line in f.readlines():
        line = line.strip()
        if not line:
            # empty lines in config file are acceptable
            continue
        if line.startswith('#'):
            # allow comment lines
            continue
        args = [word.strip() for word in line.split()]
        yield args


class ExtensionLoader:

    def __init__(self, extension_point, logger):
        self._extension_point = extension_point
        self._extensions = load_entry_points(extension_point)
        self._logger = logger

    def load(self, name, arg_arr):
        parser = argparse.ArgumentParser(name)
        try:
            extension = self._extensions[name]()
        except PluginException as e:  # noqa: F841
            self._logger.warning(
                f"Failed to instantiate {self._extension_point} extension "
                f"'{name}': {e}")
            raise argparse.ArgumentError(None, 'invalid extension')
        except Exception as e:  # noqa: F841
            self._logger.error(
                f"Failed to instantiate {self._extension_point} extension "
                f"'{name}': {e}")
            raise argparse.ArgumentError(None, 'invalid extension')
        extension.add_arguments(parser)
        return extension, parser.parse_args(arg_arr)
