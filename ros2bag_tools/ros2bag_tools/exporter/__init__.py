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


class ExporterError(Exception):
    """Error class to raise an exporter error."""

    pass


class Exporter:
    """Exporter base class for exporter extensions."""

    @staticmethod
    def add_arguments(parser):
        """Add exporter specific arguments."""
        pass

    def open(self, args):  # noqa: A003
        """Initialize exporter based on configuration."""
        pass

    def write(self, topic, msg, t):
        """Process and write/export a message."""
        pass

    def close(self):
        """Close the exporter and flush if needed."""
        pass
