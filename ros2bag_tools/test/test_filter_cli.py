# Copyright 2022 AIT Austrian Institute of Technology
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
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest

from launch import LaunchDescription
from launch.actions import ExecuteProcess
import launch_testing
import launch_testing.actions
import launch_testing.asserts
from launch_testing.asserts import EXIT_OK
import launch_testing.markers
import launch_testing.tools
import pytest

from .create_test_bags import create_day_time_bag
from .create_test_bags import create_diagnostics_bag
from .create_test_bags import create_string_bag

SHUTDOWN_TIMEOUT = 2


@pytest.mark.rostest
@launch_testing.markers.keep_alive
def generate_test_description():
    return LaunchDescription([launch_testing.actions.ReadyToTest()])


def read_all_messages_of_topic(bag_path, topic, msg_type):
    """Read all messages of given topic and type from a rosbag into a list."""
    from rclpy.serialization import deserialize_message
    from rosbag2_py import SequentialReader
    from rosbag2_tools import default_rosbag_options

    storage_options, converter_options = default_rosbag_options(bag_path)

    reader = SequentialReader()
    reader.open(storage_options, converter_options)

    result = []
    while reader.has_next():
        (tpc, data, _) = reader.read_next()
        if tpc == topic:
            result.append(deserialize_message(data, msg_type))
    return result


def count_messages(bag_path):
    """Count messages in a bag."""
    from rosbag2_py import SequentialReader
    from rosbag2_tools import default_rosbag_options

    storage_options, converter_options = default_rosbag_options(bag_path)
    reader = SequentialReader()
    reader.open(storage_options, converter_options)
    count = 0
    while reader.has_next():
        reader.read_next()
        count += 1
    return count


class TestFilterCli(unittest.TestCase):

    def setUp(self):
        self._tmp_string_bag = TemporaryDirectory()
        self.tmp_string_bag = str(
            Path(self._tmp_string_bag.name) / 'string.bag')
        create_string_bag(self.tmp_string_bag)

        self._tmp_diagnostics_bag = TemporaryDirectory()
        self.tmp_diagnostics_bag = str(
            Path(self._tmp_diagnostics_bag.name) / 'diagnostics.bag')
        create_diagnostics_bag(self.tmp_diagnostics_bag)

        self._tmp_day_time_bag = TemporaryDirectory()
        self.tmp_day_time_bag = str(
            Path(self._tmp_day_time_bag.name) / 'day_time.bag')
        create_day_time_bag(self.tmp_day_time_bag)

    def tearDown(self):
        self._tmp_string_bag.cleanup()
        self._tmp_diagnostics_bag.cleanup()
        self._tmp_day_time_bag.cleanup()

    def test_cut(self, launch_service, proc_info, proc_output):
        from example_interfaces.msg import String
        with TemporaryDirectory() as out_dir:
            outbag_path = str(Path(out_dir) / 'ros2bag_cut_test.bag')
            cmd = ['ros2', 'bag', 'cut', '--duration', '0.5', '-o', outbag_path]
            cmd.append(self.tmp_string_bag)
            bag_command_action = ExecuteProcess(
                cmd=cmd,
                name='ros2bag_tools-cli',
                output='screen'
            )
            with launch_testing.tools.launch_process(
                    launch_service, bag_command_action, proc_info, proc_output
            ) as bag_command:
                bag_command.wait_for_shutdown(timeout=SHUTDOWN_TIMEOUT)
                assert bag_command.terminated
                assert bag_command.exit_code == EXIT_OK

            msgs = read_all_messages_of_topic(outbag_path, '/data', String)
            assert len(msgs) == 1
            assert msgs[0].data == 'test_start'

    def test_cut_multiple(self, launch_service, proc_info, proc_output):
        """Test whether all messages of two bags are merged."""
        with TemporaryDirectory() as out_dir:
            outbag_path = str(Path(out_dir) / 'ros2bag_cut_multiple_test.bag')
            cmd = ['ros2', 'bag', 'cut', '--start', '0', '-o', outbag_path]
            cmd.append(self.tmp_string_bag)
            cmd.append(self.tmp_diagnostics_bag)
            bag_command_action = ExecuteProcess(
                cmd=cmd,
                name='ros2bag_tools-cli',
                output='screen'
            )
            with launch_testing.tools.launch_process(
                    launch_service, bag_command_action, proc_info, proc_output
            ) as bag_command:
                bag_command.wait_for_shutdown(timeout=SHUTDOWN_TIMEOUT)
                assert bag_command.terminated
                assert bag_command.exit_code == EXIT_OK

            msg_count = count_messages(outbag_path)
            assert msg_count == 3

    def test_cut_day_time(self, launch_service, proc_info, proc_output):
        with TemporaryDirectory() as out_dir:
            outbag_path = str(Path(out_dir) / 'ros2bag_cut_day_time_test.bag')
            cmd = ['ros2', 'bag', 'cut', '--start', '13:00', '--end', '14:00']
            cmd.append('-o')
            cmd.append(outbag_path)
            cmd.append(self.tmp_day_time_bag)
            bag_command_action = ExecuteProcess(
                cmd=cmd,
                name='ros2bag_tools-cli',
                output='screen'
            )
            with launch_testing.tools.launch_process(
                    launch_service, bag_command_action, proc_info, proc_output
            ) as bag_command:
                bag_command.wait_for_shutdown(timeout=SHUTDOWN_TIMEOUT)
                assert bag_command.terminated
                assert bag_command.exit_code == EXIT_OK

            msg_count = count_messages(outbag_path)
            assert msg_count == 2
