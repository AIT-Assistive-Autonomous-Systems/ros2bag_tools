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
import tempfile

from launch import LaunchDescription
from launch.actions import ExecuteProcess

import launch_testing
import launch_testing.actions
import launch_testing.asserts
import launch_testing.markers
import launch_testing.tools
from launch_testing.asserts import EXIT_OK

import pytest
import unittest


SHUTDOWN_TIMEOUT = 2


@pytest.mark.rostest
@launch_testing.markers.keep_alive
def generate_test_description():
    return LaunchDescription([launch_testing.actions.ReadyToTest()])


def read_all_messages_of_topic(bag_path, topic, type):
    """Read all messages of given topic and type from a rosbag into a list."""
    from rosbag2_py import SequentialReader
    from rclpy.serialization import deserialize_message
    from rosbag2_tools import default_rosbag_options

    storage_options, converter_options = default_rosbag_options(bag_path)

    reader = SequentialReader()
    reader.open(storage_options, converter_options)

    result = []
    while reader.has_next():
        (tpc, data, _) = reader.read_next()
        if tpc == topic:
            result.append(deserialize_message(data, type))
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

    def test_cut(self, launch_service, proc_info, proc_output):
        from example_interfaces.msg import String
        inbag_path = 'test/test.bag'

        with tempfile.TemporaryDirectory() as temp_dir:
            outbag_path = Path(temp_dir) / 'ros2bag_cut_test.bag'
            cmd = ['ros2', 'bag', 'cut', '--duration', '0.5', '-o', str(outbag_path)]
            cmd.append(inbag_path)
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

            msgs = read_all_messages_of_topic(str(outbag_path), '/data', String)
            assert len(msgs) == 1
            assert msgs[0].data == 'test_start'

    def test_cut_multiple(self, launch_service, proc_info, proc_output):
        """Test whether all messages of two bags are merged."""
        with tempfile.TemporaryDirectory() as temp_dir:
            outbag_path = Path(temp_dir) / 'ros2bag_cut_multiple_test.bag'
            cmd = ['ros2', 'bag', 'cut', '--start', '0', '-o', str(outbag_path)]
            cmd.append('test/test.bag')
            cmd.append('test/diagnostics.bag')
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

            msg_count = count_messages(str(outbag_path))
            assert msg_count == 3

    def test_cut_day_time(self, launch_service, proc_info, proc_output):
        inbag_path = 'test/day_time.bag'

        with tempfile.TemporaryDirectory() as temp_dir:
            outbag_path = Path(temp_dir) / 'ros2bag_cut_day_time_test.bag'
            cmd = ['ros2', 'bag', 'cut', '--start', '13:00', '--end', '14:00']
            cmd.append('-o')
            cmd.append(str(outbag_path))
            cmd.append(inbag_path)
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

            msg_count = count_messages(str(outbag_path))
            assert msg_count == 2
