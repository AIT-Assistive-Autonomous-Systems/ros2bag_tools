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

import tempfile
import os

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


@pytest.mark.rostest
@launch_testing.markers.keep_alive
def generate_test_description():
    return LaunchDescription([launch_testing.actions.ReadyToTest()])


class TestExport(unittest.TestCase):

    def test_export_stamp(self, launch_service, proc_info, proc_output):
        with tempfile.NamedTemporaryFile('w') as tmp:
            cmd = ['ros2', 'bag', 'export', '-i', 'test/diagnostics.bag',
                   '-t', '/diagnostics', 'stamp', '-o', tmp.name, '--header']
            cmd_action = ExecuteProcess(
                cmd=cmd,
                name='ros2bag_tools-cli',
                output='screen'
            )
            with launch_testing.tools.launch_process(
                launch_service, cmd_action, proc_info, proc_output
            ) as cmd:
                cmd.wait_for_shutdown(timeout=1)
                assert cmd.terminated
                assert cmd.exit_code == EXIT_OK

            with open(tmp.name, 'r') as f:
                assert ['00000000,0'] == [l.strip() for l in f.readlines()]

    def test_export_multiple(self, launch_service, proc_info, proc_output):
        with tempfile.TemporaryDirectory(suffix='images') as td \
             , tempfile.NamedTemporaryFile('w', suffix='filter') as tmp_f \
             , tempfile.NamedTemporaryFile('w', suffix='stamps') as tmp_stamps \
             , tempfile.NamedTemporaryFile('w', suffix='config') as tmp_c:

            tmp_f.write(' '.join(['cut', '--duration', '1.0', '\n']))
            tmp_f.flush()
            tmp_c.write(
                ' '.join(['/image', 'image', '--dir', td, '--name', 'im_%i.png']) + '\n' +
                ' '.join(['/image', 'stamp', '-o',
                            tmp_stamps.name, '--header'])
            )
            tmp_c.flush()

            cmd = ['ros2', 'bag', 'export', '-i', 'test/images.bag',
                    '-c', tmp_c.name, '-f', tmp_f.name]
            cmd_action = ExecuteProcess(
                cmd=cmd,
                name='ros2bag_tools-cli',
                output='screen'
            )
            with launch_testing.tools.launch_process(
                launch_service, cmd_action, proc_info, proc_output
            ) as cmd:
                cmd.wait_for_shutdown(timeout=1)
                assert cmd.terminated
                assert cmd.exit_code == EXIT_OK

            with open(tmp_stamps.name, 'r') as f:
                stamp_lines = [l.strip() for l in f.readlines()]
                assert '00000000,0' == stamp_lines[0]
                assert '00000001,1000000000' == stamp_lines[1]

            image_files = list(os.listdir(td))
            assert len(image_files) == 2
            assert 'im_00000000.png' in image_files
            assert 'im_00000001.png' in image_files
