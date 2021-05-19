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

import os
import cv2
from cv_bridge import CvBridge
from rosbag2_tools.bag_view import BagView
from ros2bag.api import print_error
from ros2bag.verb import VerbExtension


def get_screen_size():
    """
    Get size of currently active screen.
    """
    import tkinter as tk
    root = tk.Tk()
    screen_width = root.winfo_screenwidth()
    screen_height = root.winfo_screenheight()
    # just as mat.shape, return as (rows, columns)
    return (screen_height, screen_width)



class VideoVerb(VerbExtension):
    """Display or write a video of image data in a bag."""

    def add_arguments(self, parser, cli_name):  # noqa: D102
        parser.add_argument(
            'bag_file', help='bag file to read data from')
        parser.add_argument(
            '-s', '--storage', default='sqlite3',
            help='storage identifier to be used for the input bag, defaults to "sqlite3"')
        parser.add_argument(
            '-f', '--serialization-format', default='',
            help='rmw serialization format in which the messages are read, defaults to the'
                 ' rmw currently in use')
        parser.add_argument('-t', '--topic', required=True, type=str,
                            help='image topic to read')

    def main(self, *, args):  # noqa: D102
        if not os.path.exists(args.bag_file):
            return print_error("bag file '{}' does not exist!".format(args.bag_file))

        # NOTE(hidmic): in merged install workspaces on Windows, Python entrypoint lookups
        #               combined with constrained environments (as imposed by colcon test)
        #               may result in DLL loading failures when attempting to import a C
        #               extension. Therefore, do not import rosbag2_transport at the module
        #               level but on demand, right before first use.
        from rosbag2_py import (
            SequentialReader,
            StorageOptions,
            ConverterOptions,
            StorageFilter
        )
        screen_size = get_screen_size()

        reader = SequentialReader()
        in_storage_options = StorageOptions(
            uri=args.bag_file, storage_id=args.storage)
        in_converter_options = ConverterOptions(
            input_serialization_format=args.serialization_format,
            output_serialization_format=args.serialization_format)
        reader.open(in_storage_options, in_converter_options)

        filter = StorageFilter()
        filter.topics = [args.topic]

        # TODO ensure the input topic is an image topic
        bag_view = BagView(reader, filter)
        frame_rate = 30
        wait_ms = int(1000 / frame_rate)

        target_size = None

        window_name = args.topic
        for _, image, _ in bag_view:
            if not target_size:
                target_size = (image.height, image.width)
                while screen_size[0] < target_size[0]:
                    target_size = (int(target_size[0] / 2), int(target_size[1] / 2))
            image_bridge = CvBridge()
            cv_image = image_bridge.imgmsg_to_cv2(image)
            cv_image = cv2.resize(cv_image, (target_size[1], target_size[0]), interpolation=cv2.INTER_AREA)
            cv2.imshow(window_name, cv_image)
            key_pressed = cv2.waitKey(wait_ms) & 0xFF
            if key_pressed in [ord('q'), 27]:
                break
            if cv2.getWindowProperty(window_name, 0) < 0:
                # handle case where user closed window
                break
        
        cv2.destroyAllWindows()

