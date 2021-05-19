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
from ros2bag_tools.verb import ProgressTracker
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


class VideoWriter:

    def __init__(self, file_path, fps):
        self._writer = cv2.VideoWriter()
        self._file_path = file_path
        self._fps = fps
        self._initialized = False

    def __del__(self):
        self._writer.release()

    def process(self, image):
        if not self._initialized:
            fourcc = cv2.VideoWriter_fourcc('m', 'p', '4', 'v')
            video_size = (image.shape[1], image.shape[0])
            has_color = len(image.shape) >= 3 and image.shape[2] != 1
            self._writer.open(self._file_path, fourcc,
                              self._fps, video_size, has_color)
            assert(self._writer.isOpened())
            self._initialized = True
        self._writer.write(image)


class VideoDisplay:

    def __init__(self, _window_name):
        frame_rate = 30
        self._wait_ms = int(1000 / frame_rate)
        self._target_size = None
        self._window_name = _window_name

    def __del__(self):
        cv2.destroyAllWindows()

    def process(self, image):
        if not self._target_size:
            screen_size = get_screen_size()
            self._target_size = image.shape
            while screen_size[0] < self._target_size[0]:
                self._target_size = (
                    int(self._target_size[0] / 2), int(self._target_size[1] / 2))

        image = cv2.resize(
            image, (self._target_size[1], self._target_size[0]), interpolation=cv2.INTER_AREA)
        cv2.imshow(self._window_name, image)
        key_pressed = cv2.waitKey(self._wait_ms) & 0xFF
        if key_pressed in [ord('q'), 27]:
            return False
        if cv2.getWindowProperty(self._window_name, 0) < 0:
            # handle case where user closed window
            return False
        return True


def get_fps(reader, topic_name):
    metadata = reader.get_metadata()
    for entry in metadata.topics_with_message_count:
        if entry.topic_metadata.name == topic_name:
            return entry.message_count / metadata.duration.total_seconds()


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
        parser.add_argument(
            '--progress', default=False, action='store_true',
            help='display reader progress in terminal')
        parser.add_argument('-t', '--topic', required=True, type=str,
                            help='image topic to read')
        parser.add_argument('-o', '--output', type=str,
                            help='file path to video to write')

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

        reader = SequentialReader()
        in_storage_options = StorageOptions(
            uri=args.bag_file, storage_id=args.storage)
        in_converter_options = ConverterOptions(
            input_serialization_format=args.serialization_format,
            output_serialization_format=args.serialization_format)
        reader.open(in_storage_options, in_converter_options)

        filter = StorageFilter()
        filter.topics = [args.topic]

        progress = ProgressTracker()
        if args.progress:
            progress.add_estimated_work(reader, filter)

        # TODO ensure the input topic is an image topic
        bag_view = BagView(reader, filter)
        if args.output:
            processor = VideoWriter(args.output, get_fps(reader, args.topic))
        else:
            processor = VideoDisplay(args.topic)

        image_bridge = CvBridge()
        for _, image, _ in bag_view:
            cv_image = image_bridge.imgmsg_to_cv2(image)
            processor.process(cv_image)
            if args.progress:
                progress.print_update(progress.update(args.topic), every=10)

        if args.progress:
            progress.print_finish()
