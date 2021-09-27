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
from argparse import ArgumentError
from cv_bridge import CvBridge
from rosbag2_py import Info, SequentialReader, StorageOptions, ConverterOptions, StorageFilter
from ros2bag_tools.filter import FilterResult
from ros2bag_tools.filter.cut import CutFilter
from ros2bag_tools.verb import ProgressTracker
from rosbag2_tools.bag_view import BagView
from ros2bag.api import print_error
from ros2bag.verb import VerbExtension


IMAGE_MESSAGE_TYPE_NAME = 'sensor_msgs/msg/Image'
RESIZE_INTERPOLATION = cv2.INTER_AREA


def get_screen_size():
    """Get pixel size of the currently active screen."""
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
            image, (self._target_size[1], self._target_size[0]), interpolation=RESIZE_INTERPOLATION)
        cv2.imshow(self._window_name, image)
        key_pressed = cv2.waitKey(self._wait_ms) & 0xFF
        if key_pressed in [ord('q'), 27]:
            return False
        if cv2.getWindowProperty(self._window_name, 0) < 0:
            # handle case where user closed window
            return False
        return True


def estimate_fps(bag_path: str, storage_id: str, topic_name):
    info = Info()
    metadata = info.read_metadata(bag_path, storage_id)
    for entry in metadata.topics_with_message_count:
        if entry.topic_metadata.name == topic_name:
            return entry.message_count / metadata.duration.total_seconds()


def ensure_image(metadata, topic_name):
    for entry in metadata.topics_with_message_count:
        if entry.topic_metadata.name == topic_name:
            if entry.topic_metadata.type != IMAGE_MESSAGE_TYPE_NAME:
                raise ArgumentError(
                    None, f'topic type is not {IMAGE_MESSAGE_TYPE_NAME}')
            else:
                return
    raise ArgumentError(None, 'topic not in bag')


class VideoVerb(VerbExtension):
    """Display or write a video of image data in a bag."""

    def __init__(self):
        self._cut = CutFilter()

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
        parser.add_argument('-e', '--encoding', default='passthrough', type=str,
                            help='desired encoding for the output image')
        parser.add_argument('--image-resize', type=float,
                            help='image resize factor')
        self._cut.add_arguments(parser)

    def main(self, *, args):  # noqa: D102
        if not os.path.exists(args.bag_file):
            return print_error("bag file '{}' does not exist!".format(args.bag_file))

        info = Info()
        metadata = info.read_metadata(args.bag_file, args.storage)
        try:
            ensure_image(metadata, args.topic)
        except Exception as e:
            return print_error("invalid topic: {}".format(e))

        reader = SequentialReader()
        storage_options = StorageOptions(
            uri=args.bag_file, storage_id=args.storage)
        converter_options = ConverterOptions(
            input_serialization_format=args.serialization_format,
            output_serialization_format=args.serialization_format)
        reader.open(storage_options, converter_options)

        self._cut.set_args([metadata], args)

        filter = StorageFilter(topics=[args.topic])
        progress = ProgressTracker()
        if args.progress:
            progress.add_estimated_work(
                metadata, self._cut.output_size_factor(metadata))

        # TODO ensure the input topic is an image topic
        bag_view = BagView(reader, filter)
        if args.output:
            processor = VideoWriter(args.output, estimate_fps(
                args.bag_file, args.storage, args.topic))
        else:
            processor = VideoDisplay(args.topic)

        image_bridge = CvBridge()
        for tpc, image, t in bag_view:
            cut_result = self._cut.filter_msg((tpc, image, t))
            if cut_result == FilterResult.DROP_MESSAGE:
                continue
            if cut_result == FilterResult.STOP_CURRENT_BAG:
                break
            cv_image = image_bridge.imgmsg_to_cv2(image, args.encoding)
            width = int(cv_image.shape[1] * args.image_resize)
            height = int(cv_image.shape[0] * args.image_resize)
            dim = (width, height)
            if args.image_resize:
                cv_image = cv2.resize(cv_image, dim, interpolation=RESIZE_INTERPOLATION)
            processor.process(cv_image)
            if args.progress:
                progress.print_update(progress.update(args.topic), every=10)

        if args.progress:
            progress.print_finish()
