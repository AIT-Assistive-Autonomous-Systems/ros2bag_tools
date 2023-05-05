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

import cv2
from argparse import ArgumentError
from cv_bridge import CvBridge, cvtColorForDisplay
from rosbag2_py import Info, SequentialReader, StorageFilter
from ros2bag_tools.exporter.image import CompressedImageMsgWriter
from ros2bag_tools.filter import FilterResult
from ros2bag_tools.filter.cut import CutFilter
from ros2bag_tools.progress import ProgressTracker
from rosbag2_tools.bag_view import BagView
from ros2bag_tools.verb import get_reader_options
from ros2bag.api import print_error, add_standard_reader_args
from ros2bag.verb import VerbExtension


IMAGE_MESSAGE_TYPE_NAME = 'sensor_msgs/msg/Image'
COMPRESSED_IMAGE_MESSAGE_TYPE_NAME = 'sensor_msgs/msg/CompressedImage'
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
        self._fourcc = cv2.VideoWriter_fourcc('m', 'p', '4', 'v')
        if '.webm' == self._file_path[-5:]:
            self._fourcc = cv2.VideoWriter_fourcc('v', 'p', '0', '9')
        self._fps = fps
        self._initialized = False

    def __del__(self):
        self._writer.release()

    def process(self, image):
        if not self._initialized:
            video_size = (image.shape[1], image.shape[0])
            has_color = len(image.shape) >= 3 and image.shape[2] != 1
            self._writer.open(self._file_path, self._fourcc,
                              self._fps, video_size, has_color)
            assert(self._writer.isOpened())
            self._initialized = True
        self._writer.write(image)


class VideoDisplay:

    def __init__(self, _window_name, fps):
        self._wait_ms = int(1000 / fps)
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
            image, (self._target_size[1], self._target_size[0]),
            interpolation=RESIZE_INTERPOLATION)
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
    """
    Raise error if topic is not an image or compressed image.

    If topic is an image topic, returns whether or not it is compressed
    """
    for entry in metadata.topics_with_message_count:
        if entry.topic_metadata.name == topic_name:
            if entry.topic_metadata.type != IMAGE_MESSAGE_TYPE_NAME:
                if entry.topic_metadata.type != COMPRESSED_IMAGE_MESSAGE_TYPE_NAME:
                    raise ArgumentError(
                        None, f'topic type is not {IMAGE_MESSAGE_TYPE_NAME} or \
                            {COMPRESSED_IMAGE_MESSAGE_TYPE_NAME}')
                else:
                    return True
            else:
                return False
    raise ArgumentError(None, 'topic not in bag')


class VideoVerb(VerbExtension):
    """Display or write a video of image data in a bag."""

    def __init__(self):
        self._cut = CutFilter()

    def add_arguments(self, parser, cli_name):  # noqa: D102
        add_standard_reader_args(parser)
        parser.add_argument(
            '--progress', default=False, action='store_true',
            help='display reader progress in terminal')
        parser.add_argument('-t', '--topic', required=True, type=str,
                            help='image topic to read')
        parser.add_argument('-o', '--output', type=str,
                            help='file path to video to write')
        parser.add_argument('--image-resize', type=float,
                            help='image resize factor')
        parser.add_argument('--fps', type=int,
                            help='video fps, default: estimated from message count')

        display_group = parser.add_argument_group(
            'display options',
            'Options to control the color conversions performed on the input images')
        display_group.add_argument('-e', '--encoding', default='', type=str,
                                   help='desired encoding for the output image')
        display_group.add_argument('--do-dynamic-scaling', action='store_true', default=False,
                                   help='perform dynamic scaling when converting for display')
        display_group.add_argument('--min-image-value', type=float,
                                   help='minimum value to map for display')
        display_group.add_argument('--max-image-value', type=float,
                                   help='maximum value to map for display')
        # requires https://github.com/ros-perception/vision_opencv/pull/452 to be merged
        # display_group.add_argument('--colormap', type=int, default=-1,
        #                            help='colormap to use for color conversion')

        self._cut.add_arguments(parser)

    def main(self, *, args):  # noqa: D102
        info = Info()
        metadata = info.read_metadata(args.bag_path, args.storage)
        is_compressed = False
        try:
            is_compressed = ensure_image(metadata, args.topic)
        except Exception as e:
            return print_error("invalid topic: {}".format(e))

        reader = SequentialReader()
        storage_options, converter_options = get_reader_options(args)
        reader.open(storage_options, converter_options)

        self._cut.set_args([metadata], args)

        filter = StorageFilter(topics=[args.topic])
        progress = ProgressTracker()
        if args.progress:
            progress.add_estimated_work(
                metadata, self._cut.output_size_factor(metadata))

        bag_view = BagView(reader, filter)
        fps = args.fps
        if not fps:
            fps = estimate_fps(args.bag_path, args.storage, args.topic)
        if args.output:
            processor = VideoWriter(args.output, fps)
        else:
            processor = VideoDisplay(args.topic, fps)

        image_bridge = CvBridge()
        for tpc, image, t in bag_view:
            cut_result = self._cut.filter_msg((tpc, image, t))
            if cut_result == FilterResult.DROP_MESSAGE:
                continue
            if cut_result == FilterResult.STOP_CURRENT_BAG:
                break

            if is_compressed:
                cv_image = image_bridge.compressed_imgmsg_to_cv2(image)
            else:
                cv_image = image_bridge.imgmsg_to_cv2(image, args.encoding)
            if args.image_resize:
                width = int(cv_image.shape[1] * args.image_resize)
                height = int(cv_image.shape[0] * args.image_resize)
                dim = (width, height)
                cv_image = cv2.resize(
                    cv_image, dim, interpolation=RESIZE_INTERPOLATION)

            if is_compressed:
                in_fmt, __orig_enc, enc = CompressedImageMsgWriter.normalize_format_desc(
                    image.format
                )
            else:
                enc = image.encoding

            cv_image = cvtColorForDisplay(
                cv_image, enc, args.encoding,
                do_dynamic_scaling=args.do_dynamic_scaling,
                min_image_value=args.min_image_value or 0.0,
                max_image_value=args.max_image_value or 0.0,
                # colormap=args.colormap
            )

            processor.process(cv_image)
            if args.progress:
                progress.print_update(progress.update(args.topic), every=10)

        if args.progress:
            progress.print_finish()
