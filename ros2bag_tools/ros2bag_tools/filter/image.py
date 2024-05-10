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

import argparse

import cv2 as cv
from cv_bridge import CvBridge

from ros2bag_tools.filter import TypeAwareTopicFilter

from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image


def ImageResizeArg(value):
    """Either a floating point scale factor, or image resolution in width x height form."""
    try:
        return float(value)
    except ValueError:
        if 'x' in value:
            parts = value.split('x')
            if len(parts) == 2:
                try:
                    return (int(parts[0]), int(parts[1]))
                except ValueError:
                    pass
    raise argparse.ArgumentError(
        None, f'{value} must be floating point value, or image resolution in width x height form')


def resize_image(cv_im, resize_arg):
    if isinstance(resize_arg, float):
        return cv.resize(cv_im, None, fx=resize_arg, fy=resize_arg, interpolation=cv.INTER_CUBIC)
    else:
        return cv.resize(cv_im, resize_arg, interpolation=cv.INTER_CUBIC)


def resize_camera_info(msg: CameraInfo, resize_arg):
    scale_y = 0.0
    scale_x = 0.0
    if isinstance(resize_arg, float):
        scale_y = resize_arg
        scale_x = resize_arg
        msg.height = int(msg.height * scale_y)
        msg.width = int(msg.width * scale_x)
    else:
        new_height = resize_arg[1]
        new_width = resize_arg[0]
        scale_y = new_height / float(msg.height)
        scale_x = new_width / float(msg.width)
        msg.height = new_height
        msg.width = new_width

    msg.k[0] = msg.k[0] * scale_x  # fx
    msg.k[2] = msg.k[2] * scale_x  # cx
    msg.k[4] = msg.k[4] * scale_y  # fy
    msg.k[5] = msg.k[5] * scale_y  # cy

    msg.p[0] = msg.p[0] * scale_x  # fx
    msg.p[2] = msg.p[2] * scale_x  # cx
    msg.p[3] = msg.p[3] * scale_x  # T
    msg.p[5] = msg.p[5] * scale_y  # fy
    msg.p[6] = msg.p[6] * scale_y  # cy
    return msg


class ImageFilter(TypeAwareTopicFilter):

    def __init__(self):
        super(ImageFilter, self).__init__()
        self._cv_bridge = CvBridge()
        self._image_encoding = None
        self._image_size = None

    def add_arguments(self, parser):
        super().add_arguments(parser)
        parser.add_argument('--image-encoding', type=str, default='passthrough')
        parser.add_argument('--image-size', type=ImageResizeArg,
                            help=ImageResizeArg.__doc__)

    def set_args(self, metadata, args):
        super().set_args(metadata, args)
        self._image_encoding = args.image_encoding
        self._image_size = args.image_size

    def filter_typed_msg(self, item):
        (topic, msg, t) = item
        if isinstance(msg, Image):
            # to cv, with optional conversion
            cv_image = self._cv_bridge.imgmsg_to_cv2(msg, self._image_encoding)
            if self._image_size:
                cv_image = resize_image(cv_image, self._image_size)
            result_msg = self._cv_bridge.cv2_to_imgmsg(cv_image)
            if self._image_encoding != 'passthrough':
                # set encoding explicitly, as it is ambiguous between choices such as 8UC3/BGR8,
                # and the desired string is likely to be the exact argument
                result_msg.encoding = self._image_encoding
            result_msg.header = msg.header
            msg = result_msg
        elif isinstance(msg, CameraInfo):
            # taken from https://index.ros.org/p/image_proc/ to match its implementation
            if self._image_size:
                msg = resize_camera_info(msg, self._image_size)
            else:
                raise ValueError('can only apply resize to CameraInfo')
        else:
            raise ValueError(f'cannot filter message of type {type(msg)} with ImageFilter')
        return (topic, msg, t)
