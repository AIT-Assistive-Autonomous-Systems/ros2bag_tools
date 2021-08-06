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


class ImageFilter(TypeAwareTopicFilter):

    def __init__(self):
        super(ImageFilter, self).__init__()
        self._cv_bridge = CvBridge()
        self._image_encoding = None
        self._image_size = None

    def add_arguments(self, parser):
        super().add_arguments(parser)
        parser.add_argument('--image-encoding', default='passthrough')
        parser.add_argument('--image-size', type=ImageResizeArg,
                            help=ImageResizeArg.__doc__)

    def set_args(self, in_files, out_file, args):
        super().set_args(in_files, out_file, args)
        self._image_encoding = args.image_encoding
        self._image_size = args.image_size

    def filter_typed_msg(self, msg):
        (topic, image, t) = msg
        cv_image = self._cv_bridge.imgmsg_to_cv2(
            image, self._image_encoding)
        if self._image_size:
            if isinstance(self._image_size, float):
                cv_image = cv.resize(cv_image, None, fx=self._image_size, fy=self._image_size,
                                     interpolation=cv.INTER_CUBIC)
            else:
                cv_image = cv.resize(cv_image, self._image_size, interpolation=cv.INTER_CUBIC)
        result_msg = self._cv_bridge.cv2_to_imgmsg(cv_image)
        if self._image_encoding != 'passthrough':
            # set encoding explicitly, as it is ambiguous between choices such as 8UC3 and BGR8,
            # and user most likely wants the encoding to be exactly the target encoding given to
            # the filter
            result_msg.encoding = self._image_encoding
        result_msg.header = image.header
        return (topic, result_msg, t)
