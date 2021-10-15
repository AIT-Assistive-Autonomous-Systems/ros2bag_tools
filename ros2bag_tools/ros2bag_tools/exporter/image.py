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

from cv_bridge import CvBridge, CvBridgeError
from cv_bridge.boost.cv_bridge_boost import cvtColor2
from pathlib import Path
import cv2 as cv


def conversion_code(bayer_encoding, desired_encoding, demosaicing):
    algorithm = ''
    if demosaicing == 'vng':
        algorithm = '_VNG'
    elif demosaicing == 'ea':
        algorithm = '_EA'
    pattern = bayer_encoding.lstrip('bayer')[1:3].upper()
    colorspace = 'GRAY'
    enc = desired_encoding.upper()
    if enc.startswith('RGB'):
        colorspace = 'RGB'
    if enc.startswith('BGR'):
        colorspace = 'BGR'
    return getattr(cv, f'COLOR_BAYER_{pattern}2{colorspace}{algorithm}')


class ImageExporter:
    """Image files per message"""

    @staticmethod
    def add_arguments(parser):
        parser.add_argument('--dir', default='.', help='Output directory')
        parser.add_argument('--name', default='%t.png',
                            help="""Filename pattern of output images.
                            Placeholders:
                                %tpc ... topic
                                %t   ... timestamp
                                %i   ... sequence index""")
        parser.add_argument('--encoding', default='passthrough',
                            help='Output image encoding')
        parser.add_argument('--demosaicing', choices=['linear', 'vng', 'ea'],
                            help='Bayer pattern demosaicing algorithm')

    def process(self, args, images):
        dir = Path(args.dir)
        image_bridge = CvBridge()
        idx = 0

        dir.mkdir(parents=True, exist_ok=True)
        for topic, img_msg, t in images:
            img = image_bridge.imgmsg_to_cv2(img_msg)

            if img_msg.encoding.startswith('bayer_') and args.demosaicing:
                # use custom debayering
                # cvtColor2 unfortunately doesn't expose a demosaicing algorithm parameter
                conv_code = conversion_code(img_msg.encoding, args.encoding, args.demosaicing)
                img = cv.cvtColor(img, conv_code)
            else:
                try:
                    img = cvtColor2(img, img_msg.encoding, args.encoding)
                except RuntimeError as e:
                    raise CvBridgeError(e)

            tpc_path = topic.lstrip('/').replace('/', '_')
            filename = args.name.replace('%tpc', tpc_path)
            filename = filename.replace('%t', str(t))
            filename = filename.replace('%i', str(idx).zfill(8))
            img_path = dir / filename
            cv.imwrite(str(img_path), img)
            idx += 1
