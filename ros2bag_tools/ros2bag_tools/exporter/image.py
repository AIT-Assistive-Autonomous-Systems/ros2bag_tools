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
from sensor_msgs.msg import Image, CompressedImage
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

class ImageMsgWriter:
    def __init__(self):
        self.image_bridge = CvBridge()
    @staticmethod
    def get_message_type():
        """Get the supported message type.
        """
        return Image
    @staticmethod
    def convert_encoding(img, img_encoding, encoding, demosaicing):
        """Convert colorspace encoding and debayer if necessary.
        """
        if img_encoding.startswith('bayer_') and demosaicing:
            # use custom debayering
            # cvtColor2 unfortunately doesn't expose a demosaicing algorithm parameter
            conv_code = conversion_code(
                img_encoding, encoding, demosaicing)
            img = cv.cvtColor(img, conv_code)
        else:
            try:
                img = cvtColor2(img, img_encoding, encoding)
            except RuntimeError as e:
                raise CvBridgeError(e)
        return img

    def process(self, img_msg, encoding, demosaicing, ext, input_encoding=None, params=None):
        """Write encoded image to buffer and return if successfull True, buf otherwise False, buf.
        """
        img = self.image_bridge.imgmsg_to_cv2(msg)
        if encoding != 'passthrough':
            in_encoding = img_msg.encoding if not input_encoding else input_encoding
            img = ImageMsgWriter.convert_encoding(img, in_encoding, encoding, demosaicing)
        reval, buf = cv.imencode(ext, img, params)
        return buf

class CompressedImageMsgWriter:
    def __init__(self):
        self.image_bridge = CvBridge()

    @staticmethod
    def get_message_type():
        """Get the supported message type.
        """
        return CompressedImage
    @staticmethod
    def get_format(ext):
        """Get compressed format from extension.
        """
        ext = ext.lower()
        if ext == '.jpg' or ext == '.jpeg':
            form = 'jpeg'
        else:
            form = ext
        return form
        
    @staticmethod
    def cv2_to_enc(cvim):
        """Guess default encoding from shape.
        """
        if len(cvim.shape) < 3:
            encoding = 'mono'
        elif cvim.shape[2] == 3:
            encoding = 'bgr'
        elif cvim.shape[2] == 4:
            encoding = 'bgra'
        return f'{encoding}{cvim.itemsize*8}'

    def process(self, img_msg, encoding, demosaicing, ext, input_encoding=None, params=None):
        """Write encoded image to buffer and return if successfull True, buf otherwise False, buf.
        """
        img = None
        convert = encoding != 'passthrough'
        if convert or img_msg.format != CompressedImageMsgWriter.get_format(ext):
            img = self.image_bridge.compressed_imgmsg_to_cv2(img_msg)
        if convert:
            in_encoding = input_encoding if input_encoding else CompressedImageMsgWriter.cv2_to_enc(img)
            img = ImageMsgWriter.convert_encoding(img, in_encoding, encoding, demosaicing)
        if img is not None:
            reval, buf = cv.imencode(ext, img, params)
        else:
            reval, buf = True, img_msg.data
        return reval, buf

def image_msg_writer_factory(msg):
    """Factory to select converter based on image message type.
    """
    # Could be solved via extension points in the future
    writers = [ImageMsgWriter, CompressedImageMsgWriter]
    msg_to_writer = { w.get_message_type(): w for w in writers }

    return msg_to_writer[type(msg)]()

class ImageExporter:
    """Image files per message"""

    @staticmethod
    def add_arguments(parser):
        parser.add_argument('--dir', default='.', help='Output directory')
        parser.add_argument('--name', default='%t.png',
                            help="""Filename pattern of output images.
                            Placeholders:
                                %%tpc ... topic
                                %%t   ... timestamp
                                %%i   ... sequence index""")
        parser.add_argument('--encoding', default='passthrough',
                            help='Output image encoding')
        parser.add_argument('--demosaicing', choices=['linear', 'vng', 'ea'],
                            help='Bayer pattern demosaicing algorithm')
        parser.add_argument('--input-encoding', default='',
                            help='Override intput encoding, (e.g. if input is gray but bayer)')

    def process(self, args, images):
        dir = Path(args.dir)
        image_bridge = CvBridge()
        idx = 0

        if not Path(args.name).suffix:
            raise  ValueError(f'Extension/Suffix of name {args.name} must not be empty')

        dir.mkdir(parents=True, exist_ok=True)
        writers = {}

        for topic, img_msg, t in images:
            writer = writers.get(type(img_msg), None) 
            if not writer:
                writer = image_msg_writer_factory(img_msg)
                writers[type(img_msg)] = writer

            tpc_path = topic.lstrip('/').replace('/', '_')
            filename = args.name.replace('%tpc', tpc_path)
            filename = filename.replace('%t', str(t))
            filename = filename.replace('%i', str(idx).zfill(8))
            img_path = dir / filename
            ext = img_path.suffix

            retval, buf = writer.process(img_msg, args.encoding, args.demosaicing, ext, args.input_encoding)

            if retval:
                with img_path.open('wb') as f:
                    f.write(buf)
            else:
                raise  RuntimeError('Image could not be processed/encoded')
            idx += 1
