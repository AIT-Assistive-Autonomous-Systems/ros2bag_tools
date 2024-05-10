# Copyright 2022 AIT Austrian Institute of Technology GmbH
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

from abc import ABCMeta
from abc import abstractmethod
from pathlib import Path
from typing import Any, AnyStr, Optional, Tuple, Union

import cv2 as cv
from cv_bridge import CvBridge
from cv_bridge import CvBridgeError
from cv_bridge.boost.cv_bridge_boost import cvtColor2
import numpy as np
from ros2bag_tools.exporter import Exporter
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image


def check_override_encoding(
    in_enc: Union[str, int],
    out_enc: Union[str, int],
    bridge: CvBridge
) -> str:
    if not in_enc and not isinstance(in_enc, int):
        return in_enc
    if not out_enc and not isinstance(out_enc, int):
        return in_enc
    in_type = bridge.cvtype2_to_dtype_with_channels(in_enc) if isinstance(
        in_enc, int) else bridge.encoding_to_dtype_with_channels(in_enc)
    out_type = bridge.cvtype2_to_dtype_with_channels(out_enc) if isinstance(
        out_enc, int) else bridge.encoding_to_dtype_with_channels(out_enc)

    in_size = np.dtype(in_type[0]).itemsize * in_type[1]
    out_size = np.dtype(out_type[0]).itemsize * out_type[1]
    if in_size != out_size:
        raise ValueError(
            f'Input type and output type size per pixel do not match ({in_enc}, {out_enc})')
    return out_enc


def bayer_conversion_code(bayer_encoding: str, desired_encoding: str, demosaicing: str) -> int:
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


class ImageMsgWriterBase(metaclass=ABCMeta):
    """
    Base class image message types to buffers.

    Encodes type specific messages to specific buffer outputs representing the extension.
    """

    @staticmethod
    @abstractmethod
    def get_supported_msg_type() -> Any:
        pass

    @abstractmethod
    def process(
        self,
        img_msg,
        out_enc: str,
        demosaicing: Optional[str],
        ext: str,
        in_enc: Optional[str] = None,
    ) -> AnyStr:
        """
        Write encoded image to buffer.

        Args:
        ----
        img_msg: Input message to be extracted/recoded.
        out_enc: Desired output encoding or None/Empty.
        demosaicing: Demosaicing type if desired or None.
        ext: Extension or suffix of the representing the output format/file.
             For compressed formats if the stored format is the same as the
             requested format no decoding or recoding will be performed.
        in_enc: Override image encoding derived from message (or None/Empty).
                If the input encoding encoding can be derived without decoding
                the image and the output encoding is the same no decoding occurs.
                If they differ or color conversion may be performed.
                Overriding only works if the derived format and in_enc are of same
                input size.

        Return:
        ------
            A str/byte like object suitable for writing to a binary file.

        """


class ImageMsgWriter(ImageMsgWriterBase):
    """
    Encode and Decode for standard image message.

    Decoding to image Mat is accomplished using cv_bridge and encoding using cv.imencode.
    Thus any restrictions apply based on these functions.
    """

    def __init__(self):
        self.image_bridge = CvBridge()

    @staticmethod
    def get_supported_msg_type() -> Any:
        return Image

    @staticmethod
    def convert_encoding(img, in_enc, out_enc, demosaicing):
        """Convert colorspace encoding and debayer if desired."""
        if in_enc.startswith('bayer_') and demosaicing:
            # use custom debayering
            # cvtColor2 unfortunately doesn't expose a demosaicing algorithm parameter
            conv_code = bayer_conversion_code(
                in_enc, out_enc, demosaicing)
            img = cv.cvtColor(img, conv_code)
        else:
            try:
                img = cvtColor2(img, in_enc, out_enc)
            except RuntimeError as e:
                raise CvBridgeError(e)
        return img

    def process(
        self,
        img_msg: Image,
        out_enc: str,
        demosaicing: Optional[str],
        ext: str,
        in_enc: Optional[str] = None
    ) -> AnyStr:
        img = self.image_bridge.imgmsg_to_cv2(img_msg)
        in_enc = check_override_encoding(
            img_msg.encoding, in_enc, self.image_bridge)
        if out_enc and in_enc != out_enc:
            img = ImageMsgWriter.convert_encoding(
                img, in_enc, out_enc, demosaicing)
        reval, buf = cv.imencode(ext, img)

        if not reval:
            raise RuntimeError(
                f'Could not encode image: shape={img.shape}, in_enc={in_enc}, out_enc={out_enc}')
        return buf


class CompressedImageMsgWriter(ImageMsgWriterBase):
    """
    Encode and Decode for standard image message.

    Decoding to image Mat is accomplished using cv_bridge and encoding using cv.imencode.
    Thus any restrictions apply based on these functions.
    """

    def __init__(self):
        self.image_bridge = CvBridge()

    @staticmethod
    def get_supported_msg_type():
        return CompressedImage

    @staticmethod
    def get_ext_format(ext: str) -> str:
        ext = ext.lower()[1:]
        if ext in ['jpg', 'jpeg']:
            ext = 'jpeg'
        return ext

    @staticmethod
    def normalize_format_desc(desc: str) -> Tuple[str, Optional[str], Optional[str]]:
        """
        Return normalized format from CompressedImage format descriptor.

        A new format descriptor of the CompressedImage transport will look like this:
            <original encoding>; <format> compressed <target/stored encoding>
        While old descriptors only contains <format>.

        Return:
        ------
            A tuple containing the normalized format and the original encoding and stored
            encoding (or None/Empty str).

        """
        descs = desc.lower().split(';')
        if len(descs) > 2:
            raise ValueError(
                f'CompressedImage format descriptor not supported: {desc}')
        enc, fmt = (None, descs[0]) if len(descs) < 2 else descs
        fmt = fmt.split('compressed')
        if len(fmt) > 2:
            raise ValueError(
                f'CompressedImage format descriptor not supported: {desc}')
        fmt, compr_enc = (fmt[0], None) if len(fmt) < 2 else fmt
        fmt = fmt.strip(' ')

        if compr_enc:
            compr_enc = compr_enc.strip(' ')
        if enc:
            enc = enc.strip(' ')
        if fmt in ['jpeg', 'jpg']:
            fmt = 'jpeg'
        return fmt, enc, compr_enc

    @staticmethod
    def cv2_to_enc(cvim: np.ndarray):
        """Guess default encoding from shape."""
        if len(cvim.shape) < 3:
            encoding = 'mono'
        elif cvim.shape[2] == 3:
            encoding = 'bgr'
        elif cvim.shape[2] == 4:
            encoding = 'bgra'
        else:
            raise RuntimeError(
                f'Cannot guess encoding from data shape: {cvim.shape}')
        return f'{encoding}{cvim.itemsize*8}'

    def process(
        self,
        img_msg: CompressedImage,
        out_enc: str,
        demosaicing: Optional[str],
        ext: str,
        in_enc: Optional[str] = None
    ) -> AnyStr:
        img = None
        in_fmt, __orig_enc, compr_enc = CompressedImageMsgWriter.normalize_format_desc(
            img_msg.format
        )
        in_enc = check_override_encoding(compr_enc, in_enc, self.image_bridge)
        convert = in_enc != out_enc and out_enc
        out_fmt = CompressedImageMsgWriter.get_ext_format(ext)
        decode = convert or in_fmt != out_fmt
        if decode:
            img = self.image_bridge.compressed_imgmsg_to_cv2(img_msg)
        if convert:
            if not in_enc:
                in_enc = CompressedImageMsgWriter.cv2_to_enc(img)
            if in_enc != out_enc:
                img = ImageMsgWriter.convert_encoding(
                    img, in_enc, out_enc, demosaicing)
        if img is not None:
            reval, buf = cv.imencode(ext, img)
        else:
            reval, buf = True, img_msg.data
        if not reval:
            raise RuntimeError(
                f'Could not encode image: shape={img.shape}, in_enc={in_enc}, out_enc={out_enc}')
        return buf


def image_msg_writer_factory(msg):
    # Could be solved via extension points in the future
    writers = [ImageMsgWriter, CompressedImageMsgWriter]
    msg_to_writer = {w.get_supported_msg_type(): w for w in writers}

    return msg_to_writer[type(msg)]()


def none_if(val, none_val):
    if val == none_val:
        return None
    return val


class ImageExporter(Exporter):
    """Image files per message."""

    def __init__(self):
        self._idx = 0
        self._writers = {}
        self._args = None
        self._dir = None

    @staticmethod
    def add_arguments(parser):
        parser.add_argument('--dir', default='.', help='Output directory')
        parser.add_argument('--name', default='%t.png',
                            help="""Filename pattern of output images.
                            Placeholders:
                                %%tpc ... topic
                                %%t   ... timestamp
                                %%i   ... sequence index""")
        parser.add_argument('--output-encoding', default='passthrough',
                            help='Output image encoding.')
        parser.add_argument('--demosaicing', choices=['linear', 'vng', 'ea'],
                            help='Bayer pattern demosaicing algorithm')
        parser.add_argument('--input-encoding', default='passthrough',
                            help='Override input encoding, (e.g. if input is gray but bayer)')

    def open(self, args):  # noqa: A003
        self._idx = 0
        self._dir = Path(args.dir)

        if not Path(args.name).suffix:
            raise ValueError(
                f'Extension/Suffix of name {args.name} must not be empty')

        self._dir.mkdir(parents=True, exist_ok=True)

        self._input_encoding = none_if(args.input_encoding, 'passthrough')
        self._output_encoding = none_if(args.output_encoding, 'passthrough')
        self._args = args

    def write(self, topic, img_msg, t):
        writer = self._writers.get(type(img_msg), None)
        if not writer:
            writer = image_msg_writer_factory(img_msg)
            self._writers[type(img_msg)] = writer

        tpc_path = topic.lstrip('/').replace('/', '_')
        filename = self._args.name.replace('%tpc', tpc_path)
        filename = filename.replace('%t', str(t))
        filename = filename.replace('%i', str(self._idx).zfill(8))
        img_path = self._dir / filename
        ext = img_path.suffix

        buf = writer.process(
            img_msg,
            self._output_encoding,
            self._args.demosaicing,
            ext,
            self._input_encoding
        )

        with img_path.open('wb') as f:
            f.write(buf)
        self._idx += 1
