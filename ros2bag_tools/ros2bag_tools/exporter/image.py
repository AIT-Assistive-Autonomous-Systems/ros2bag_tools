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

    @staticmethod
    def add_arguments(parser):
        parser.add_argument('--dir', default='.', help='Output directory')
        parser.add_argument('--name', default='%t.png',
                            help='Filename pattern of output images. Placeholders: %tpc for topic, %t for timestamp, %i for sequence index')
        parser.add_argument('--encoding', default='passthrough',
                            help='Output image encoding')
        parser.add_argument('--demosaicing', choices=['linear', 'vng', 'ea'],
                            help='Bayer pattern demosaicing algorithm')

    def process(self, args, images):
        dir = Path(args.dir)
        image_bridge = CvBridge()
        idx = 0
        for topic, img_msg, t in images:
            tpc_path = topic.lstrip('/').replace('/', '_')
            sub_dir = dir / tpc_path
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

            sub_dir.mkdir(parents=True, exist_ok=True)
            filename = args.name.replace('%tpc', tpc_path)
            filename = filename.replace('%t', str(t))
            filename = filename.replace('%i', str(idx).zfill(8))
            img_path = sub_dir / filename
            cv.imwrite(str(img_path), img)
            idx += 1
