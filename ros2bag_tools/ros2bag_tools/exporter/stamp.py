from rclpy.time import Time


class StampExporter:
    """Timestamps by message index"""

    @staticmethod
    def add_arguments(parser):
        parser.add_argument('--out', '-o', required=True, type=str,
                            help='Path to output file')
        parser.add_argument('--header', action='store_true',
                            help='Use header stamp rather than bag time')

    def process(self, args, msgs):
        idx = 0
        with open(args.out, 'w') as f:
            for _, img_msg, stamp in msgs:
                if args.header:
                    stamp = Time.from_msg(img_msg.header.stamp).nanoseconds
                f.write(f'{str(idx).zfill(8)},{stamp}\n')
                idx += 1
