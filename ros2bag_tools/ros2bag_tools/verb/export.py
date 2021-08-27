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
from rosbag2_tools.bag_view import BagView
from ros2bag.api import print_error
from ros2bag.verb import VerbExtension
from rosbag2_py import StorageFilter
from rclpy.time import Time
from geometry_msgs.msg import Vector3, Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from geodesy.utm import fromLatLong


class ExporterError(Exception):
    pass


class TUMTrajectoryExporter:

    @staticmethod
    def add_arguments(parser):
        parser.add_argument('--tum-precision', default=4, type=int)

    def process(self, args, odometry_iter):
        with open(args.output, 'w') as f:
            fmt = '{0:.' + str(args.tum_precision) + 'f}'
            
            # if the input is a geo pose it is UTM-projected, and we need to detect
            # (zone, band) changes from message to message
            previous_utm_zone_band = None

            for topic_name, msg, _ in odometry_iter:
                if isinstance(msg, Odometry):
                    pos = msg.pose.pose.position
                    ori = msg.pose.pose.orientation
                elif isinstance(msg, NavSatFix):
                    utm_point = fromLatLong(msg.latitude, msg.longitude, msg.altitude)
                    pos = Vector3()
                    pos.x = utm_point.easting
                    pos.y = utm_point.northing
                    pos.z = utm_point.altitude
                    utm_lattice_coordinate = (utm_point.zone, utm_point.band)
                    if previous_utm_zone_band and previous_utm_zone_band != utm_lattice_coordinate:
                        raise ExporterError(f'UTM (zone, band) changes between messages in topic {topic_name}')
                    previous_utm_zone_band = utm_lattice_coordinate
                    # orientation is unit for NavSatFix
                    ori = Quaternion()
                else:
                    raise TypeError(f'{self.__class__.__name__} can not export messages of type {msg.__class__.__name__}')

                t_ros = Time.from_msg(msg.header.stamp)
                t_sec = t_ros.nanoseconds / 1e9

                f.write(str(t_sec))
                f.write(' ')
                f.write(fmt.format(pos.x))
                f.write(' ')
                f.write(fmt.format(pos.y))
                f.write(' ')
                f.write(fmt.format(pos.z))
                f.write(' ')
                f.write(fmt.format(ori.x))
                f.write(' ')
                f.write(fmt.format(ori.y))
                f.write(' ')
                f.write(fmt.format(ori.z))
                f.write(' ')
                f.write(fmt.format(ori.w))
                f.write('\n')


class ExportVerb(VerbExtension):
    """Export bag data to other formats."""

    def add_arguments(self, parser, cli_name):  # noqa: D102
        parser.add_argument(
            'bag_file', help='bag file to read data from')
        parser.add_argument('-t', '--topic', nargs='+', type=str,
                            help='Topics with optional field names to export')
        parser.add_argument('-o', '--output', required=True, type=str,
                            help='Path to output file')
        parser.add_argument('--format', type=str,
                            help='Format to export to')
        TUMTrajectoryExporter.add_arguments(parser)

    def main(self, *, args):  # noqa: D102
        if not os.path.exists(args.bag_file):
            return print_error("bag file '{}' does not exist!".format(args.bag_file))

        if not args.topic:
            return print_error("topics to export are required")

        topics_with_field = [tuple(t.split('.', 1)) for t in args.topic]

        filter = StorageFilter(topics=[twf[0] for twf in topics_with_field])
        view = BagView(args.bag_file, filter)

        exporter = TUMTrajectoryExporter()
        try:
            exporter.process(args, view)
        except ExporterError as e:
            return print_error(f'export failed: {str(e)}')
