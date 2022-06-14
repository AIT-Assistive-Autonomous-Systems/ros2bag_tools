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

from rclpy.time import Time
from geometry_msgs.msg import Vector3, Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from geodesy.utm import fromLatLong
from ros2bag_tools.exporter import ExporterError


class TUMTrajectoryExporter:
    """TUM pose trajectories."""

    @staticmethod
    def add_arguments(parser):
        parser.add_argument('--precision', default=4, type=int)
        parser.add_argument('--out', '-o', required=True, type=str, help='Path to output file')

    def process(self, args, odometry_iter):
        with open(args.out, 'w') as f:
            fmt = '{0:.' + str(args.precision) + 'f}'

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
                        msg = f'UTM (zone, band) changes between messages in topic {topic_name}'
                        raise ExporterError(msg)
                    previous_utm_zone_band = utm_lattice_coordinate
                    # orientation is unit for NavSatFix
                    ori = Quaternion()
                else:
                    exp_clz = self.__class__.__name__
                    msg_clz = msg.__class__.__name__
                    raise TypeError(f'{exp_clz} can not export messages of type {msg_clz}')

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
