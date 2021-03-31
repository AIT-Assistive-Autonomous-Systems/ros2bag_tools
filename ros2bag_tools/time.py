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
from datetime import date, datetime, timezone, time
import os
import re
import yaml
from rclpy.time import Duration, Time, CONVERSION_CONSTANT


def ros_time_from_nanoseconds(ns):
    time_s = int(ns / CONVERSION_CONSTANT)
    time_ns_only = ns % CONVERSION_CONSTANT
    return Time(seconds=time_s, nanoseconds=time_ns_only)


def ros_duration_from_nanoseconds(ns):
    duration_s = int(ns / CONVERSION_CONSTANT)
    duration_ns_only = ns % CONVERSION_CONSTANT
    return Duration(seconds=duration_s, nanoseconds=duration_ns_only)


def get_bag_bounds(bag_paths):
    # TODO(ZeilingerM): use rosbag2_py metadata read api instead, as soon as it is available
    total_start = Time(seconds=2**63/CONVERSION_CONSTANT)
    total_end = Time()

    for bag_path in bag_paths:
        with open(os.path.join(bag_path, 'metadata.yaml'), 'r') as f:
            metadata = yaml.safe_load(f)

        bagfile_information = metadata['rosbag2_bagfile_information']

        start_ns = int(bagfile_information['starting_time']['nanoseconds_since_epoch'])
        duration_ns = int(bagfile_information['duration']['nanoseconds'])
        start_time = ros_time_from_nanoseconds(start_ns)
        duration = ros_duration_from_nanoseconds(duration_ns)
        end_time = start_time + duration

        if start_time < total_start:
            total_start = start_time
        if end_time > total_end:
            total_end = end_time
    return (total_start, total_end)


def duration_seconds(d):
    return d.nanoseconds / (1000 * 1000 * 1000)


def ros_to_utc(ros_time):
    s = ros_time.seconds_nanoseconds()[0]
    return datetime.utcfromtimestamp(s)


def ros_add_daytime(t, day_time: time):
    """
    Get ROS time of a point in time on the same day but at given day time.
    """
    utc_day = ros_to_utc(t).date()
    utc_day_time = datetime.combine(utc_day, datetime.min.time())
    day_offset = datetime.combine(date.min, day_time) - datetime.min.replace(tzinfo=timezone.utc)
    utc_stamp = (utc_day_time + day_offset).timestamp()
    return Time(seconds=utc_stamp)


def is_same_day(gmt1, gmt2):
    return gmt1.year == gmt2.year and gmt1.month == gmt2.month and gmt1.day == gmt2.day


def DurationType(values):
    try:
        seconds = float(values)
        if seconds < 0:
            raise argparse.ArgumentTypeError("duration must be positive")
        sn = int((seconds - int(seconds)) * 1000 * 1000 * 1000)
        return Duration(seconds=int(seconds), nanoseconds=sn)
    except ValueError:
        raise argparse.ArgumentTypeError("duration must be float (in seconds)")


def DayTimeType(values):
    try:
        match = re.findall(r"(\d+):(\d+):(\d+):(\d+)", values)
        if len(match) == 0:
            match = re.findall(r"(\d+):(\d+):(\d+)", values)
        if len(match) == 0:
            match = re.findall(r"(\d+):(\d+)", values)
        if len(match) == 0:
            raise argparse.ArgumentTypeError("pass daytime as hh:mm[:ss[:ms]]")

        if len(match[0]) < 2 or len(match[0]) > 4:
            raise argparse.ArgumentTypeError("pass daytime as hh:mm[:ss[:ms]]")

        ms = 0
        s = 0
        h = int(match[0][0])
        m = int(match[0][1])
        if len(match[0]) >= 3:
            s = int(match[0][2])
        if len(match[0]) >= 4:
            ms = int(match[0][3])

        if h < 0 or h > 23:
            raise argparse.ArgumentTypeError("hour between 0 and 23")
        if m < 0 or m > 59:
            raise argparse.ArgumentTypeError("minute between 0 and 59")
        if s < 0 or s > 59:
            raise argparse.ArgumentTypeError("second between 0 and 59")
        if ms < 0 or ms > 999:
            raise argparse.ArgumentTypeError("millisecond between 0 and 999")

        return time(h, m, s, ms * 1000, tzinfo=timezone.utc)
    except ValueError:
        raise argparse.ArgumentTypeError("duration must be float (in seconds)")


def DurationOrDayTimeType(values):
    try:
        return DayTimeType(values)
    except argparse.ArgumentTypeError:
        return DurationType(values)
