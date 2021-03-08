# Copyright 2020 AIT Austrian Institute of Technology GmbH
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
from datetime import datetime, timedelta, timezone
import os
import re
import yaml
from rclpy.time import Duration, Time
from ros2bag_tools.filter import BagMessageFilter


def get_bag_bounds(bag_path):
    # TODO(ZeilingerM): use rosbag2_py metadata read api as soon as it is available
    with open(os.path.join(bag_path, 'metadata.yaml'), 'r') as f:
        metadata = yaml.safe_load(f)

    bagfile_information = metadata['rosbag2_bagfile_information']

    start_ns = int(bagfile_information['starting_time']['nanoseconds_since_epoch'])
    duration_ns = int(bagfile_information['duration']['nanoseconds'])

    # TODO from sec impl
    ns_scale = 1000 * 1000 * 1000
    start_time = Time(seconds=start_ns / ns_scale, nanoseconds=start_ns % ns_scale)
    duration = Duration(seconds=duration_ns / ns_scale, nanoseconds=duration_ns % ns_scale)
    end_time = start_time + duration
    return (start_time, end_time)


def compute_timespan(start, duration, end, bags_start_time, bags_end_time):
    start_time = bags_start_time
    end_time = bags_end_time

    if start is not None:
        if isinstance(start, DayTime):
            start_time = ros_add_daytime(start_time, start)
        else:
            start_time += start
        if duration is not None:
            end_time = start_time + duration

    if end is not None:
        if isinstance(end, DayTime):
            end_time = ros_add_daytime(end_time, end)
        else:
            end_time = start_time + end
        if duration is not None:
            start_time = end_time - duration

    if start is None and end is None and duration is not None:
        end_time = start_time + duration

    return (start_time, end_time)


def time_seconds(t):
    return t.nanoseconds / (1000 * 1000 * 1000)


def duration_seconds(d):
    return d.nanoseconds / (1000 * 1000 * 1000)


def ros_to_utc(ros_time):
    return datetime.utcfromtimestamp(time_seconds(ros_time))


def utc_to_stamp(utc):
    epoch = datetime(1970, 1, 1, 0, 0, 0, tzinfo=timezone.utc)
    return int((utc - epoch).total_seconds())


def time_str(t):
    utc = ros_to_utc(t)
    ns = t.nanoseconds % (1000 * 1000 * 1000)
    return "{:02d}:{:02d}:{:02d}:{:09d}".format(utc.hour, utc.minute, utc.second, ns)


def ros_add_daytime(t, daytime):
    utc = ros_to_utc(t)
    utc_day = datetime(utc.year, utc.month, utc.day, 0, 0, 0, tzinfo=timezone.utc)
    delta = timedelta(seconds=daytime.s, minutes=daytime.m, hours=daytime.h)
    utc_stamp = utc_to_stamp(utc_day + delta)
    return Time(seconds=utc_stamp, nanoseconds=daytime.ms * 1000 * 1000)


def is_same_day(gmt1, gmt2):
    return gmt1.year == gmt2.year and gmt1.month == gmt2.month and gmt1.day == gmt2.day


class DayTime:
    def __init__(self, h, m=0, s=0, ms=0):
        self.h = h
        self.m = m
        self.s = s
        # milliseconds
        self.ms = ms

    def to_ros_dur(self):
        return Duration(self.h * 60 * 60 + self.m * 60 + self.s, self.ms * 1000 * 1000)

    def __gt__(self, other):
        if self.h > other.h:
            return True
        if self.h == other.h and self.m > other.m:
            return True
        if self.h == other.h and self.m == other.m and self.s > other.s:
            return True
        if (self.h == other.h and self.m == other.m
                and self.s == other.s and self.ms > other.ms):
            return True
        return False

    def __eq__(self, other):
        return (self.h == other.h and self.m == other.m
                and self.s == other.s and self.ms == other.ms)


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
            raise argparse.ArgumentTypeError("pass daytime as hh:mm:ss[:ms]")

        if len(match[0]) < 3 or len(match[0]) > 4:
            raise argparse.ArgumentTypeError("pass daytime as hh:mm:ss[:ms]")

        h = int(match[0][0])
        m = int(match[0][1])
        s = int(match[0][2])
        ms = 0

        if len(match[0]) == 4:
            ms = int(match[0][3])

        if h < 0 or h > 23:
            raise argparse.ArgumentTypeError("hour between 0 and 23")
        if m < 0 or m > 59:
            raise argparse.ArgumentTypeError("minute between 0 and 59")
        if s < 0 or s > 59:
            raise argparse.ArgumentTypeError("second between 0 and 59")
        if ms < 0 or ms > 999:
            raise argparse.ArgumentTypeError("millisecond between 0 and 999")

        return DayTime(h, m, s, ms)
    except ValueError:
        raise argparse.ArgumentTypeError("duration must be float (in seconds)")


def DurationOrDayTimeType(values):
    try:
        return DayTimeType(values)
    except argparse.ArgumentTypeError:
        return DurationType(values)


def print_timeline(start, end, bag_start, bag_end, filename):
    print('Cut times (UTC)')
    print('bag start: {} ({} ns since epoch)'.format(time_str(bag_start), bag_start.nanoseconds))
    print('bag end:   {} ({} ns since epoch)'.format(time_str(bag_end), bag_end.nanoseconds))
    print('cut start: {} ({} ns since epoch)'.format(time_str(start), start.nanoseconds))
    print('cut end:   {} ({} ns since epoch)'.format(time_str(end), end.nanoseconds))
    perc = (duration_seconds(end - start) / duration_seconds(bag_end - bag_start)) * 100
    print('{:.2f}% of bag will be copied to {}'.format(perc, filename))


class CutFilter(BagMessageFilter):

    def __init__(self):
        self._start = None
        self._end = None

    def add_arguments(self, parser):
        parser.add_argument(
            '--start', help='cut start time (utc time of day or offset in seconds)',
            type=DurationOrDayTimeType)
        parser.add_argument(
            '--end', help='cut end time (utc time of day or offset in seconds)',
            type=DurationOrDayTimeType)
        parser.add_argument(
            '--duration', help='duration of the time span that is cut',
            type=DurationType)

    def set_args(self, in_file, out_file, args):
        (bag_start, bag_end) = get_bag_bounds(in_file)

        if args.start is not None and args.end is not None and args.duration is not None:
            raise argparse.ArgumentError(
                'Only one or two of --start, --end and --duration may be used')

        start_end_on_same_day = is_same_day(ros_to_utc(bag_start), ros_to_utc(bag_end))

        start_is_daytime = isinstance(args.start, DayTime)
        end_is_daytime = isinstance(args.end, DayTime)
        if not start_end_on_same_day and (start_is_daytime or end_is_daytime):
            raise argparse.ArgumentError(
                """when using start and end day times
                the start and end of the bag must be on the same day""")

        if start_is_daytime and end_is_daytime and args.start > args.end:
            raise argparse.ArgumentError('start daytime is after end daytime')

        bag_duration = bag_end - bag_start
        if args.duration is not None and args.duration > bag_duration:
            raise argparse.ArgumentError('--duration is larger than bag duration')
        if args.start is not None and not start_is_daytime and args.start > bag_duration:
            raise argparse.ArgumentError('--start offset is larger than bag duration')
        if args.end is not None and not end_is_daytime and args.end > bag_duration:
            raise argparse.ArgumentError('--end offset is larger than bag duration')

        (start, end) = compute_timespan(args.start, args.duration, args.end, bag_start, bag_end)

        self._start = start
        self._end = end

        print_timeline(self._start, self._end, bag_start, bag_end, out_file)

    def filter_msg(self, msg):
        (_, _, t) = msg
        if t >= self._start.nanoseconds and t <= self._end.nanoseconds:
            return msg
        return None
