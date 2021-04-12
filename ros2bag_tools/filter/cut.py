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
from datetime import datetime, time
from ros2bag_tools.filter import BagMessageFilter, FilterResult
from ros2bag_tools.time import DurationOrDayTimeType, DurationType, get_bag_bounds, is_same_day,\
    datetime_to_ros_time, add_daytime


def compute_timespan(start, duration, end, bags_start_time: datetime, bags_end_time: datetime):
    start_time = bags_start_time
    end_time = bags_end_time

    if start is not None:
        if isinstance(start, time):
            assert(start_time.date() == end_time.date())
            start_time = add_daytime(start_time.date(), start)
        else:
            start_time += start
        if duration is not None:
            end_time = start_time + duration

    if end is not None:
        if isinstance(end, time):
            assert(start_time.date() == end_time.date())
            end_time = add_daytime(start_time.date(), end)
        else:
            end_time = start_time + end
        if duration is not None:
            start_time = end_time - duration

    if start is None and end is None and duration is not None:
        end_time = start_time + duration

    return (start_time, end_time)


def open_reader(path):
    from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
    reader = SequentialReader()
    storage_options = StorageOptions(uri=path, storage_id='sqlite3')
    converter_options = ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr')
    reader.open(storage_options, converter_options)
    return reader


class CutFilter(BagMessageFilter):

    def __init__(self):
        self._start_time = None
        self._end_time = None
        self._start_arg = None
        self._end_arg = None
        self._duration_arg = None

    def add_arguments(self, parser):
        self._start_arg = parser.add_argument(
            '--start', help='cut start time (utc time of day in hh:mm[:ss], or offset in seconds)',
            type=DurationOrDayTimeType)
        self._end_arg = parser.add_argument(
            '--end', help='cut end time (utc time of day in hh:mm[:ss], or offset in seconds)',
            type=DurationOrDayTimeType)
        self._duration_arg = parser.add_argument(
            '--duration', help='duration of the output time span',
            type=DurationType)

    def set_args(self, in_files, _out_file, args):
        readers = [open_reader(file_name) for file_name in in_files]
        (bag_start, bag_end) = get_bag_bounds(readers)

        if args.start is not None and args.end is not None and args.duration is not None:
            raise argparse.ArgumentError(
                self._duration_arg,
                'Only one or two of --start, --end and --duration may be used')

        start_end_on_same_day = is_same_day(bag_start, bag_end)
        start_is_daytime = isinstance(args.start, time)
        end_is_daytime = isinstance(args.end, time)
        if not start_end_on_same_day and (start_is_daytime or end_is_daytime):
            raise argparse.ArgumentError(
                self._end_arg,
                """when using start and end day times
                the start and end of the bag must be on the same day""")

        if start_is_daytime and end_is_daytime and args.start > args.end:
            raise argparse.ArgumentError(None, 'start daytime is after end daytime')

        bag_duration = bag_end - bag_start
        if args.duration is not None and args.duration > bag_duration:
            raise argparse.ArgumentError(
                self._duration_arg,
                '--duration is larger than bag duration')
        if args.start is not None and not start_is_daytime and args.start > bag_duration:
            raise argparse.ArgumentError(
                self._start_arg,
                '--start offset is larger than bag duration')
        if args.end is not None and not end_is_daytime and args.end > bag_duration:
            raise argparse.ArgumentError(
                self._end_arg,
                '--end offset is larger than bag duration')

        (start, end) = compute_timespan(args.start, args.duration, args.end, bag_start, bag_end)
        if end < bag_start or start > bag_end:
            raise argparse.ArgumentError(
                None,
                "time bounds are outside the duration of the input bags")

        self._start_time = datetime_to_ros_time(start)
        self._end_time = datetime_to_ros_time(end)

    def set_storage_filter(self, storage_filter):
        # check existing filter values, to ensure the filter is not changed to filter less than
        # previously set
        if self._start_time.nanoseconds > storage_filter.start_time:
            storage_filter.start_time = self._start_time.nanoseconds
        if self._start_time.nanoseconds < storage_filter.stop_time:
            storage_filter.stop_time = self._end_time.nanoseconds

    def filter_msg(self, msg):
        (_, _, t) = msg
        if t < self._start_time.nanoseconds:
            return FilterResult.DROP_MESSAGE
        if t > self._end_time.nanoseconds:
            return FilterResult.STOP_CURRENT_BAG
        return msg
