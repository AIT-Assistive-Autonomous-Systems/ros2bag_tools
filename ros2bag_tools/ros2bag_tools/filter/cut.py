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
import yaml
from datetime import datetime, time, timezone
from rclpy.qos import QoSDurabilityPolicy
from rclpy.serialization import serialize_message
from ros2bag_tools.reader import TopicDeserializer
from ros2bag_tools.filter import FilterExtension, FilterResult
from ros2bag_tools.filter.restamp import set_header_stamp
from ros2bag_tools.time import DurationOrDayTimeType, DurationType, get_bag_bounds, is_same_day,\
    datetime_to_ros_time, add_daytime, ros_to_datetime_utc


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


class CutFilter(FilterExtension):

    def __init__(self):
        self._start_time = None
        self._end_time = None
        self._start_arg = None
        self._end_arg = None
        self._duration_arg = None
        self._transient_local_policy = None
        self._transient_local_policy_arg = None
        self._topic_qos_durability_dict = {}
        self._deserializer = TopicDeserializer()

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
        self._transient_local_policy_arg = parser.add_argument(
            '--transient-local-policy',
            default='snap',
            choices=['snap', 'keep', 'drop'],
            help='Specifiy how to handle messages with transient_local durability with timestamps '
            'prior to the start cut time, defaults to "snap". '
            '"snap": set timestamp of prior transient_local messages to the start time '
            'of the new bag. '
            '"keep": keep prior transient_local messages with their original timestamp, '
            'might lead to time gaps in the bag. '
            '"drop": drop all prior transient_local messages')

    def set_args(self, metadatas, args):
        (bag_start, bag_end) = get_bag_bounds(metadatas)
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
            raise argparse.ArgumentError(
                None, 'start daytime is after end daytime')

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

        (start, end) = compute_timespan(args.start,
                                        args.duration, args.end, bag_start, bag_end)
        if end < bag_start or start > bag_end:
            raise argparse.ArgumentError(
                None,
                f'time bounds ({start}, {end}) are outside the '
                + f'duration of the input bags ({bag_start}, {bag_end})')

        self._start_time = datetime_to_ros_time(start)
        self._end_time = datetime_to_ros_time(end)

        self._transient_local_policy = args.transient_local_policy
        for metadata in metadatas:
            for topic in metadata.topics_with_message_count:
                self._topic_qos_durability_dict[topic.topic_metadata.name] = yaml.safe_load(
                    topic.topic_metadata.offered_qos_profiles)[0]['durability']

    def output_size_factor(self, metadata):
        start = metadata.starting_time.astimezone(timezone.utc)
        end = start + metadata.duration
        filter_start = ros_to_datetime_utc(self._start_time)
        filter_end = ros_to_datetime_utc(self._end_time)
        if start < filter_start:
            start = filter_start
        if filter_end < end:
            end = filter_end
        return min(1, max(0, (end - start) / metadata.duration))

    def filter_topic(self, topic_metadata):
        self._deserializer.add_topic(topic_metadata)
        return topic_metadata

    def filter_msg(self, serialized_msg):
        (topic, data, t) = serialized_msg
        # You may assume that this filtering is not necessary, as get_storage_filter sets
        # the filters on storage level. There are two cases when this is not sufficient:
        # * filters sequenced before or after this instance of cut may change the message
        #   timestamp to something else than in the bag data
        # * the underlying storage implementation does not support storage time filters
        if t < self._start_time.nanoseconds:
            if self._topic_qos_durability_dict[topic] == QoSDurabilityPolicy.TRANSIENT_LOCAL:
                if self._transient_local_policy == 'drop':
                    return FilterResult.DROP_MESSAGE
                elif self._transient_local_policy == 'snap':
                    msg = self._deserializer.deserialize(topic, data)
                    msg = set_header_stamp(msg, self._start_time.nanoseconds)
                    t = self._start_time.nanoseconds
                    data = serialize_message(msg)
                elif self._transient_local_policy == 'keep':
                    pass
            else:
                return FilterResult.DROP_MESSAGE
        if t > self._end_time.nanoseconds:
            return FilterResult.STOP_CURRENT_BAG
        return (topic, data, t)
