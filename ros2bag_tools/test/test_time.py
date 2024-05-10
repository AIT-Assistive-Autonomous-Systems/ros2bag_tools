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

from argparse import ArgumentTypeError
from datetime import date
from datetime import datetime
from datetime import time
from datetime import timedelta
from datetime import timezone

import pytest

from rclpy.time import CONVERSION_CONSTANT

from ros2bag_tools.time import add_daytime
from ros2bag_tools.time import datetime_to_ros_time
from ros2bag_tools.time import DurationOrDayTimeType
from ros2bag_tools.time import get_bag_bounds

from rosbag2_py import Info


def test_duration_or_daytime_type():
    day_time = DurationOrDayTimeType('13:00')
    assert (day_time == time(13, tzinfo=timezone.utc))

    day_time = DurationOrDayTimeType('0:10:10:999')
    assert (day_time == time(0, 10, 10, 999 * 1000, tzinfo=timezone.utc))

    day_time = DurationOrDayTimeType('0:00:00:500')
    assert (day_time == time(0, 0, 0, 500 * 1000, tzinfo=timezone.utc))

    duration = DurationOrDayTimeType('10')
    assert (duration == timedelta(seconds=10))

    duration = DurationOrDayTimeType('10.5')
    assert (duration == timedelta(seconds=10, milliseconds=500))

    with pytest.raises(ArgumentTypeError):
        DurationOrDayTimeType('a')

    with pytest.raises(ArgumentTypeError):
        DurationOrDayTimeType('13:60')


def test_ros_day_time():
    day_time = DurationOrDayTimeType('0:01')
    t_start = date(1970, 1, 1)
    t_end = add_daytime(t_start, day_time)
    ros_t_end = datetime_to_ros_time(t_end)
    assert (ros_t_end.nanoseconds == 60 * CONVERSION_CONSTANT)

    day_time = DurationOrDayTimeType('0:00:00:500')
    t_start = date(1970, 1, 2)
    t_end = add_daytime(t_start, day_time)
    ros_t_end = datetime_to_ros_time(t_end)

    day_ns = 24 * 60 * 60 * CONVERSION_CONSTANT
    time_ns = 500 * 1000 * 1000
    assert (ros_t_end.nanoseconds == day_ns + time_ns)


def test_bag_bounds(tmp_day_time_bag):
    info = Info()
    metadata = info.read_metadata(tmp_day_time_bag, '')
    (bag_start, bag_end) = get_bag_bounds([metadata])
    assert (bag_start == datetime(1970, 1, 1, hour=12, minute=59,
           second=59, microsecond=999999, tzinfo=timezone.utc))
    assert (bag_end == datetime(1970, 1, 1, hour=14, minute=0,
           second=0, microsecond=1, tzinfo=timezone.utc))
