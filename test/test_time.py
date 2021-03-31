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
from rclpy.time import Duration, Time, CONVERSION_CONSTANT
from datetime import time, timezone
from ros2bag_tools.time import DurationOrDayTimeType, ros_add_daytime
import pytest


def test_duration_or_daytime_type():
    day_time = DurationOrDayTimeType('13:00')
    assert(isinstance(day_time, time))
    assert(day_time == time(13, tzinfo=timezone.utc))

    day_time = DurationOrDayTimeType('0:10:10:999')
    assert(isinstance(day_time, time))
    assert(day_time == time(0, 10, 10, 999 * 1000, tzinfo=timezone.utc))

    day_time = DurationOrDayTimeType('0:00:00:500')
    assert(isinstance(day_time, time))
    assert(day_time == time(0, 0, 0, 500 * 1000, tzinfo=timezone.utc))

    duration = DurationOrDayTimeType('10')
    assert(isinstance(duration, Duration))
    assert(duration == Duration(seconds=10))

    with pytest.raises(ArgumentTypeError):
        DurationOrDayTimeType('a')

    with pytest.raises(ArgumentTypeError):
        DurationOrDayTimeType('13:60')


def test_ros_day_time():
    day_time = DurationOrDayTimeType('0:01')
    t_start = Time(seconds=1)
    t_end = ros_add_daytime(t_start, day_time)
    assert(t_end.nanoseconds == 60 * CONVERSION_CONSTANT)

    day_time = DurationOrDayTimeType('0:00:00:500')
    t_start = Time(seconds=1 + 24 * 60 * 60)
    t_end = ros_add_daytime(t_start, day_time)

    day_ns = 24 * 60 * 60 * CONVERSION_CONSTANT
    time_ns = 500 * 1000 * 1000
    assert(t_end.nanoseconds == day_ns + time_ns)
