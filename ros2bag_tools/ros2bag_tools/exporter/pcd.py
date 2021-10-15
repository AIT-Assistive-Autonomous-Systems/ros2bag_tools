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

import numpy as np
from pathlib import Path


INT8 = 1
UINT8 = 2
INT16 = 3
UINT16 = 4
INT32 = 5
UINT32 = 6
FLOAT32 = 7
FLOAT64 = 8


def field_size(datatype):
    if datatype == INT8 or datatype == UINT8:
        return 1
    elif datatype == INT16 or datatype == UINT16:
        return 2
    elif datatype == FLOAT64:
        return 8
    else:
        return 4


def field_type_str(datatype):
    if datatype == INT8 or datatype == INT16 or datatype == INT32:
        return 'i'
    elif datatype == UINT8 or datatype == UINT16 or datatype == UINT32:
        return 'u'
    elif datatype == FLOAT32 or datatype == FLOAT64:
        return 'f'
    else:
        raise TypeError('unknown pcd datatype')


def pcd_type_to_np_type(datatype):
    if datatype == UINT8:
        return np.uint8
    if datatype == INT8:
        return np.int8
    elif datatype == FLOAT32:
        return np.float32
    else:
        raise TypeError(f'pcd field type {datatype} cannot be converted to numpy dtype')    


# def field_buffer(cloud, field):
#     n_points = cloud.height * cloud.width
#     return np.ndarray((n_points, field.count),
#                       dtype=field_type_to_np(field.datatype),
#                       buffer=cloud.data,
#                       offset=field.offset,
#                       strides=(1, cloud.point_step))


class PcdExporter:
    """ASCII PCD file per point cloud message"""

    @staticmethod
    def add_arguments(parser):
        parser.add_argument('--dir', default='.', help='Output directory')
        parser.add_argument('--name', default='%t.pcd',
                            help="""Filename pattern of output pcd files.
                            Placeholders:
                                %tpc ... topic
                                %t   ... timestamp
                                %i   ... sequence index""")

    def process(self, args, clouds):
        dir = Path(args.dir)
        idx = 0
        for topic, cloud, t in clouds:
            tpc_path = topic.lstrip('/').replace('/', '_')
            filename = args.name.replace('%tpc', tpc_path)
            filename = filename.replace('%t', str(t))
            filename = filename.replace('%i', str(idx).zfill(8))
            cloud_path = dir / filename

            with open(str(cloud_path), 'w') as f:
                field_names = [f.name for f in cloud.fields]
                field_sizes = [str(field_size(f.datatype)) for f in cloud.fields]
                field_types = [field_type_str(f.datatype) for f in cloud.fields]
                field_counts = [str(f.count) for f in cloud.fields]

                field_names = ' '.join(field_names)
                field_sizes = ' '.join(field_sizes)
                field_types = ' '.join(field_types)
                field_counts = ' '.join(field_counts)

                f.write('VERSION .7\n')
                f.write(f'FIELDS {field_names}\n')
                f.write(f'SIZE {field_sizes}\n')
                f.write(f'TYPE {field_types}\n')
                f.write(f'COUNT {field_counts}\n')
                f.write(f'WIDTH {cloud.width}\n')
                f.write(f'HEIGHT {cloud.height}\n')
                f.write('VIEWPOINT 0 0 0 1 0 0 0\n')
                n_points = cloud.width * cloud.height
                f.write(f'POINTS {n_points}\n')
                f.write('DATA ascii\n')

                # field_buffers = [field_buffer(cloud, f) for f in cloud.fields]
                for i in range(n_points):
                    offset = i * cloud.point_step
                    for field in cloud.fields:
                        val = np.frombuffer(cloud.data, count=1, offset=offset + field.offset,
                                            dtype=pcd_type_to_np_type(field.datatype))[0]
                        f.write(f'{val} ')
                    f.write('\n')
            idx += 1

# .PCD v.7 - Point Cloud Data file format
# VERSION .7
# FIELDS x y z
# SIZE 4 4 4 4
# TYPE F F F F
# COUNT 1 1 1 1
# WIDTH 213
# HEIGHT 1
# VIEWPOINT 0 0 0 1 0 0 0
# POINTS 213
# DATA ascii
# 0.93773 0.33763 0 4.2108e+06