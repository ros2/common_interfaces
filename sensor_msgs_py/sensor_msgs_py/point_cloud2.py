# Copyright 2008 Willow Garage, Inc.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the Willow Garage, Inc. nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.


"""
Serialization of sensor_msgs.PointCloud2 messages.

Author: Tim Field
ROS 2 port by Sebastian Grans
File originally ported from:
https://github.com/ros/common_msgs/blob/f48b00d43cdb82ed9367e0956db332484f676598/
sensor_msgs/src/sensor_msgs/point_cloud2.py

Author: Dheera Venkatraman
numpy version of read_points / read_points_list
"""

import ctypes
import numpy as np
import struct
import sys

from sensor_msgs.msg import PointCloud2, PointField


_DATATYPES = {}
_DATATYPES[PointField.INT8] = ('b', 1)
_DATATYPES[PointField.UINT8] = ('B', 1)
_DATATYPES[PointField.INT16] = ('h', 2)
_DATATYPES[PointField.UINT16] = ('H', 2)
_DATATYPES[PointField.INT32] = ('i', 4)
_DATATYPES[PointField.UINT32] = ('I', 4)
_DATATYPES[PointField.FLOAT32] = ('f', 4)
_DATATYPES[PointField.FLOAT64] = ('d', 8)

_DATATYPES_NUMPY_MAP = {
    PointField.INT8: np.int8,
    PointField.UINT8: np.uint8,
    PointField.INT16: np.int16,
    PointField.UINT16: np.uint16,
    PointField.INT32: np.int32,
    PointField.UINT32: np.uint32,
    PointField.FLOAT32: np.float32,
    PointField.FLOAT64: np.float64,
}


def read_points(cloud, field_names=None, skip_nans=False, uvs=[]):
    """
    Read points from a sensor_msgs.PointCloud2 message.

    :param cloud: The point cloud to read from sensor_msgs.PointCloud2.
    :param field_names: The names of fields to read. If None, read all fields.
                        (Type: Iterable, Default: None)
    :param skip_nans: If True, then don't return any point with a NaN value.
                      (Type: Bool, Default: False)
    :param uvs: If specified, then only return the points at the given
        coordinates. (Type: Iterable, Default: empty list)
    :return: An iterable for values for each point. For more efficient access use
    read_points_list directly.
    """
    return iter(read_points_list(cloud, field_names=field_names, skip_nans=skip_nans, uvs=[]))


def read_points_list(cloud, field_names=None, skip_nans=False, uvs=[]):
    """
    Read points from a sensor_msgs.PointCloud2 message.

    :param cloud: The point cloud to read from sensor_msgs.PointCloud2.
    :param field_names: The names of fields to read. If None, read all fields.
                        (Type: Iterable, Default: None)
    :param skip_nans: If True, then don't return any point with a NaN value.
                      (Type: Bool, Default: False)
    :param uvs: If specified, then only return the points at the given
        coordinates. (Type: Iterable, Default: empty list)
    :return: numpy.recarray with values for each point.
    """
    assert isinstance(cloud, PointCloud2), \
        'cloud is not a sensor_msgs.msg.PointCloud2'

    assert cloud.point_step * cloud.width * cloud.height == len(cloud.data), \
        'length of data does not match point_step * width * height'

    all_field_names = []
    np_struct = []
    total_used_bytes = 0
    for field in cloud.fields:
        all_field_names.append(field.name)
        assert field.datatype in _DATATYPES_NUMPY_MAP, \
            'invalid datatype %d specified for field %s' % (field.datatype, field.name)
        field_np_datatype = _DATATYPES_NUMPY_MAP[field.datatype]
        np_struct.append((field.name, field_np_datatype))
        total_used_bytes += np.nbytes[field_np_datatype]

    assert cloud.point_step >= total_used_bytes, \
        'error: total byte sizes of fields exceeds point_step'

    if cloud.point_step > total_used_bytes:
        np_struct.append(('unused_bytes', np.uint8, cloud.point_step - total_used_bytes))

    points = np.frombuffer(cloud.data, dtype=np_struct).view(dtype=np.recarray)

    if skip_nans:
        nan_indexes = None
        for field_name in all_field_names:
            if nan_indexes is None:
                nan_indexes = np.isnan(points[field_name])
            else:
                nan_indexes = nan_indexes | np.isnan(points[field_name])

        points = points[~nan_indexes]

    if uvs:
        fetch_indexes = [(v * cloud.width + u) for u, v in uvs]
        points = points[fetch_indexes]

    # if endianness between cloud and system doesn't match then byteswap everything
    if cloud.is_bigendian == np.little_endian:
        points = points.byteswap()

    if field_names is None:
        return points
    else:
        return points[list(field_names)]


def create_cloud(header, fields, points):
    """
    Create a sensor_msgs.msg.PointCloud2 message.

    :param header: The point cloud header. (Type: std_msgs.msg.Header)
    :param fields: The point cloud fields.
                   (Type: iterable of sensor_msgs.msg.PointField)
    :param points: The point cloud points. List of iterables, i.e. one iterable
                   for each point, with the elements of each iterable being the
                   values of the fields for that point (in the same order as
                   the fields parameter)
    :return: The point cloud as sensor_msgs.msg.PointCloud2
    """
    cloud_struct = struct.Struct(_get_struct_fmt(False, fields))

    buff = ctypes.create_string_buffer(cloud_struct.size * len(points))

    point_step, pack_into = cloud_struct.size, cloud_struct.pack_into
    offset = 0
    for p in points:
        pack_into(buff, offset, *p)
        offset += point_step

    return PointCloud2(header=header,
                       height=1,
                       width=len(points),
                       is_dense=False,
                       is_bigendian=False,
                       fields=fields,
                       point_step=cloud_struct.size,
                       row_step=cloud_struct.size * len(points),
                       data=buff.raw)


def create_cloud_xyz32(header, points):
    """
    Create a sensor_msgs.msg.PointCloud2 message with (x, y, z) fields.

    :param header: The point cloud header. (Type: std_msgs.msg.Header)
    :param points: The point cloud points. (Type: Iterable)
    :return: The point cloud as sensor_msgs.msg.PointCloud2.
    """
    fields = [PointField(name='x', offset=0,
                         datatype=PointField.FLOAT32, count=1),
              PointField(name='y', offset=4,
                         datatype=PointField.FLOAT32, count=1),
              PointField(name='z', offset=8,
                         datatype=PointField.FLOAT32, count=1)]
    return create_cloud(header, fields, points)


def _get_struct_fmt(is_bigendian, fields, field_names=None):
    fmt = '>' if is_bigendian else '<'

    offset = 0
    for field in (f for f in sorted(fields, key=lambda f: f.offset)
                  if field_names is None or f.name in field_names):
        if offset < field.offset:
            fmt += 'x' * (field.offset - offset)
            offset = field.offset
        if field.datatype not in _DATATYPES:
            print('Skipping unknown PointField datatype [%d]' %
                  field.datatype, file=sys.stderr)
        else:
            datatype_fmt, datatype_length = _DATATYPES[field.datatype]
            fmt += field.count * datatype_fmt
            offset += field.count * datatype_length

    return fmt
