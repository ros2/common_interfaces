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
"""

import sys
from collections import namedtuple
from typing import Iterable, List, Optional, NamedTuple

import numpy as np
from numpy.lib.recfunctions import structured_to_unstructured
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header

_DATATYPES = {}
_DATATYPES[PointField.INT8] = (np.int8, 8)
_DATATYPES[PointField.UINT8] = (np.uint8, 8)
_DATATYPES[PointField.INT16] = (np.int16, 16)
_DATATYPES[PointField.UINT16] = (np.uint16, 16)
_DATATYPES[PointField.INT32] = (np.int32, 32)
_DATATYPES[PointField.UINT32] = (np.uint32, 32)
_DATATYPES[PointField.FLOAT32] = (np.float32, 32)
_DATATYPES[PointField.FLOAT64] = (np.float64, 64)


def read_points(
        cloud: PointCloud2,
        field_names: Optional[List[str]] = None,
        skip_nans: bool = False,
        uvs: Optional[Iterable] = None,
        reshape_organized_cloud: bool = False) -> np.ndarray:
    """
    Read points from a sensor_msgs.PointCloud2 message.

    :param cloud: The point cloud to read from sensor_msgs.PointCloud2.
    :param field_names: The names of fields to read. If None, read all fields.
                        (Type: Iterable, Default: None)
    :param skip_nans: If True, then don't return any point with a NaN value.
                      (Type: Bool, Default: False)
    :param uvs: If specified, then only return the points at the given
        coordinates. (Type: Iterable, Default: None)
    :return: Structured NumPy array containing all points.
    """
    assert isinstance(cloud, PointCloud2), \
        'Cloud is not a sensor_msgs.msg.PointCloud2'

    # Create a list containing the names of all fields
    all_field_names = []
    for i, field in enumerate(cloud.fields):
        if field.name == "":
            name = f"unnamed_field_{i}"
        else:
            name = field.name
        assert not name in all_field_names, "Duplicate field names are not allowed!"
        all_field_names.append(name)

    # Create a tuple for each field containing name and data type
    dtype = np.dtype({
        'names': all_field_names,
        'formats': [np.dtype(_DATATYPES[field.datatype][0]).str for field in cloud.fields],
        'offsets': [field.offset for field in cloud.fields],
    })

    # Cast bytes to numpy array
    points = np.ndarray(
        shape=(cloud.width * cloud.height, ),
        dtype=dtype,
        buffer=cloud.data)

    # Keep the only requested fields
    if field_names is not None:
        assert all(field_name in all_field_names for field_name in field_names), \
            "Requests field is not in the fields of the PointCloud!"
        # Mask fields
        points = points[list(field_names)]

    # Swap array if byte order does not match
    if bool(sys.byteorder != "little") != bool(cloud.is_bigendian):
        points = points.byteswap(inplace=True)

    # Check if we want to drop points with nan values
    if skip_nans and not cloud.is_dense:
        # Init mask which selects all points
        not_nan_mask = np.ones(len(points), dtype=np.bool)
        for field_name in points.dtype.names:
            # Only keep points without any non values in the mask
            not_nan_mask = np.logical_and(not_nan_mask, ~np.isnan(points[field_name]))
        # Select these points
        points = points[not_nan_mask]

    # Select points indexed by the uvs field
    if uvs is not None:
        # Don't convert to numpy array if it is already one
        if not isinstance(uvs, np.ndarray):
            uvs = np.fromiter(uvs, int)
        # Index requested points
        points = points[uvs]

    # Cast into 2d array if cloud is 'organized'
    if reshape_organized_cloud and cloud.height > 1:
        points = points.reshape(cloud.height, cloud.width)

    return points


def read_points_numpy(
        cloud: PointCloud2,
        field_names: Optional[List[str]] = None,
        skip_nans: bool = False,
        uvs: Optional[Iterable] = None,
        reshape_organized_cloud: bool = False) -> np.ndarray:
    """
    Read equally typed fields from  sensor_msgs.PointCloud2 message
    as a unstructured numpy array.

    This method is better suited if one wants to perform build math operations
    on e.g. all x,y,z fields.
    But it is limited to fields with the same dtype as unstructured numpy arrays
    only contain one dtype.

    :param cloud: The point cloud to read from sensor_msgs.PointCloud2.
    :param field_names: The names of fields to read. If None, read all fields.
                        (Type: Iterable, Default: None)
    :param skip_nans: If True, then don't return any point with a NaN value.
                      (Type: Bool, Default: False)
    :param uvs: If specified, then only return the points at the given
        coordinates. (Type: Iterable, Default: None)
    :return: Numpy array containing all points.
    """
    assert all(cloud.fields[0].datatype == field.datatype for field in cloud.fields[1:]), \
        "All fields need to have the same datatype. Use `read_points()` otherwise."
    structured_numpy_array = read_points(cloud, field_names, skip_nans, uvs, reshape_organized_cloud)
    return structured_to_unstructured(structured_numpy_array)


def read_points_list(
        cloud: PointCloud2,
        field_names: Optional[List[str]] = None,
        skip_nans: bool = False,
        uvs: Optional[Iterable] = None) -> List[NamedTuple]:
    """
    Read points from a sensor_msgs.PointCloud2 message.

    This function returns a list of namedtuples. It operates on top of the
    read_points method. For more efficient access use read_points directly.

    :param cloud: The point cloud to read from. (Type: sensor_msgs.PointCloud2)
    :param field_names: The names of fields to read. If None, read all fields.
                        (Type: Iterable, Default: None)
    :param skip_nans: If True, then don't return any point with a NaN value.
                      (Type: Bool, Default: False)
    :param uvs: If specified, then only return the points at the given
                coordinates. (Type: Iterable, Default: None]
    :return: List of namedtuples containing the values for each point
    """
    assert isinstance(cloud, PointCloud2), \
        'cloud is not a sensor_msgs.msg.PointCloud2'

    if field_names is None:
        field_names = [f.name for f in cloud.fields]

    Point = namedtuple('Point', field_names)

    return [Point._make(p) for p in read_points(cloud, field_names,
                                                skip_nans, uvs)]


def create_cloud(
        header: Header,
        fields: List[PointField],
        points: Iterable) -> PointCloud2:
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
    # Convert to numpy if it isn't already
    points = np.asarray(points, dtype=_DATATYPES[fields[0].datatype][0])
    # Convert data to f32, flatten it and cat it to a byte string
    data = points.reshape(-1).tobytes()
    # Define offsets
    point_value_bits = _DATATYPES[fields[0].datatype][1]
    point_num_values = len(fields)
    point_bytes = (point_num_values * point_value_bits) // 8 # Bytes used by one point
    # Put everything together
    return PointCloud2(
        header = header,
        height = 1,
        width = len(points),
        is_dense = False,
        is_bigendian = False,
        fields = fields,
        point_step = point_bytes,
        row_step = point_bytes * len(points),
        data = data)


def create_cloud_xyz32(header: Header, points: Iterable) -> PointCloud2:
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
