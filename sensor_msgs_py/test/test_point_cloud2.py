# Copyright 2020 Willow Garage, Inc.
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


import sys
import unittest

import numpy as np
try:
    from numpy.lib.recfunctions import structured_to_unstructured, unstructured_to_structured
except ImportError:
    from sensor_msgs_py.numpy_compat import (structured_to_unstructured,
                                             unstructured_to_structured)

from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header

pylist = [(0.0, 0.1, 0.2),
          (1.0, 1.1, 1.2),
          (2.0, 2.1, 2.2),
          (3.0, 3.1, 3.2),
          (4.0, np.nan, 4.2),
          (5.0, 5.1, 5.2)]
points = np.array(pylist, dtype=np.float32)

fields = [PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
          PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
          PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)]

itemsize = points.itemsize
data = points.tobytes()

# 3D (xyz) point cloud (nx3)
pcd = PointCloud2(
    header=Header(frame_id='frame'),
    height=1,
    width=points.shape[0],
    is_dense=False,
    is_bigendian=sys.byteorder != 'little',
    fields=fields,
    point_step=(itemsize * 3),  # Every point consists of three float32s.
    row_step=(itemsize * 3 * points.shape[0]),
    data=data
)


# 2D (yz) point cloud
fields2 = [PointField(name='y', offset=0,
                      datatype=PointField.FLOAT32, count=1),
           PointField(name='z', offset=4,
                      datatype=PointField.FLOAT32, count=1)]
pylist2 = points[:, 1:].tolist()  # y and z column.
data2 = points[:, 1:].tobytes()
pcd2 = PointCloud2(
    header=Header(frame_id='frame'),
    height=1,
    width=points.shape[0],
    is_dense=False,
    is_bigendian=sys.byteorder != 'little',
    fields=fields,
    point_step=(itemsize * 2),  # Every point consists of two float32s.
    row_step=(itemsize * 2 * points.shape[0]),
    data=data
)

# Organized point cloud (Points are aligned in 2D matrix)
points3 = points.reshape(2, 3, -1)
pcd3 = PointCloud2(
    header=Header(frame_id='frame'),
    height=points3.shape[1],
    width=points3.shape[0],
    is_dense=False,
    is_bigendian=sys.byteorder != 'little',
    fields=fields,
    # Every point consists of three float32s.
    point_step=(itemsize * points3.shape[-1]),
    row_step=(itemsize * points3.shape[-1] * points3.shape[0]),
    data=points3.tobytes()
)

# Check multiple datatype pointclouds
struct_points = np.array(
    # Make each point a tuple
    list(map(tuple, points)),
    dtype=[
        ('a', np.float32),
        ('b', np.float64),
        ('c', np.uint8),
    ])

struct_points_itemsize = struct_points.itemsize

struct_points_fields = [
    PointField(name='a', offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name='b', offset=4, datatype=PointField.FLOAT64, count=1),
    PointField(name='c', offset=12, datatype=PointField.UINT8, count=1)]

pcd4 = PointCloud2(
    header=Header(frame_id='frame'),
    height=1,
    width=struct_points.shape[0],
    is_dense=False,
    is_bigendian=sys.byteorder != 'little',
    fields=struct_points_fields,
    # This time a struct array is used.
    # The itemsize therfore represents the size of a complete point
    point_step=struct_points_itemsize,
    row_step=(struct_points_itemsize * points.shape[0]),
    data=struct_points.tobytes()
)

# Point cloud with a field with count > 1
count = 3
fields5 = [PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=3)]

itemsize = points.itemsize * count
data = points.tobytes()

pcd5 = PointCloud2(
    header=Header(frame_id='frame'),
    height=1,
    width=points.shape[0],
    is_dense=False,
    is_bigendian=sys.byteorder != 'little',
    fields=fields5,
    point_step=(itemsize),
    row_step=(itemsize * points.shape[0]),
    data=data
)

# Point cloud with two matching data-types, and one dissimilar
struct_points_6 = np.array(
    # Make each point a tuple
    list(map(tuple, points)),
    dtype=[
        ('a', np.float32),
        ('b', np.float32),
        ('c', np.uint8),
    ])

struct_points_itemsize_6 = struct_points_6.itemsize

struct_points_fields_6 = [
    PointField(name='a', offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name='b', offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name='c', offset=8, datatype=PointField.UINT8, count=1)]

pcd6 = PointCloud2(
    header=Header(frame_id='frame'),
    height=1,
    width=struct_points_6.shape[0],
    is_dense=False,
    is_bigendian=sys.byteorder != 'little',
    fields=struct_points_fields_6,
    point_step=struct_points_itemsize_6,
    row_step=(struct_points_itemsize_6 * points.shape[0]),
    data=struct_points_6.tobytes()
)

# End padding in data layout
# Itemsize has 20 bytes rather than 12
itemsize_end_padding = 20
dtype_end_padding = np.dtype({'names': ['x', 'y', 'z'],
                              'formats': ['f4', 'f4', 'f4'],
                              'offsets': [0, 4, 8],
                              'itemsize': itemsize_end_padding})
points_end_padding = np.array(pylist, dtype=dtype_end_padding)


class TestPointCloud2Methods(unittest.TestCase):

    def test_read_points(self):
        # Test that converting a PointCloud2 to a list, is equivalent to
        # the original list of points.
        pcd_list = list(map(list, point_cloud2.read_points(pcd)))
        self.assertTrue(np.allclose(pcd_list, pylist, equal_nan=True))

    def test_read_points_field(self):
        # Test that field selection is working.
        pcd_list = list(map(
            list,
            point_cloud2.read_points(pcd, field_names=['x', 'z'])))
        # Check correct shape.
        self.assertTrue(np.array(pcd_list).shape == points[:, [0, 2]].shape)
        # Check "correct" values.
        self.assertTrue(np.allclose(pcd_list, points[:, [0, 2]],
                                    equal_nan=True))

    def test_read_points_skip_nan(self):
        # Test that removing NaNs work.
        pcd_list = list(map(
            list,
            point_cloud2.read_points(pcd, skip_nans=True)))
        points_nonan = points[~np.any(np.isnan(points), axis=1)]
        # Check correct shape
        self.assertTrue(np.array(pcd_list).shape == points_nonan.shape)
        # Check correct values.
        # We do not expect NaNs, so I explicitly state that NaNs aren't
        # considered equal. (This is the default behavious of `allclose()`)
        self.assertTrue(np.allclose(pcd_list, points_nonan, equal_nan=False))

    def test_read_points_list(self):
        # Check that reading a PointCloud2 message to a list is performed
        # correctly.
        points_named = point_cloud2.read_points_list(pcd)
        self.assertTrue(np.allclose(
            np.array(points_named), points, equal_nan=True))

    def test_read_points_organized(self):
        # Checks if organized clouds are handled correctly
        # Test if it is converted into a unorganized one by default
        pcd_list = list(map(list, point_cloud2.read_points(pcd3)))
        self.assertTrue(np.allclose(pcd_list, pylist, equal_nan=True))
        # Test if organization is correctly if requested
        pcd_points = point_cloud2.read_points(
            pcd3, reshape_organized_cloud=True)
        # Because we have a 2d array of points now it is easier to cat it into a
        # unstructured NumPy array instead of converting it into a list of lists
        pcd_points = structured_to_unstructured(pcd_points)
        self.assertTrue(np.allclose(pcd_points, points3, equal_nan=True))

    def test_read_points_numpy(self):
        # Checks if the deserialization to an unstructured numpy array works
        pcd_points = point_cloud2.read_points_numpy(pcd)
        self.assertTrue(np.allclose(pcd_points, points, equal_nan=True))

    def test_read_points_numpy_same_types(self):
        # Checks if the deserialization to an unstructured numpy array throws
        # assertion exception when data types are not uniform
        with self.assertRaises(AssertionError):
            point_cloud2.read_points_numpy(pcd4)

    def test_read_points_numpy_specific_fields(self):
        # Checks if the deserialization to an unstructured numpy array works
        # when only selecting certain fields
        pcd_points = point_cloud2.read_points_numpy(pcd6, field_names=('a', 'b'))
        self.assertTrue(pcd_points.shape == (6, 2))

    def test_read_points_different_types(self):
        # Checks if the deserialization to an unstructured numpy array works
        pcd_points = point_cloud2.read_points(pcd4)
        self.assertTrue(
            all(np.allclose(pcd_points[name], struct_points[name], equal_nan=True)
                for name in struct_points.dtype.names))
        self.assertEqual(struct_points.dtype, pcd_points.dtype)

    def test_read_points_non_one_count(self):
        pcd_points = point_cloud2.read_points_numpy(pcd5)
        self.assertTrue(
            np.allclose(pcd_points, points, equal_nan=True))

    def test_read_points_non_one_count_structured(self):
        pcd_points = point_cloud2.read_points(pcd5)
        pcd_points_unstructured = structured_to_unstructured(pcd_points)
        self.assertTrue(
            np.allclose(pcd_points_unstructured, points, equal_nan=True))
        self.assertEqual(pcd_points.dtype.names, ('x_0', 'x_1', 'x_2'))

    def test_create_cloud(self):
        thispcd = point_cloud2.create_cloud(Header(frame_id='frame'),
                                            fields, pylist)
        self.assertEqual(thispcd, pcd)
        thispcd = point_cloud2.create_cloud(Header(frame_id='frame2'),
                                            fields, pylist)
        self.assertFalse(thispcd == pcd)
        thispcd = point_cloud2.create_cloud(Header(frame_id='frame'),
                                            fields2, pylist2)
        self.assertFalse(thispcd == pcd)

    def test_create_cloud_itemsize(self):

        with self.assertRaises(AssertionError):
            point_cloud2.create_cloud(
                Header(frame_id='frame'),
                fields,
                points_end_padding)

        thispcd = point_cloud2.create_cloud(
            Header(frame_id='frame'),
            fields,
            points_end_padding,
            points_end_padding.dtype.itemsize)
        self.assertEqual(thispcd.point_step, itemsize_end_padding)

    def test_create_cloud_different_types(self):
        # Check if we are able to create a point cloud with different data
        thispcd = point_cloud2.create_cloud(
            Header(frame_id='frame'),
            struct_points_fields,
            struct_points)
        self.assertEqual(thispcd, pcd4)

    def test_create_cloud_xyz32(self):
        thispcd = point_cloud2.create_cloud_xyz32(
            Header(frame_id='frame'),
            pylist)
        self.assertEqual(thispcd, pcd)

    def test_create_cloud_xyz32_organized(self):
        # Checks if organized clouds are handled correctly
        thispcd = point_cloud2.create_cloud_xyz32(
            Header(frame_id='frame'),
            points3)
        self.assertEqual(thispcd, pcd3)

    def test_create_cloud__non_one_count(self):
        thispcd = point_cloud2.create_cloud(
            Header(frame_id='frame'),
            fields5,
            points)
        self.assertEqual(thispcd, pcd5)

    def test_read_cloud_with_non_standard_point_step(self):
        itemsize = 123  # Larger than normal point step size

        # Copy to new array with larger itemsize
        points_larger_itemsize = np.array(
            unstructured_to_structured(points),
            dtype=np.dtype({
                'names': ['x', 'y', 'z'],
                'formats': ['<f4', '<f4', '<f4'],
                'offsets': [0, 4, 8],
                'itemsize': itemsize
            })
        )

        # Create pointcloud with itemsize == point_step from the padded array
        pc = PointCloud2(
            header=Header(frame_id='frame'),
            height=1,
            width=points_larger_itemsize.shape[0],
            is_dense=False,
            is_bigendian=sys.byteorder != 'little',
            fields=fields,
            point_step=itemsize,
            row_step=itemsize * points_larger_itemsize.shape[0],
            data=points_larger_itemsize.tobytes()
        )

        # Deserialize point cloud
        reconstructed_points = point_cloud2.read_points_numpy(pc)
        self.assertTrue(np.allclose(points, reconstructed_points, equal_nan=True))


if __name__ == '__main__':
    unittest.main()
