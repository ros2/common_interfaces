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


import unittest

import numpy as np

from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header


pylist = [[0.0, 0.1, 0.2],
          [1.0, 1.1, 1.2],
          [2.0, 2.1, 2.2],
          [3.0, 3.1, 3.2],
          [4.0, np.nan, 4.2]]
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
    is_bigendian=False,  # Not sure how to properly determine this.
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
    is_bigendian=False,  # Not sure how to properly determine this.
    fields=fields,
    point_step=(itemsize * 2),  # Every point consists of three float32s.
    row_step=(itemsize * 2 * points.shape[0]),
    data=data
)


class TestPointCloud2Methods(unittest.TestCase):

    def test_read_points(self):
        # Test that converting a PointCloud2 to a list, is equivalent to
        # the original list of points.
        pcd_list = list(point_cloud2.read_points(pcd))
        self.assertTrue(np.allclose(pcd_list, pylist, equal_nan=True))

    def test_read_points_field(self):
        # Test that field selection is working.
        pcd_list = list(point_cloud2.read_points(pcd, field_names=['x', 'z']))
        # Check correct shape.
        self.assertTrue(np.array(pcd_list).shape == points[:, [0, 2]].shape)
        # Check "correct" values.
        self.assertTrue(np.allclose(pcd_list, points[:, [0, 2]],
                                    equal_nan=True))

    def test_read_points_skip_nan(self):
        # Test that removing NaNs work.
        pcd_list = list(point_cloud2.read_points(pcd, skip_nans=True))
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
        self.assertTrue(np.allclose(np.array(points_named), points, equal_nan=True))

    def test_create_cloud(self):
        thispcd = point_cloud2.create_cloud(Header(frame_id='frame'),
                                            fields, pylist)
        self.assertTrue(thispcd == pcd)
        thispcd = point_cloud2.create_cloud(Header(frame_id='frame2'),
                                            fields, pylist)
        self.assertFalse(thispcd == pcd)
        thispcd = point_cloud2.create_cloud(Header(frame_id='frame'),
                                            fields2, pylist2)
        self.assertFalse(thispcd == pcd)

    def test_create_cloud_xyz32(self):
        thispcd = point_cloud2.create_cloud_xyz32(Header(frame_id='frame'),
                                                  pylist)
        self.assertTrue(thispcd == pcd)


if __name__ == '__main__':
    unittest.main()
