#!/usr/bin/env python
# -*- coding: utf-8 -*-

'''
Author  : Heethesh Vhavle
Email   : heethesh@cmu.edu
Version : 1.0.0
Date    : Nov 30, 2019
'''

# Python 2/3 compatibility
from __future__ import print_function, absolute_import, division

# Handle OpenCV import
from import_cv2 import *

# External modules
# import open3d
import numpy as np
from pathlib import Path

# ROS modules
import rospy
import ros_numpy
from cv_bridge import CvBridge, CvBridgeError
from tf2_msgs.msg import TFMessage
from sensor_msgs import point_cloud2
from sensor_msgs.msg import Image, CameraInfo
from sensor_msgs.msg import PointCloud2, PointField
from geometry_msgs.msg import Vector3, Quaternion, Transform, TransformStamped
from tf.transformations import (translation_matrix,
                                quaternion_matrix,
                                concatenate_matrices,
                                euler_from_quaternion)

CV_BRIDGE = CvBridge()


def ros_time_from_nsecs(timestamp):
    return rospy.Time.from_sec(timestamp * 1e-9)


def make_image_message(img, header, coding='bgr8'):
    # Convert grayscale to RGB
    if len(img.shape) == 2:
        img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    # Create image using CV bridge
    msg = CV_BRIDGE.cv2_to_imgmsg(img, coding)
    msg.header = header
    return msg


def message_to_cv2(msg, coding='bgr8'):
    # Read image using CV bridge
    try:
        img = CV_BRIDGE.imgmsg_to_cv2(msg, coding)
        return img
    except CvBridgeError as e:
        print(e)
        rospy.logerr(e)
        return None


def open3d_to_numpy(pcd):
    return np.asarray(pcd.points)


def numpy_to_open3d(array):
    pcd = open3d.geometry.PointCloud()
    pcd.points = open3d.utility.Vector3dVector(array)
    return pcd


def make_pointcloud_message_ply(header, plydata):
    # Extract data from plydata
    x = plydata['vertex']['x']
    y = plydata['vertex']['y']
    z = plydata['vertex']['z']

    # Prepare point cloud data
    points = list(zip(x, y, z))
    fields = [PointField('x', 0, PointField.FLOAT32, 1),
              PointField('y', 4, PointField.FLOAT32, 1),
              PointField('z', 8, PointField.FLOAT32, 1)]

    # Create point cloud message
    msg = point_cloud2.create_cloud(header, fields, points)
    return msg


def make_pointcloud_message_numpy(header, array):
    # Create point cloud message
    return point_cloud2.create_cloud_xyz32(header, array)


def make_pointcloud_message_open3d(header, pcd):
    # Create point cloud message
    array = open3d_to_numpy(pcd)
    return point_cloud2.create_cloud_xyz32(header, array)


def pointcloud_message_to_numpy(msg):
    return ros_numpy.point_cloud2.pointcloud2_to_xyz_array(msg)


def pointcloud_message_to_open3d(msg):
    array = pointcloud_message_to_numpy(msg)
    pcd = numpy_to_open3d(array)
    return pcd


def homogenize_pointcloud(points):
    output = np.c_[points, np.ones(len(points)).reshape(-1, 1)]
    return output


def position_to_numpy(position):
    return np.asarray([position.x, position.y, position.z])


def orientation_to_numpy(orientation):
    return np.asarray([orientation.x, orientation.y, orientation.z, orientation.w])


def orientation_to_rpy(orientation):
    return euler_from_quaternion(orientation_to_numpy(orientation))


def quaternion_to_rpy(quaternion):
    return euler_from_quaternion(quaternion)


def make_vector3_message(vector):
    msg = Vector3()
    msg.x = vector[0]
    msg.y = vector[1]
    msg.z = vector[2]
    return msg


def make_quaternion_message(coefficients, to_xyzw=True):
    # Convert WXYZ to XYZW is flag set
    if to_xyzw: coefficients = np.roll(coefficients, -1)
    msg = Quaternion()
    msg.x = coefficients[0]
    msg.y = coefficients[1]
    msg.z = coefficients[2]
    msg.w = coefficients[3]
    return msg


def make_transform_stamped_message(parent_frame, child_frame, transform):
    msg = TransformStamped()
    msg.header.frame_id = parent_frame
    # msg.header.stamp is filled later while writing to bag file
    msg.child_frame_id = child_frame
    msg.transform.translation = make_vector3_message(
        transform['translation'])
    msg.transform.rotation = make_quaternion_message(
        transform['rotation']['coefficients'])
    return msg
