#!/usr/bin/env python3.6
# -*- coding: utf-8 -*-

'''
Author  : Heethesh Vhavle
Email   : heethesh@cmu.edu
Version : 1.0.0
Date    : Nov 30, 2019
'''

# Python 2/3 compatibility
from __future__ import print_function, absolute_import, division

# Built-in modules
import os
import json
import argparse

# External modules
import numpy as np
from tqdm import tqdm
from plyfile import PlyData, PlyElement

# Handle OpenCV import
from import_cv2 import *

# ROS modules
import rospy
import rosbag
from std_msgs.msg import Header
from tf2_msgs.msg import TFMessage

import utils
from argoverse.utils.json_utils import read_json_file
from argoverse.data_loading.argoverse_tracking_loader import ArgoverseTrackingLoader


class BagConverter:
    def __init__(self, args):
        # Setup Argoverse loader
        self.dataset_dir = args.dataset_dir
        self.log_id = args.log_id
        self.argoverse_loader = ArgoverseTrackingLoader(self.dataset_dir)
        self.argoverse_data = self.argoverse_loader.get(self.log_id)
        print(self.argoverse_data)

        # List of cameras to publish
        self.cameras_list = [camera for camera in self.argoverse_loader.CAMERA_LIST
                             if camera in args.cameras]
        self.load_camera_info()

        # Images timestamps
        self.image_files_dict = {
            camera: self.argoverse_data.timestamp_image_dict[camera]
            for camera in self.cameras_list}
        self.image_timestamps = {
            camera: list(self.image_files_dict[camera].keys())
            for camera in self.cameras_list}

        # Save image files dict to be added into the bag file later using Python2
        with open('/tmp/image_data_%s.json' % self.log_id, 'w') as f:
            data = {'image_files_dict': self.image_files_dict,
                    'image_timestamps': self.image_timestamps,
                    'cameras_list': self.cameras_list}
            json.dump(data, f, indent=4)

        # LiDAR timestamps
        self.lidar_files_dict = self.argoverse_data.timestamp_lidar_dict
        self.lidar_timestamps = list(self.lidar_files_dict.keys())

        # ROSBAG output path
        if not os.path.isdir(args.output_dir): os.makedirs(args.output_dir)
        self.output_filename = os.path.join(args.output_dir, '%s_no_images.bag' % self.log_id)
        self.bag = rosbag.Bag(self.output_filename, 'w')

        # Topic names
        self.lidar_topic = '/argoverse/lidar/pointcloud'
        self.camera_info_topic_template = '/argoverse/%s/camera_info'

        # TF frames
        self.map_frame = 'city'
        self.vehicle_frame = 'egovehicle'
        self.load_camera_static_tf()

    def load_camera_info(self):
        # Make a dictionary of CameraInfo messages for all cameras listed
        self.camera_info_dict = {
            camera: utils.make_camera_info_message(
                self.argoverse_data.get_calibration(camera))
            for camera in self.cameras_list}

    def load_camera_static_tf(self):
        # Make a dictionary of static TF messages for all cameras listed
        self.camera_static_tf_dict = {
            camera: utils.make_transform_stamped_message(self.vehicle_frame,
                camera, self.argoverse_data.get_calibration(
                    camera).calib_data['value']['vehicle_SE3_camera_'])
            for camera in self.argoverse_loader.CAMERA_LIST}

    def get_camera_info_message(self, camera, timestamp):
        # Make camera info message
        camera_info_msg = self.camera_info_dict[camera]
        camera_info_msg.header.stamp = utils.ros_time_from_nsecs(timestamp)
        return camera_info_msg

    def get_lidar_message(self, timestamp):
        # Load ply file
        filename = self.lidar_files_dict[timestamp]
        with open(filename, 'rb') as f:
            plydata = PlyData.read(f)

        # Make header message
        header = Header()
        header.frame_id = self.vehicle_frame  # LiDAR frame is ego vehicle frame
        header.stamp = utils.ros_time_from_nsecs(timestamp)

        # Make point cloud message (both LiDARs combined - 64 rings)
        pcl_msg = utils.make_pointcloud_message(header, plydata)
        return pcl_msg

    def convert(self):
        # Keep first timestamp for publishing static TF message
        static_timestamp = self.lidar_timestamps[0]

        # Publish LiDAR PointCloud messages
        for timestamp in tqdm(self.lidar_timestamps, desc='LiDAR, Pose'):
            lidar_msg = self.get_lidar_message(timestamp)
            self.bag.write(self.lidar_topic, lidar_msg,
                utils.ros_time_from_nsecs(timestamp))

            # Publish ego vehicle pose
            tf_msg = utils.argoverse_pose_to_transform_message(self.dataset_dir,
                self.log_id, self.map_frame, self.vehicle_frame, timestamp, read_json_file)
            self.bag.write('/tf', tf_msg, utils.ros_time_from_nsecs(timestamp))

        # Publish camera messages
        for camera in tqdm(self.cameras_list, desc='CameraInfo'):
            static_timestamp = min(static_timestamp, self.image_timestamps[camera][0])
            for timestamp in tqdm(self.image_timestamps[camera], desc=camera, leave=False):
                camera_info_msg = self.get_camera_info_message(camera, timestamp)
                self.bag.write(self.camera_info_topic_template % camera, camera_info_msg,
                    utils.ros_time_from_nsecs(timestamp))

        # Publish static TF messages
        tf_msg = TFMessage()
        for camera in tqdm(self.camera_static_tf_dict, desc='Static TF'):
            msg = self.camera_static_tf_dict[camera]
            msg.header.stamp = utils.ros_time_from_nsecs(static_timestamp)
            tf_msg.transforms.append(msg)
        self.bag.write('/tf_static', tf_msg, utils.ros_time_from_nsecs(static_timestamp))

        # Close rosbag file
        self.bag.close()


if __name__ == '__main__':
    # Argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--dataset_dir', type=str, required=True,
        help='Path to dataset directory containing the logs')
    parser.add_argument('--log_id', type=str, required=True,
        help='Argoverse sequence log ID')
    parser.add_argument('--output_dir', type=str, required=True,
        help='Bag file output directory')
    parser.add_argument('--cameras', nargs='+', default=[],
        help='List of cameras to add to the bag file.\
        Cameras available:\
        ring_front_center,\
        ring_front_left,\
        ring_front_right,\
        ring_rear_left,\
        ring_rear_right,\
        ring_side_left,\
        ring_side_right,\
        stereo_front_left,\
        stereo_front_right]')
    args = parser.parse_args()
    args.cameras = sum([camera.split() for camera in args.cameras], [])

    # Start conversion
    bag_converter = BagConverter(args)
    bag_converter.convert()
