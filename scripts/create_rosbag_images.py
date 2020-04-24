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

# Built-in modules
import os
import json
import argparse

# External modules
import numpy as np
from tqdm import tqdm

# Handle OpenCV import
from import_cv2 import *

# ROS modules
import rospy
import rosbag
from std_msgs.msg import Header

import utils


class BagImageConverter:
    def __init__(self, args):
        # ROSBAG output path
        self.log_id = args.log_id
        if not os.path.isdir(args.output_dir): os.makedirs(args.output_dir)
        self.inter_bag_file = os.path.join(args.output_dir, '%s_no_images.bag' % self.log_id)
        self.inter_bag = rosbag.Bag(self.inter_bag_file)
        self.output_filename = os.path.join(args.output_dir, '%s.bag' % self.log_id)
        self.bag = rosbag.Bag(self.output_filename, 'w')

        # Load image data
        with open('/tmp/image_data_%s.json' % self.log_id, 'r') as f:
            data = json.load(f)
        self.image_files_dict = data['image_files_dict']
        self.image_timestamps = data['image_timestamps']
        self.cameras_list = data['cameras_list']

        # Topic names
        self.image_topic_template = '/argoverse/%s/image_rect'

        # Copy all topics to final bag file
        self.copy_topics()

    def copy_topics(self):
        # Copy all topics to final bag file
        print('Copying intermediate messages to final bag file...')
        for topic, msg, timestamp in self.inter_bag.read_messages():
            self.bag.write(topic, msg, timestamp)

    def get_image_message(self, camera, timestamp):
        # Make header message
        header = Header()
        header.frame_id = camera
        header.stamp = utils.ros_time_from_nsecs(timestamp)

        # Make image message
        img = cv2.imread(self.image_files_dict[camera][str(timestamp)])
        # img = cv2.resize(img, None, fx=0.2, fy=0.2)
        image_msg = utils.make_image_message(img, header)

        return image_msg

    def convert(self):
        # Publish camera messages
        for camera in tqdm(self.cameras_list, desc='Images'):
            for timestamp in tqdm(self.image_timestamps[camera], desc=camera, leave=False):
                image_msg = self.get_image_message(camera, timestamp)
                self.bag.write(self.image_topic_template % camera, image_msg,
                    utils.ros_time_from_nsecs(timestamp))

        # Close rosbag file
        self.inter_bag.close()
        self.bag.close()
        print('\nBag file saved to', self.output_filename)


if __name__ == '__main__':
    # Argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--log_id', type=str, required=True,
        help='Argoverse sequence log ID')
    parser.add_argument('--output_dir', type=str, required=True,
        help='Bag file output directory')
    args = parser.parse_args()

    # Start conversion
    bag_converter = BagImageConverter(args)
    bag_converter.convert()
