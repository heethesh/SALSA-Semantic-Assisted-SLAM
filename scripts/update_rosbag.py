#!/usr/bin/env python
# -*- coding: utf-8 -*-
# @Author: Heethesh Vhavle
# @Date:   Nov 20, 2019
# @Last Modified by:   Heethesh Vhavle
# @Last Modified time: Apr 26, 2020

# Python 2/3 compatibility
from __future__ import print_function, absolute_import, division

import os
import sys
import argparse

import cv2
import numpy as np
from tqdm import tqdm

import rospy
import rosbag
from std_msgs.msg import Header

import utils


class BagConverter:
    def __init__(self, args):
        self.args = args
        self.bag = rosbag.Bag(args.input_bag)
        self.new_bag = rosbag.Bag(args.output_bag, 'w')

        if args.dataset == 'open-loris':
            self.image_topic = '/d400/color/image_raw'
            self.scores_topic = '/d400/annotation/dynamic_scores'
            self.topics = [
                '/d400/accel/imu_info',
                '/d400/accel/sample',
                '/d400/aligned_depth_to_color/camera_info',
                '/d400/aligned_depth_to_color/image_raw',
                '/d400/color/camera_info',
                '/d400/color/image_raw',
                '/d400/depth/camera_info',
                '/d400/depth/image_raw',
                '/d400/gyro/imu_info',
                '/d400/gyro/sample',
                '/gt',
                '/odom',
                '/tf_static',
            ]

        elif args.dataset == 'tum-rgbd':
            self.image_topic = '/camera/rgb/image_color'
            self.scores_topic = '/camera/annotation/dynamic_scores'
            self.topics = [
                '/camera/depth/camera_info',
                '/camera/depth/image',
                '/camera/rgb/camera_info',
                '/camera/rgb/image_color',
                '/cortex_marker_array',
                '/tf',
            ]

        else:
            print('Dataset not supported')
            sys.exit(1)

        self.copy_topics()

        self.image_count = self.bag.get_message_count(self.image_topic)
        print('Found %d image messages' % self.image_count)

    def copy_topics(self):
        print('Copying the following topics to the new bag file:', self.topics)
        for topic, msg, timestamp in self.bag.read_messages():
            if topic in self.topics:
                self.new_bag.write(topic, msg, timestamp)
        print('Done!\n')

    def update(self, images_dict):
        print('Adding new topic %s ...' % self.scores_topic)

        assert len(images_dict.keys()) == self.image_count, \
            'Image count does not match!'

        count = 0
        for topic, msg, timestamp in self.bag.read_messages():
            if topic == self.image_topic:
                sys.stdout.write('\rProcessing %05d/%05d...' % (count + 1, self.image_count))
                sys.stdout.flush()

                dict_timestamp = str(timestamp.to_nsec())
                assert dict_timestamp in images_dict, \
                    'Corresponding timestamp image not found!'

                filename, frame_count = images_dict[dict_timestamp]
                assert frame_count == count, 'Frame count does not match!'

                header = Header()
                header.frame_id = msg.header.frame_id
                header.stamp = timestamp

                img = cv2.imread(filename)
                image_msg = utils.make_image_message(img, header)
                self.new_bag.write(self.scores_topic, image_msg, timestamp)
                count += 1

        print('\nDone!')

    def close(self):
        self.bag.close()
        self.new_bag.close()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Add images to a ROS bag.')
    parser.add_argument('--input-bag', help='Input ROS bag.')
    parser.add_argument('--output-bag', help='Input ROS bag.')
    parser.add_argument('--image-dir', help='Annotated images directory.')
    parser.add_argument('--dataset', help='Options: open-loris | tum-rgbd.')
    args = parser.parse_args()

    files = os.listdir(args.image_dir)
    images_dict = {
        file.split('-')[-1].split('.')[0]:
        [os.path.join(args.image_dir, file), int(file.split('-')[-2])]
        for file in files
    }

    bag_converter = BagConverter(args)
    bag_converter.update(images_dict)
    bag_converter.close()
