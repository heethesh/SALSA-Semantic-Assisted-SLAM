#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Copyright 2016 Massachusetts Institute of Technology

'''
Extract images from a rosbag.
'''

import os
import sys
import argparse

import cv2

import rosbag
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


def main():
    '''
    Extract a folder of images from a rosbag.
    '''
    parser = argparse.ArgumentParser(description='Extract images from a ROS bag.')
    parser.add_argument('bag_file', help='Input ROS bag.')
    parser.add_argument('output_dir', help='Output directory.')
    parser.add_argument('image_topic', help='Image topic.')
    args = parser.parse_args()

    bridge = CvBridge()
    bag = rosbag.Bag(args.bag_file, 'r')
    total = bag.get_message_count(args.image_topic)

    print('Extracting %d images from %s on topic %s into %s...\n' % (total,
        args.bag_file, args.image_topic, args.output_dir))

    count = 0
    for topic, msg, t in bag.read_messages(topics=[args.image_topic]):
        sys.stdout.write('\rProcessing %05d/%05d...' % (count + 1, total))
        sys.stdout.flush()

        img = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        filename = os.path.join(args.output_dir, 'frame-%05d-%d.png' % (count, t.to_nsec()))
        cv2.imwrite(filename, img)

        count += 1

    bag.close()
    print('\nDone!')


if __name__ == '__main__':
    main()
