#!/usr/bin/env python
# -*- coding: utf-8 -*-

'''
Author  : Heethesh Vhavle
Email   : heethesh@cmu.edu
Version : 1.0.1
Date    : May 7, 2020
'''

# Python 2/3 compatibility
from __future__ import print_function, absolute_import, division

# Python modules
import os
import datetime

# External modules
import matplotlib.pyplot as plt

# ROS modules
import tf
import rospy
import message_filters

# ROS messages
from sensor_msgs.msg import Image

# Local python modules
import utils

# Global variables
POSE_DATA = []
listener = None

# Set these params
IMAGE_TOPIC = '/camera/rgb/image_color'
FRAME_1 = '/world'
FRAME_2 = '/kinect'


########################### Functions ###########################


def callback(image_msg):
    try:
        (trans, rot) = listener.lookupTransform(FRAME_1, FRAME_2, rospy.Time())
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        return

    timestamp_secs = image_msg.header.stamp.to_sec()

    # Save poses
    POSE_DATA.append([timestamp_secs, trans, rot])


def shutdown_hook():
    filename = '/tmp/%s.txt' % datetime.datetime.now()
    filename = filename.replace(' ', '-')

    with open(filename, 'w') as f:
        # Add any additional headers here
        # f.write('header data\n')

        # Save the pose data
        for data in POSE_DATA:
            f.write('%s %s %s %s %s %s %s %s\n' % (
                data[0],
                data[1][0],
                data[1][1],
                data[1][2],
                data[2][0],
                data[2][1],
                data[2][2],
                data[2][3],
            ))

    rospy.loginfo('%d poses saved to %s' % (len(POSE_DATA), filename))


def run(**kwargs):
    global listener

    # Start node
    rospy.init_node('pose_extractor', anonymous=True)
    rospy.loginfo('Current PID: [%d]' % os.getpid())
    listener = tf.TransformListener()

    # Handle params and topics
    image = rospy.get_param('~image', IMAGE_TOPIC)

    # Subscribe to topics
    info_sub = rospy.Subscriber(image, Image, callback)

    # Shutdown hook
    rospy.on_shutdown(shutdown_hook)

    # Keep python from exiting until this node is stopped
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo('Shutting down')


if __name__ == '__main__':
    run()
