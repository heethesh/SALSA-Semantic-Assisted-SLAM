#!/usr/bin/env python
# -*- coding: utf-8 -*-

'''
Author  : Heethesh Vhavle
Email   : heethesh@cmu.edu
Version : 1.0.1
Date    : Apr 24, 2020

References:
http://wiki.ros.org/message_filters
http://wiki.ros.org/cv_bridge/Tutorials/
http://docs.ros.org/api/image_geometry/html/python/
http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscribe
'''

# Python 2/3 compatibility
from __future__ import print_function, absolute_import, division

# Python modules
import os

# Handle paths and OpenCV import
from import_cv2 import *

# External modules
import matplotlib.pyplot as plt

# ROS modules
import rospy
import message_filters

# ROS messages
from sensor_msgs.msg import Image

# Local python modules
import utils

# Global variables
COUNT = 0


########################### Functions ###########################


def callback(image_msg, scores_msg, **kwargs):
    global COUNT

    # Convert to cv2 images
    img = utils.message_to_cv2(image_msg)
    scores = utils.message_to_cv2(scores_msg)

    # Overlay
    output = cv2.addWeighted(scores, 0.6, img, 1.0, 0)
    cv2.imshow('image', output)
    cv2.waitKey(2)
    # cv2.imwrite('test/%06d.png' % COUNT, output)
    COUNT += 1


def shutdown_hook():
    cv2.destroyAllWindows()


def run(**kwargs):
    # Start node
    rospy.init_node('visualizer', anonymous=True)
    rospy.loginfo('Current PID: [%d]' % os.getpid())

    # Handle params and topics
    image = rospy.get_param('~image', '/d400/color/image_raw')
    scores = rospy.get_param('~scores', '/d400/annotation/dynamic_scores')

    # Publish output topic
    # image_pub = rospy.Publisher(output_image, Image, queue_size=5)

    # Subscribe to topics
    image_sub = message_filters.Subscriber(image, Image)
    scores_sub = message_filters.Subscriber(scores, Image)

    # Synchronize the topics by time
    ats = message_filters.ApproximateTimeSynchronizer(
        [image_sub, scores_sub], queue_size=1, slop=0.5)
    ats.registerCallback(callback, **kwargs)

    # Shutdown hook
    rospy.on_shutdown(shutdown_hook)

    # Keep python from exiting until this node is stopped
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo('Shutting down')


if __name__ == '__main__':
    run()
