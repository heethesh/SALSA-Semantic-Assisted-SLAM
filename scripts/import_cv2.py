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
import sys
import inspect
import platform


def add_path(path):
    if path not in sys.path:
        sys.path.insert(0, path)

# Display Python version
python_version = platform.python_version()
print('Python', python_version)

# Handle OpenCV
ROS_CV = '/opt/ros/' + os.environ['ROS_DISTRO'] + '/lib/python2.7/dist-packages'
if python_version.startswith('3'):
    if ROS_CV in sys.path: sys.path.remove(ROS_CV)
    import cv2
    print('OpenCV Version (Py3):', cv2.__version__)
    add_path(ROS_CV)
else:
    import cv2
    print('OpenCV Version (ROS):', cv2.__version__)
