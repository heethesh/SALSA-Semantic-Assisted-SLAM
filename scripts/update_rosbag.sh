#!/bin/bash
# @Author: Heethesh Vhavle
# @Date:   Apr 24, 2020
# @Last Modified by:   Heethesh Vhavle
# @Last Modified time: Apr 24, 2020

python update_rosbag.py \
  --input-bag "/home/heethesh/ROS-Workspaces/slam_ws/src/bags/cafe1-1_2-rosbag/uncompressed/cafe1-2.bag" \
  --output-bag "/home/heethesh/ROS-Workspaces/slam_ws/src/bags/cafe1-1_2-rosbag/uncompressed/cafe1-2-annotated.bag" \
  --image-dir "/home/heethesh/ROS-Workspaces/slam_ws/src/annotated_images/cafe1-2"
