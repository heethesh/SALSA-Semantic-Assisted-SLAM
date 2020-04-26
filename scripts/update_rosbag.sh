#!/bin/bash
# @Author: Heethesh Vhavle
# @Date:   Apr 24, 2020
# @Last Modified by:   Heethesh Vhavle
# @Last Modified time: Apr 26, 2020

python update_rosbag.py \
  --input-bag "/home/heethesh/ROS-Workspaces/slam_ws/src/bags/rgbd_dataset_freiburg3_walking_xyz/rgbd_dataset_freiburg3_walking_xyz.bag" \
  --output-bag "/home/heethesh/ROS-Workspaces/slam_ws/src/bags/rgbd_dataset_freiburg3_walking_xyz/rgbd_dataset_freiburg3_walking_xyz-annotated.bag" \
  --image-dir "/home/heethesh/ROS-Workspaces/slam_ws/src/annotated_images/walking_xyz" \
  --dataset "tum-rgbd"
