[![ROS Distro: Melodic](https://img.shields.io/badge/ROS-Melodic-blue.svg)](http://wiki.ros.org/melodic)
[![CI](https://github.com/heethesh/SLAM-Project/workflows/Lint/badge.svg)](https://github.com/heethesh/SLAM-Project/actions)
[![License: BSD](https://img.shields.io/badge/License-BSD-yellow.svg)](./LICENSE)

# SALSA: Semantic Assisted Life-Long SLAM for Indoor Environments

We propose a learning augmented lifelong SLAM method for indoor environments. Most of the existing SLAM methods assume a static environment and disregard dynamic objects. Another problem is that most feature and semantic based SLAM methods fail in repetitive environments. The unexpected changes of surroundings corrupts the quality of the tracking and leads to system failure. This project aims to use learning methods to classify landmarks and objects as dynamic and/or repeatable in nature to better handle optimization, achieve robust performance in a changing environment, and to re-localize in a lifelong-setting. We propose using semantic information and assigning scores to object feature points based on their probability to be dynamic and/or repeatable. We update the front-end, optimization cost functions, and BOW feature generation for loop closures from the original [ORB-SLAM2 pipeline](https://github.com/appliedAI-Initiative/orb_slam_2_ros). Please see [our paper](docs/report.pdf) for more details.

<img src="docs/scores-table.jpg?raw=true" width="450">
All 80 classes from the COCO dataset are assigned scores. This list is only a subset of some interesting objects. Some objects are exclusively labelled as dynamic objects.

&nbsp;  
&nbsp;
<img src="docs/scoremaps.jpg?raw=true" width="500">

Mask-RCNN was used to segment object instances and the scores were mapped according to the scale shown on the right. The left column shows score maps for the OpenLORIS-Scene [3] cafe1-2 sequence and the right column shows the score maps for TUM-RGBD  walking_static sequence.

## Setup and Usage
The current version does not perform segmentation online, the score maps are pre-computed using Detectron 2's Mask RCNN network and appended into the ROS bag file (using `scripts/update_rosbag.py`). See [USAGE.md](docs/USAGE.md) for more details on setting up and running this pipeline.

```
catkin build
roslaunch orb_slam_2_ros salsa.launch
```

## Results

#### RMSE Absolute Trajectory Error (ATE) on TUM-RGBD Dataset
| Sequences      | ORB-SLAM2 [1] | DS-SLAM [2] | SALSA (Ours) |
|----------------|---------------|-------------|--------------|
| walking_static | 0.4030m       | 0.0081m     | **0.0059m**      |
| walking_xyz    | 0.1780m       | **0.0247m**     | 0.0521m      |

#### RMSE Relative Pose Error (RPE) on TUM-RGBD Dataset
| Sequences      | ORB-SLAM2 [1] | DS-SLAM [2] | SALSA (Ours) |
|----------------|---------------|-------------|--------------|
| walking_static | 0.2162m       | 0.0102      | **0.00829m**     |
| walking_xyz    | 0.4124m       | 0.0333      | **0.02951m**     |

#### RMSE Absolute Trajectory Error (ATE) on OpenLORIS-Scene [3] Dataset
| Sequences | ORB-SLAM2 [1] | SALSA (Ours) |
|-----------|-----------|-------------|
| cafe1_1   | 0.0777m   | **0.0464m**     |
| cafe1_2   | 0.0813m   | **0.0588m**     |

<img src="docs/results.png?raw=true">
Results of tracking on OpenLORIS-Scene [3] cafe1-1 sequence, TUM-RGBD walking_static and walking_xyz sequences. The columns depict the ground truth trajectory, estimated trajectory results, and tracking and mapping results visualized in Rviz.

## Videos
TUM-RGBD Sequences: [YouTube Link](https://youtu.be/0TvNcxreGtI)  
OpenLORIS-Scene Sequences: [YouTube Link](https://youtu.be/9ku1nIotTUw).  
<br>
<img src="docs/tracking.png?raw=true">
Dynamic feature points in the red mask region are removed. Possibly dynamic and repeatable feature points have a shade a green/blue and rest of the static object features points are labelled in yellow.

## References
[1] R. Mur-Artal and J. D. Tardos, "Orb-slam2: An open-source slam" system for monocular, stereo, and rgb-d cameras,” IEEE Transactions on Robotics, vol. 33, no. 5, pp. 1255–1262, 2017.  

[2] C. Yu, Z. Liu, X. Liu, F. Xie, Y. Yang, Q. Wei, and Q. Fei, "Dsslam: A semantic visual slam towards dynamic environments," in 2018 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS), 2018, pp. 1168–1174.  

3] X. Shi, D. Li, P. Zhao, Q. Tian, Y. Tian, Q. Long, C. Zhu, J. Song, F. Qiao, L. Song, Y. Guo, Z. Wang, Y. Zhang, B. Qin, W. Yang, F. Wang, R. H. M. Chan, and Q. She, "Are we ready for service robots? the openloris-scene datasets for lifelong slam," 2019. [Dataset Link](https://lifelong-robotic-vision.github.io/dataset/scene)
