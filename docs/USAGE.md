# Usage

The current version does not perform segmentation online, the score maps are pre-computed using Detectron 2's Mask RCNN network and appended into the ROS bag file.

## Extract Images from ROS Bag

Run the following script to extract images, given a topic, from a ROS bag file to a specific folder (we used TUM-RGBD and OpenLORIS-Scene here). Remember to handle RGB/BGR image convention based on the dataset.

```
python bag_to_images.py BAG_FILE OUTPUT_DIR IMAGE_TOPIC
```

## Generating Score Maps

Here, we will segment the objects and compute the scoremaps (see the [report](docs/report.pdf) for more details). Make sure you clone the `detectron2` submodule too which is in this repository and follow the installation procedure [here](https://github.com/heethesh/detectron2/blob/8e67751ccc666ef3f5845222b707d37365a6ec19/INSTALL.md).

```
git submodule update --init --recursive
```

As mentioned in the report, we manually assign scores for each of the classes of objects (COCO here). An example scores file is shown [here](data/scores.tsv). The columns represent scores for dynamic, possibly dynamic, and repeatable categories. The row order of class labels is the same as the COCO 2014/2017 release (see [this file](https://github.com/amikelive/coco-labels/blob/master/coco-labels-2014_2017.txt) for the order).

Use the [`segment.sh`](https://github.com/heethesh/detectron2/blob/8e67751ccc666ef3f5845222b707d37365a6ec19/demo/segment.sh) script to run the segmentation. Specify the extracted images, output folder, and the scores files path. This script also lets you change the dataset, network, and weights to use another segmentation algorithm.

## Appending Score Maps into ROS Bag

Finally, append the scoremaps generated back into the ROS bag file. This will actually create a new bag file with the scores topic (at `/CAMERA_NAME/annotation/dynamic_scores`). `DATASET` can either be `"tum-rgbd"` or `"open-loris"`.

```
python update_rosbag.py \
  --input-bag INPUT_BAG_PATH \
  --output-bag OUTPUT_BAG_PATH \
  --image-dir SCOREMAPS_FOLDER \
  --dataset DATASET
```

You should now be able to playback the ROS bag and visualize the synchronized scoremaps. **Note**: there is a known segfault issue related to image encoding and OpenCV while visualizing the scoremaps topic using the `image_view` node. Try visualizing it using Rviz. The scoremaps data in the topic, however, should still be valid and works with the ORB-SLAM pipeline.

## Launch SALSA

Follow the setup procedure [here](https://github.com/appliedAI-Initiative/orb_slam_2_ros#2-building-orb_slam2_ros) to install ORB-SLAM dependencies.

Now, we are ready to run the ORB-SLAM pipeline. Please see our report for the changes that we have made in the pipeline. We have tested on monocular camera data, RGB-D camera is still experimental and might work. Stereo mode is supported in the back-end optimization but the front-end (feature culling and ROS node integration) is not fully supported yet. We have already updated the camera parameters and topic names for D435 (for OpenLORIS-Scene) and TUM-RGBD [here](https://github.com/heethesh/SALSA-Semantic-Assisted-SLAM/tree/master/orb_slam_2_ros/ros/launch).

```
catkin build
rosbag play BAG_FILE  // -l for loop optionally
roslaunch orb_slam_2_ros salsa.launch
```

The backend optimization changes can be disabled and the optimizer weights (as discussed in our paper) can be set [here](https://github.com/heethesh/SALSA-Semantic-Assisted-SLAM/blob/master/orb_slam_2_ros/orb_slam2/src/Optimizer.cc#L41).

## Evaluation

You may use the [export_pose.py](scripts/export_pose.py) script to export the current camera poses to file and use TUM-RGBDs trajectory evaluation tools. This script is a ROS node and hit `Ctrl+C` to shutdown the node and the poses file is generated at `/tmp`.

```
rosrun export_poses.py
Ctrl+C  // to shutdown node and save the poses
```
