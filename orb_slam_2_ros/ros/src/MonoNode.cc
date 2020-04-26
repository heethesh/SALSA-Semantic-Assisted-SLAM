#include "MonoNode.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "Mono");
  ros::start();

  if (argc > 1) {
    ROS_WARN("Arguments supplied via command line are neglected.");
  }

  // Create SLAM system. It initializes all system threads and gets ready to
  // process frames.
  ros::NodeHandle node_handle;
  image_transport::ImageTransport image_transport(node_handle);

  MonoNode node(ORB_SLAM2::System::MONOCULAR, node_handle, image_transport);

  node.Init();

  ros::spin();

  ros::shutdown();

  return 0;
}

MonoNode::MonoNode(ORB_SLAM2::System::eSensor sensor,
                   ros::NodeHandle &node_handle,
                   image_transport::ImageTransport &image_transport)
    : Node(sensor, node_handle, image_transport) {
  // image_subscriber = image_transport.subscribe("/camera/image_raw", 1,
  //                                              &MonoNode::ImageCallback, this);
  image_subscriber = new message_filters::Subscriber<sensor_msgs::Image>(
      node_handle, "/camera/image_raw", 1);
  semantic_subscriber_ = new message_filters::Subscriber<sensor_msgs::Image> (
      node_handle, "/camera/annotation/dynamic_scores", 1);
  camera_info_topic_ = "/camera/camera_info";

  sync_ = new message_filters::Synchronizer<sync_pol> (
      sync_pol(10), *image_subscriber, *semantic_subscriber_ );
  sync_->registerCallback(boost::bind(&MonoNode::ImageCallback, this, _1, _2));

}

MonoNode::~MonoNode() {}

void MonoNode::ImageCallback(const sensor_msgs::ImageConstPtr &msg,
                             const sensor_msgs::ImageConstPtr& msgSem) {
  cv_bridge::CvImageConstPtr cv_in_ptr;
  try {
    cv_in_ptr = cv_bridge::toCvShare(msg);
  } catch (cv_bridge::Exception &e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  cv_bridge::CvImageConstPtr cv_ptrSem;
  try {
    cv_ptrSem = cv_bridge::toCvShare(msgSem);
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  current_frame_time_ = msg->header.stamp;

  orb_slam_->TrackMonocular(cv_in_ptr->image, cv_in_ptr->header.stamp.toSec(), cv_ptrSem->image);

  Update();
}
