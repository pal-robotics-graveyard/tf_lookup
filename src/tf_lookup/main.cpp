#include <ros/ros.h>
#include "pal_tf_lookup/TfLookup.h"

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "tf_lookup");
  ROS_INFO("tf_lookup starting");

  ros::NodeHandle nh;
  pal::TfLookup tfl;
  tfl.start(nh);

  ros::spin();
}
