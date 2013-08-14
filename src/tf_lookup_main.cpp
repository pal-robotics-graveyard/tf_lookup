#include <ros/ros.h>
#include "tf_lookup/tf_lookup.h"

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "tf_lookup");
  ROS_INFO("tf_lookup starting");

  ros::NodeHandle nh;
  tf_lookup::TfLookup tfl;
  tfl.start(nh);

  ros::spin();
}
