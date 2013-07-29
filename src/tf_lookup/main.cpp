#include "pal_tf_lookup/TfLookup.h"

#include <ros/ros.h>

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "tf_lookup");

  ROS_INFO("tf_lookup starting");

  ros::NodeHandle nh;
  pal::TfLookup tfl;
  tfl.advertiseServices(nh);
  ros::Rate rate(10);

  while(ros::ok())
  {
    int i = 0;
    ros::spinOnce();
    if (++i%100)
      tfl.periodicCheck();
    rate.sleep();
  }
}
