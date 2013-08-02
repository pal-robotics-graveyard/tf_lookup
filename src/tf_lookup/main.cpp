#include "pal_tf_lookup/TfLookup.h"

#include <ros/ros.h>

void cb(const ros::TimerEvent& te, pal::TfLookup& tfl)
{
  tfl.periodicCheck();
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "tf_lookup");

  ROS_INFO("tf_lookup starting");

  ros::NodeHandle nh;
  pal::TfLookup tfl;
  tfl.advertiseServices(nh);
  tfl.startAlServer(nh);

  ros::Timer t = nh.createTimer(ros::Duration(0.1), boost::bind(&cb, _1, boost::ref(tfl)));
  ros::spin();
}
