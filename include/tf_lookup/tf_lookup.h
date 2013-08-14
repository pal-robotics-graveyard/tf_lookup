#ifndef TF_LOOKUP_H
#define TF_LOOKUP_H

#include <ros/ros.h>
#include <ros/time.h>
#include <actionlib/server/action_server.h>

#include "tf_lookup/TfLookupAction.h"
#include "tf_lookup/tf_stream_server.h"

namespace tf_lookup
{
  ROS_DECLARE_MESSAGE(lookupTransformRequest);
  ROS_DECLARE_MESSAGE(lookupTransformResponse);
}

namespace geometry_msgs
{ ROS_DECLARE_MESSAGE(TransformStamped); }

namespace tf
{
  class TransformListener;
  class StampedTransform;
}

namespace tf_lookup
{
  class TfLookup
  {
    private:
      typedef actionlib::ActionServer<tf_lookup::TfLookupAction> AlServer;

    public:
      TfLookup();
      virtual ~TfLookup();
      /**
       * @brief Advertises the services the plugin provides so they can be
       *        called from any ros node.
       */
      virtual void start(ros::NodeHandle &n);

    private:
      void periodicCheck(const ros::TimerEvent& te);
      bool srvLookupTransform(tf_lookup::lookupTransformRequest &req,
          tf_lookup::lookupTransformResponse &res);
      bool _lookupTransform(const std::string& target,
          const std::string& source, const ros::Time& time,
          tf::StampedTransform& trans);
      bool lookupTransform(const std::string& target,
          const std::string& source, const ros::Time& time,
          geometry_msgs::TransformStamped& trans);
      void alGoalCb(AlServer::GoalHandle gh);

      ros::ServiceServer                     _srvLookupTransform;
      std::unique_ptr<tf::TransformListener> _tfListener;
      ros::Time                              _lastTime;
      ros::Timer                             _check_timer;
      TfStreamServer                         _tf_streamer;
      std::unique_ptr<AlServer>              _al_server;
  };
}

#endif // TFLOOKUP_H
