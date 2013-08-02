#ifndef TF_LOOKUP_H
#define TF_LOOKUP_H

#include "pal_tf_lookup/TfLookupAction.h"
#include <ros/ros.h>
#include <ros/time.h>
#include <actionlib/server/action_server.h>

namespace pal_tf_lookup
{
  ROS_DECLARE_MESSAGE(lookupTransformRequest);
  ROS_DECLARE_MESSAGE(lookupTransformResponse);
}

namespace tf
{
  class TransformListener;
  class StampedTransform;
}

namespace pal
{
  class TfLookup
  {
    public:
      TfLookup();
      virtual ~TfLookup();
      /**
       * @brief Advertises the services the plugin provides so they can be
       *        called from any ros node.
       */
      virtual void start(ros::NodeHandle &n);

    private:
      typedef actionlib::ActionServer<pal_tf_lookup::TfLookupAction> AlServer;

      void periodicCheck(const ros::TimerEvent& te);
      bool srvLookupTransform(pal_tf_lookup::lookupTransformRequest &req,
          pal_tf_lookup::lookupTransformResponse &res);
      void alRespond(AlServer::GoalHandle gh);
      bool lookupTransform(const std::string& target,
          const std::string& source, const ros::Time& time,
          tf::StampedTransform& trans);
      void oneAtATime(AlServer::GoalHandle goal);

      ros::ServiceServer                     _srvLookupTransform;
      std::unique_ptr<tf::TransformListener> _tfListener;
      ros::Time                              _lastTime;
      ros::Timer                             _check_timer;

      std::unique_ptr<AlServer>              _al_server;
      pal_tf_lookup::TfLookupResult          _al_result;
  };
}

#endif // TFLOOKUP_H
