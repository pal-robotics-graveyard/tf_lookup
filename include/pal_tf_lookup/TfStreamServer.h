#ifndef TFSTREAMSERVER_H
#define TFSTREAMSERVER_H

#include <string>
#include <map>
#include <memory>
#include <ros/ros.h>
#include <actionlib/server/action_server.h>
#include "pal_tf_lookup/TfStreamAction.h"

namespace geometry_msgs
{ ROS_DECLARE_MESSAGE(TransformStamped); }

namespace pal
{
  class TfStream;

  class TfStreamServer
  {
    friend class TfStream;

    private:
      typedef actionlib::ActionServer<pal_tf_lookup::TfStreamAction> AlServer;
      typedef boost::shared_ptr<TfStream> StreamPtr;
      typedef boost::function<bool(const std::string&, const std::string&,
          const ros::Time&, geometry_msgs::TransformStamped&)> LookupFun;

    public:
      TfStreamServer();
      virtual ~TfStreamServer();

      void start(ros::NodeHandle& nh, const LookupFun& lookup_fun);

    private:
      void alGoalCb(AlServer::GoalHandle gh);
      void alCancelCb(AlServer::GoalHandle gh);
      void alStreamer();
      std::string generateId();
      void updateStream(AlServer::GoalHandle gh);
      void addStream(AlServer::GoalHandle gh);

      std::unique_ptr<AlServer>        _al_server;
      ros::NodeHandle                  _nh;
      ros::Timer                       _al_stream_timer;
      std::map<std::string, StreamPtr> _streams;
      LookupFun                        _lookup_fun;
  };
}

#endif
