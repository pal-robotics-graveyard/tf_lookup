#include "pal_tf_lookup/TfLookup.h"

#include <ros/ros.h>
#include <ros/time.h>
#include <boost/thread.hpp>
#include <tf/transform_listener.h>
#include "pal_tf_lookup/lookupTransform.h"
#include "pal_tf_lookup/TfLookupAction.h"
#include <actionlib/server/action_server.h>
#include <queue>

namespace pal
{
  TfLookup::TfLookup()
  {
    _tfListener.reset(new tf::TransformListener());
    _tfListener->setExtrapolationLimit(ros::Duration(0.2));
  }

  TfLookup::~TfLookup()
  {}

  void TfLookup::start(ros::NodeHandle &n)
  {
    _srvLookupTransform = n.advertiseService("/lookupTransform",
        &TfLookup::srvLookupTransform, this);

    _al_server.reset(new AlServer(n, "tf_lookup",
          boost::bind(&TfLookup::oneAtATime, this, _1), false));
    _al_server->start();

   _check_timer = n.createTimer(ros::Duration(5),
       boost::bind(&TfLookup::periodicCheck, this, _1));
  }

  void TfLookup::alRespond(AlServer::GoalHandle gh)
  {
    auto goal = gh.getGoal();

    tf::StampedTransform trans;
    bool success = lookupTransform(goal->target_frame, goal->source_frame,
        goal->transform_time, trans);

    if (success)
    {
      AlServer::Result res;
      tf::transformStampedTFToMsg(trans, res.transform);
      gh.setSucceeded(res);
    }
    else
      gh.setAborted(AlServer::Result(), "could not find a transform");
  }

  void TfLookup::oneAtATime(AlServer::GoalHandle gh)
  {
    if (!gh.getGoal())
    {
      gh.setCanceled(AlServer::Result(), "canceled :(");
      return;
    }
    gh.setAccepted();
    alRespond(gh);
  }

  void TfLookup::periodicCheck(const ros::TimerEvent& te)
  {
    /* If current simulated time is less than previous simulated time,
     * it probably means that we run on simulated time and that gazebo was
     * reset. So we should recreate the tf listener to discard old tranforms
     */
    if (ros::Time::now() < _lastTime)
    {
      ROS_WARN("detected a time rollback, resetting tfListener");
      _tfListener.reset(new tf::TransformListener());
      _tfListener->setExtrapolationLimit(ros::Duration(0.2));
    }

    _lastTime = ros::Time::now();
  }

  bool TfLookup::srvLookupTransform(pal_tf_lookup::lookupTransformRequest &req,
      pal_tf_lookup::lookupTransformResponse &res)
  {
    tf::StampedTransform trans;

    bool success = lookupTransform(req.target_frame, req.source_frame,
          req.transform_time, trans);
    if (success)
      tf::transformStampedTFToMsg(trans, res.transform);

    return success;
  }

  bool TfLookup::lookupTransform(const std::string& target,
      const std::string& source, const ros::Time& time,
      tf::StampedTransform& trans)
  {
    std::string err;

    if (!_tfListener->canTransform(target, source, time, &err))
    {
      ROS_ERROR_STREAM_THROTTLE(1,
          "Error getting transform from " << source
          << " to " << target
          << " at time: " << time
          << " : " << err);
      return false;
    }

    _tfListener->lookupTransform(target, source, time, trans);
    return true;
  }
}
