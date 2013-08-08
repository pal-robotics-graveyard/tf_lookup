#include "pal_tf_lookup/TfLookup.h"

#include <ros/ros.h>
#include <ros/time.h>
#include <tf/transform_listener.h>
#include "pal_tf_lookup/lookupTransform.h"
#include "pal_tf_lookup/TfLookupAction.h"
#include <actionlib/server/action_server.h>

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
          boost::bind(&TfLookup::alGoalCb, this, _1), false));
    _al_server->start();

    _tf_streamer.start(n, boost::bind(&TfLookup::lookupTransform,
          this, _1, _2, _3, _4));

   _check_timer = n.createTimer(ros::Duration(5),
       boost::bind(&TfLookup::periodicCheck, this, _1));
  }

  void TfLookup::alGoalCb(AlServer::GoalHandle gh)
  {
    if (!gh.getGoal())
    {
      gh.setCanceled(AlServer::Result(), "something went wrong, goal canceled");
      return;
    }
    gh.setAccepted();
    auto goal = gh.getGoal();
    AlServer::Result r;

    if (lookupTransform(goal->target_frame, goal->source_frame,
          ros::Time(0), r.transform))
      gh.setSucceeded(r);
    else
      gh.setAborted(r);
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
    return lookupTransform(req.target_frame, req.source_frame,
          req.transform_time, res.transform);
  }

  bool TfLookup::lookupTransform(const std::string& target,
      const std::string& source, const ros::Time& time,
      geometry_msgs::TransformStamped& trans)
  {
    tf::StampedTransform tr;

    bool success = _lookupTransform(target, source, time, tr);

    if (success)
      tf::transformStampedTFToMsg(tr, trans);

    return success;
  }

  bool TfLookup::_lookupTransform(const std::string& target,
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
