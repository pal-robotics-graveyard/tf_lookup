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
    if (!_al_thread)
      _al_thread.reset(new boost::thread(&TfLookup::alExecute, this));

   _check_timer = n.createTimer(ros::Duration(0.1),
       boost::bind(&TfLookup::periodicCheck, this, _1));
  }


  void TfLookup::alExecute()
  {
    while (ros::ok())
    {
      AlServer::GoalHandle gh;
      {
        boost::mutex::scoped_lock lock(_al_mutex);
        if (_al_goals.empty())
        {
          _al_cond.wait(lock);
          continue;
        }

        gh = _al_goals.front();
        _al_goals.pop();
      }
      if (!gh.getGoal())
      {
        gh.setCanceled(AlServer::Result(), "canceled :(");
        continue;
      }
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
  }

  void TfLookup::oneAtATime(AlServer::GoalHandle gh)
  {
    boost::mutex::scoped_lock lock(_al_mutex);
    if (!gh.getGoal())
    {
      gh.setCanceled(AlServer::Result(), "canceled :(");
      return;
    }
    gh.setAccepted();

    _al_goals.push(gh);
    _al_cond.notify_one();
  }

  void TfLookup::periodicCheck(const ros::TimerEvent& te)
  {
    /* If current simulated time is less than previous simulated time,
     * it probably means that we run on simulated time and that gazebo was
     * reset. So we should recreate the tf listener to discard old tranforms
     */
    if (ros::Time::now() < _lastTime)
    {
      boost::mutex::scoped_lock guard(_tfListenerMutex);

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
    boost::mutex::scoped_lock guard(_tfListenerMutex);

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
