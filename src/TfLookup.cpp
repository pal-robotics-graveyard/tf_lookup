#include "pal_tf_lookup/TfLookup.h"

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include "pal_tf_lookup/lookupTransform.h"

namespace pal
{
  TfLookup::TfLookup()
  {
    resetTfListener();
  }

  TfLookup::~TfLookup()
  {}

  void TfLookup::advertiseServices(ros::NodeHandle &n)
  {
    _srvLookupTransform = n.advertiseService("/lookupTransform",
        &TfLookup::srvLookupTransform, this);
  }

  void TfLookup::resetTfListener()
  {
    boost::mutex::scoped_lock guard(_tfListenerMutex);

    ROS_WARN("detected a time rollback, resetting tfListener");
    _tfListener.reset(new tf::TransformListener());
    _tfListener->setExtrapolationLimit(ros::Duration(0.2));
  }

  /* call me maybe */
  void TfLookup::periodicCheck()
  {
    /* If current simulated time is less than previous simulated time,
     * it probably means that we run on simulated time and that gazebo was
     * reset. So we should recreate the tf listener to discard old tranforms
     */
    if (ros::Time::now() < _lastTime)
      resetTfListener();

    _lastTime = ros::Time::now();
  }

  bool TfLookup::srvLookupTransform(pal_tf_lookup::lookupTransformRequest &req,
      pal_tf_lookup::lookupTransformResponse &res)
  {
    tf::StampedTransform trans;
    try
    {
      boost::mutex::scoped_lock guard(_tfListenerMutex);
      _tfListener->lookupTransform(req.target_frame, req.source_frame,
          req.transform_time, trans);
    }
    catch (tf::TransformException const &e)
    {
      ROS_ERROR_STREAM_THROTTLE(1,
          "Error getting transform from " << req.source_frame
          << " to " << req.target_frame
          << " at time: " << req.transform_time
          << " : " << e.what());
      return false;
    }

    tf::transformStampedTFToMsg(trans, res.transform);
    return true;
  }
}
