#include "tf_lookup/tf_stream.h"

#include "tf_lookup/Subscription.h"
#include <tf/tfMessage.h>

namespace tf_lookup
{
  TfStream::TfStream(ros::NodeHandle& nh, const std::string& id,
      const LookupFun& lookup_fun) :
    _id(id),
    _pub(nh.advertise<tf::tfMessage>(_id, 10)),
    _lookup_fun(lookup_fun),
    _last_subscriber_time(ros::Time::now())
  {}

  TfStream::~TfStream()
  {}

  void TfStream::updateTransforms(const TrVect& transforms)
  {
    ROS_DEBUG_STREAM("updating stream " << _id << "with "
        << transforms.size() << " transforms");
    _transforms = transforms;
  }

  bool TfStream::shouldCleanup()
  {
    if (_pub.getNumSubscribers() > 0)
      _last_subscriber_time = ros::Time::now();

    if (ros::Time::now().toSec() - _last_subscriber_time.toSec() < 5.0) //XXX: magic value
      return false;

    return true;
  }

  void TfStream::publish()
  {
    tf::tfMessage m;
    ROS_DEBUG_STREAM("publishing for stream " << _id);
    for (auto& t : _transforms)
    {
      ROS_DEBUG_STREAM("  - transform from " << t.target << " to " << t.source);
      geometry_msgs::TransformStamped trans;
      if (_lookup_fun(t.target, t.source, ros::Time(0), trans))
        m.transforms.push_back(trans);
    }

    _pub.publish(m);
  }
}
