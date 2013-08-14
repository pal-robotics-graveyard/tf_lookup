#include "tf_lookup/tf_stream.h"

#include "tf_lookup/Subscription.h"
#include <tf/tfMessage.h>

namespace tf_lookup
{
  TfStream::TfStream(ros::NodeHandle& nh, const std::string& id,
      const LookupFun& lookup_fun) :
    _id(id),
    _pub(nh.advertise<tf::tfMessage>(_id, 10)),
    _lookup_fun(lookup_fun)
  {}

  TfStream::~TfStream()
  {}

  void TfStream::updateTransforms(const TrVect& transforms)
  {
    ROS_DEBUG_STREAM("updating stream " << _id << "with "
        << transforms.size() << " transforms");
    _transforms = transforms;
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
