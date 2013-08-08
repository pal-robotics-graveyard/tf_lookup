#ifndef TFSTREAM_H
#define TFSTREAM_H

#include <string>
#include <vector>
#include <ros/ros.h>

namespace pal_tf_lookup
{ ROS_DECLARE_MESSAGE(Subscription); }

namespace geometry_msgs
{ ROS_DECLARE_MESSAGE(TransformStamped); }

namespace pal
{
  class TfStream
  {
    private:
      typedef std::vector<pal_tf_lookup::Subscription> TrVect;
      typedef boost::function<bool(const std::string&, const std::string&,
          const ros::Time&, geometry_msgs::TransformStamped&)> LookupFun;

    public:
      TfStream(ros::NodeHandle& nh, const std::string& id,
          const LookupFun& lookup_fun);
      virtual ~TfStream();

      TfStream(const TfStream& rhs)             = delete;
      TfStream& operator=(const TfStream& rhs)  = delete;

      void updateTransforms(const TrVect& transforms);

      void publish();

    private:
      std::string               _id;
      ros::Publisher            _pub;
      TrVect                    _transforms;
      LookupFun                 _lookup_fun;
  };
}

#endif
