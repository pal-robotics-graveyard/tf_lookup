#ifndef TFSTREAM_H
#define TFSTREAM_H

#include <string>
#include <vector>
#include <ros/ros.h>
#include <ros/time.h>

namespace geometry_msgs
{ ROS_DECLARE_MESSAGE(TransformStamped); }

namespace tf_lookup
{
  ROS_DECLARE_MESSAGE(Subscription);

  /** Publish a given set of tf transforms on a topic.
   */
  class TfStream
  {
    private:
      typedef std::vector<tf_lookup::Subscription> TrVect;
      typedef boost::function<bool(const std::string&, const std::string&,
          const ros::Time&, geometry_msgs::TransformStamped&)> LookupFun;

    public:

      /** Initializes the stream.
       * \param nh         NodeHandle for scoping the advertised topic.
       * \param id         Unique id for this stream.
       * \param lookup_fun Function to call to look up a tf transform.
       */
      TfStream(ros::NodeHandle& nh, const std::string& id,
          const LookupFun& lookup_fun);

      virtual ~TfStream();

      TfStream(const TfStream& rhs)             = delete;
      TfStream& operator=(const TfStream& rhs)  = delete;

      void updateTransforms(const TrVect& transforms);
      void publish();
      bool shouldCleanup();

    private:
      std::string               _id;
      ros::Publisher            _pub;
      TrVect                    _transforms;
      LookupFun                 _lookup_fun;
      ros::Time                 _last_subscriber_time;
  };
}

#endif
