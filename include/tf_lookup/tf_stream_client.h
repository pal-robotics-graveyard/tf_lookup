#ifndef TFSTREAMCLIENT_H
#define TFSTREAMCLIENT_H

#include <map>
#include <string>
#include <memory>
#include <boost/function.hpp>
#include <boost/shared_ptr.hpp>
#include <ros/ros.h>
#include <ros/time.h>

namespace geometry_msgs
{ ROS_DECLARE_MESSAGE(TransformStamped); }
namespace tf
{ ROS_DECLARE_MESSAGE(tfMessage); }
namespace actionlib
{
  template <class T>
  class SimpleActionClient;
  class SimpleClientGoalState;
}

namespace tf_lookup
{
  ROS_DECLARE_MESSAGE(TfStreamAction);
  ROS_DECLARE_MESSAGE(TfStreamGoal);
  ROS_DECLARE_MESSAGE(TfStreamResult);

  class TfSCTransform;

  class TfStreamClient
  {
    friend class TfSCTransform;

    public:
      typedef boost::shared_ptr<TfSCTransform> Handle;

    private:
      typedef boost::function
        <void(const geometry_msgs::TransformStampedConstPtr&)> Callback;
      typedef std::list<TfSCTransform*>::iterator              ListIter;
      typedef actionlib::SimpleActionClient
        <tf_lookup::TfStreamAction>                        AlClient;
      typedef tf_lookup::TfStreamGoal                      AlGoal;
      typedef tf_lookup::TfStreamResultConstPtr            AlResultConstPtr;
      typedef tf::tfMessageConstPtr                            FeedConstPtr;

    public:
      TfStreamClient(ros::NodeHandle& nh);

      virtual ~TfStreamClient();

      /**
       * Adds a subscription to a transform. The handle returned is used to
       * manage the subscription. When the last handle dies, the callback is
       * deleted and the subscription is canceled
       */
      Handle addTransform(const std::string& target,
          const std::string& source, const Callback& cb);

    private:
      void mainCallback(const FeedConstPtr& feed);
      void alCallback(const actionlib::SimpleClientGoalState& state,
          const AlResultConstPtr& result);
      void updateTransforms();

      std::map<std::string, TfSCTransform*>  _transforms;
      ros::NodeHandle                        _nh;
      std::unique_ptr<ros::Subscriber>       _sub;
      std::unique_ptr<AlClient>              _al_client;
      std::string                            _sub_id;
      ros::Timer                             _retry_timer;
  };
}

#endif
