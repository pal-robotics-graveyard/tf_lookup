#ifndef TFLOOKUPCLIENT_H
#define TFLOOKUPCLIENT_H

#include <memory>
#include <string>
#include <vector>
#include <ros/ros.h>
#include <boost/function.hpp>
#include <actionlib/client/action_client.h>

#include "tf_lookup/TfLookupAction.h"

namespace geometry_msgs
{ ROS_DECLARE_MESSAGE(TransformStamped); }

namespace tf_lookup
{
  class TfLookupClient
  {
    private:
      typedef geometry_msgs::TransformStamped                        Transform;
      typedef geometry_msgs::TransformStampedConstPtr         TransformConstPtr;
      typedef boost::function<void(bool, const TransformConstPtr&)>  Callback;
      typedef actionlib::ActionClient<tf_lookup::TfLookupAction> AlClient;
      typedef tf_lookup::TfLookupGoal                            Goal;
      typedef std::pair<AlClient::GoalHandle, Callback>              GhCbPair;

      struct gh_compare : public std::unary_function<GhCbPair, bool>
      {
        explicit gh_compare(const AlClient::GoalHandle& gh) : _gh(gh) {}
        bool operator() (const GhCbPair& p) { return p.first == _gh; }
        AlClient::GoalHandle _gh;
      };

    public:
      TfLookupClient(ros::NodeHandle &nh);
      virtual ~TfLookupClient();

      bool queryTransform(const std::string& target, const std::string& source,
          const Callback& cb);

    private:
      void tfAlTransitionCb(AlClient::GoalHandle gh);

      ros::NodeHandle           _nh;
      std::unique_ptr<AlClient> _al_client;
      std::vector<GhCbPair>     _al_goals;
  };
}

#endif
