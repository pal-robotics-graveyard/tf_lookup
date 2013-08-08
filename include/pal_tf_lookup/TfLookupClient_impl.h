#include "pal_tf_lookup/TfLookupClient.h"

#include <geometry_msgs/TransformStamped.h>
#include <ros/time.h>

namespace pal
{
  TfLookupClient::TfLookupClient(ros::NodeHandle &nh)
    : _nh(nh)
  {
    _al_client.reset(new AlClient(_nh, "/tf_lookup"));
  }

  TfLookupClient::~TfLookupClient()
  {}

  bool TfLookupClient::queryTransform(const std::string& target,
      const std::string& source, const Callback& cb)
  {
    if (!_al_client->isServerConnected())
      return false;

    Goal goal;
    goal.target_frame = target;
    goal.source_frame = source;
    goal.transform_time = ros::Time(0);

    _al_goals.push_back(GhCbPair(_al_client->sendGoal(goal,
          boost::bind(&TfLookupClient::tfAlTransitionCb, this, _1)), cb));

    return true;
  }

  void TfLookupClient::tfAlTransitionCb(AlClient::GoalHandle gh)
  {
    auto it = std::find_if(_al_goals.begin(), _al_goals.end(), gh_compare(gh));
    if (it == _al_goals.end())
      return;

    switch (gh.getCommState().state_)
    {
      case actionlib::CommState::DONE:
        switch(gh.getTerminalState().state_)
        {
          case actionlib::TerminalState::SUCCEEDED:
            it->second(true,
                TransformConstPtr(new Transform(gh.getResult()->transform)));
            break;
          default:
            it->second(false, TransformConstPtr());
            break;
        }
        _al_goals.erase(it);
        break;
      default:
        break;
    }
  }
}
