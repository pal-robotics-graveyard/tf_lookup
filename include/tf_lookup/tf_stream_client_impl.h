#include "tf_lookup/tf_stream_client.h"

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/tfMessage.h>

#include "tf_lookup/TfStreamAction.h"
#include "tf_lookup/tf_sc_transform.h"

namespace tf_lookup
{
  TfStreamClient::TfStreamClient(ros::NodeHandle& nh) : _nh(nh)
  {
    _al_client.reset(new AlClient(_nh, "/tf_stream"));
  }

  TfStreamClient::~TfStreamClient()
  {}

  TfStreamClient::Handle TfStreamClient::addTransform(const std::string& target,
      const std::string& source, const Callback& cb)
  {
    return Handle(new TfSCTransform(target + "@" + source, this, cb));
  }

  void TfStreamClient::updateTransforms()
  {
    /* Retry every second if the server is not reachable */
    if (!_al_client->isServerConnected() || (!_sub && _sub_id == "pending"))
    {
      _retry_timer = _nh.createTimer(ros::Duration(1.0),
          boost::bind(&TfStreamClient::updateTransforms, this), true);
      return;
    }

    AlGoal g;
    if (_sub)
    {
      g.update = true;
      g.subscription_id = _sub_id;

      _retry_timer.stop();
    }
    else
      _sub_id = "pending";

    g.transforms.reserve(_transforms.size());
    for (auto t : _transforms)
    {
      tf_lookup::Subscription s;
      auto mid = t.first.find("@");
      s.target = t.first.substr(0, mid);
      s.source = t.first.substr(mid+1);
      g.transforms.push_back(s);
    }

    _al_client->sendGoal(g,
        boost::bind(&TfStreamClient::alCallback, this, _1, _2));
  }

  void TfStreamClient::mainCallback(const FeedConstPtr& feed)
  {
    for (auto t : feed->transforms)
    {
      const std::string& parent = t.header.frame_id;
      const std::string& child = t.child_frame_id;
      std::string tr = parent + "@" + child;

      auto it = _transforms.find(tr);
      if (it == _transforms.end())
      {
        ROS_WARN("we have received an unsollicited transform: [%s]->[%s]",
            parent.c_str(), child.c_str());
        continue;
      }
      it->second->_cb(boost::shared_ptr<geometry_msgs::TransformStamped>
          (new geometry_msgs::TransformStamped(t)));
    }
  }

  void TfStreamClient::alCallback(const actionlib::SimpleClientGoalState& gs,
      const AlResultConstPtr& result)
  {
    if (gs != actionlib::SimpleClientGoalState::SUCCEEDED)
      return;

    if(!_sub)
    {
      if (result->topic == "") /* these aren't the droids you're looking for */
      {
        updateTransforms();
        return;
      }
      _sub.reset(new ros::Subscriber(_nh.subscribe(result->topic, 1,
            &TfStreamClient::mainCallback, this)));

      _sub_id = result->subscription_id;
    }
  }


  TfSCTransform::TfSCTransform(const std::string& key,
      TfStreamClient* psc, const TfStreamClient::Callback& cb)
    : _psc(psc), _cb(cb), _key(key)
  {
    ROS_INFO_STREAM("created tf stream for " << _key);
    _psc->_transforms[key] = this;
    _psc->updateTransforms();
  }

  TfSCTransform::~TfSCTransform()
  {
    _psc->_transforms.erase(_key);
    _psc->updateTransforms();
  }
}
