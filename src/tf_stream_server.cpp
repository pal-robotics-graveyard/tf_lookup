#include "tf_lookup/tf_stream_server.h"

#include "tf_lookup/tf_stream.h"

namespace tf_lookup
{
  TfStreamServer::TfStreamServer()
  {}

  TfStreamServer::~TfStreamServer()
  {}

  void TfStreamServer::start(ros::NodeHandle& nh, const LookupFun& lookup_fun, const ResolveFun& resolve_fun)
  {
    _nh = nh;
    _lookup_fun = lookup_fun;
    _resolve_fun = resolve_fun;
    _al_server.reset(new AlServer(_nh, "tf_stream",
          boost::bind(&TfStreamServer::alGoalCb, this, _1), false));
    _al_server->start();
    _al_stream_timer = _nh.createTimer(ros::Duration(0.1),
        boost::bind(&TfStreamServer::alStreamer, this));
  }

  void TfStreamServer::alStreamer()
  {
    for (auto it=_streams.begin(); it != _streams.end(); )
    {
      if (it->second->shouldCleanup())
        it = _streams.erase(it);
      else
        ++it;
    }

    for (auto it : _streams)
      it.second->publish();
  }

  std::string TfStreamServer::generateId()
  {
    std::ostringstream s;
    s << "tfs_" << ros::Time::now().toNSec();
    return s.str();
  }

  void TfStreamServer::updateStream(AlServer::GoalHandle gh)
  {
    const auto& goal = gh.getGoal();
    const std::string& id = goal->subscription_id;
    if (_streams.find(id) == _streams.end())
    {
      gh.setCanceled();
      return;
    }

    gh.setAccepted();

    TrVect transforms;
    int ts = goal->transforms.size();
    transforms.resize(ts);
    for (int i=0; i < ts; ++i)
    {
      transforms[i].target = _resolve_fun(goal->transforms[i].target);
      transforms[i].source = _resolve_fun(goal->transforms[i].source);
    }

    _streams[id]->updateTransforms(transforms);
    gh.setSucceeded();
  }

  void TfStreamServer::addStream(AlServer::GoalHandle gh)
  {
    gh.setAccepted();
    std::string id = generateId();
    _streams[id].reset(new TfStream(_nh, id, _lookup_fun));
    _streams[id]->updateTransforms(gh.getGoal()->transforms);
    AlServer::Result r;
    r.subscription_id = id;
    r.topic = _nh.resolveName(id);
    gh.setSucceeded(r);
  }

  void TfStreamServer::alGoalCb(AlServer::GoalHandle gh)
  {
    if (!gh.getGoal())
    {
      gh.setCanceled(AlServer::Result(), "something went wrong, goal canceled");
      return;
    }

    if (gh.getGoal()->update)
      updateStream(gh);
    else
      addStream(gh);
  }
}
