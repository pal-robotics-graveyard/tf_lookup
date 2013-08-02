#ifndef TF_LOOKUP_H
#define TF_LOOKUP_H

#include "pal_tf_lookup/lookupTransform.h"
#include "pal_tf_lookup/TfLookupAction.h"
#include <ros/ros.h>
#include <boost/thread/mutex.hpp>
#include <boost/thread.hpp>
#include <actionlib/server/action_server.h>
#include <queue>

namespace tf
{
  class TransformListener;
  class StampedTransform;
}

namespace ros
{ class Time; }

namespace pal
{
  class TfLookup
  {
    public:
      TfLookup();
      virtual ~TfLookup();
      /**
       * @brief Advertises the services the plugin provides so they can be
       *        called from any ros node.
       */
      virtual void advertiseServices(ros::NodeHandle &n);

      /**
       * @brief Starts the Actionlib server that allows asynchronous
       *        tf lookup calls
       */
      virtual void startAlServer(ros::NodeHandle& n);

      /**
       * @brief To be called from time to time to check if everything is alright
       */
      virtual void periodicCheck();

    private:
      typedef actionlib::ActionServer<pal_tf_lookup::TfLookupAction> AlServer;

      bool srvLookupTransform(pal_tf_lookup::lookupTransformRequest &req,
          pal_tf_lookup::lookupTransformResponse &res);
      void resetTfListener();
      void alExecute();
      bool lookupTransform(const std::string& target,
          const std::string& source, const ros::Time& time,
          tf::StampedTransform& trans);
      void oneAtATime(AlServer::GoalHandle goal);

      ros::ServiceServer                     _srvLookupTransform;
      boost::mutex                           _tfListenerMutex;
      std::unique_ptr<tf::TransformListener> _tfListener;
      ros::Time                              _lastTime;

      std::unique_ptr<AlServer>              _al_server;
      pal_tf_lookup::TfLookupResult          _al_result;
      std::queue<AlServer::GoalHandle>       _al_goals;
      boost::condition_variable              _al_cond;
      boost::mutex                           _al_mutex;
      std::unique_ptr<boost::thread>         _al_thread;
  };
}

#endif // TFLOOKUP_H
