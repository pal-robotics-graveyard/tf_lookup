#ifndef TF_LOOKUP_H
#define TF_LOOKUP_H

#include "pal_tf_lookup/lookupTransform.h"
#include <ros/ros.h>
#include <boost/thread/mutex.hpp>

namespace tf
{
  class TransformListener;
}

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
       * @brief To be called from time to time to check if everything is alright
       */
      virtual void periodicCheck();

    private:
      bool srvLookupTransform(pal_tf_lookup::lookupTransformRequest &req,
          pal_tf_lookup::lookupTransformResponse &res);
      void resetTfListener();

      ros::ServiceServer                     _srvLookupTransform;
      boost::mutex                           _tfListenerMutex;
      std::unique_ptr<tf::TransformListener> _tfListener;
      ros::Time                              _lastTime;
  };
}

#endif // TFLOOKUP_H
