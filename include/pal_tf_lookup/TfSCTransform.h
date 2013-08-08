#ifndef TFSCTRANSFORM_H
#define TFSCTRANSFORM_H

#include "pal_tf_lookup/TfStreamClient.h"

namespace pal
{
  class TfSCTransform
  {
    friend class TfStreamClient;

    public:
      virtual ~TfSCTransform();

      TfSCTransform(const TfSCTransform&)            = delete;
      TfSCTransform& operator=(const TfSCTransform&) = delete;

    private:
      TfSCTransform(const std::string& key,
          TfStreamClient* psc, TfStreamClient::Callback& cb);

      TfStreamClient*           _psc;
      TfStreamClient::Callback  _cb;
      std::string               _key;
  };
}

#endif
