#ifndef TFSCTRANSFORM_H
#define TFSCTRANSFORM_H

#include "tf_lookup/tf_stream_client.h"

namespace tf_lookup
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
          TfStreamClient* psc, const TfStreamClient::Callback& cb);

      TfStreamClient*           _psc;
      TfStreamClient::Callback  _cb;
      std::string               _key;
  };
}

#endif
