#include "pal_tf_lookup/TfSCTransform.h"

namespace pal
{
  TfSCTransform::TfSCTransform(const std::string& key,
      TfStreamClient* psc, TfStreamClient::Callback& cb)
    : _psc(psc), _cb(cb), _key(key)
  {
    _psc->_transforms[key] = this;
    _psc->updateTransforms();
  }

  TfSCTransform::~TfSCTransform()
  {
    _psc->_transforms.erase(_key);
    _psc->updateTransforms();
  }
}
