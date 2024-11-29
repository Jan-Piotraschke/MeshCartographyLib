#pragma once

#include "MeshDefinition/pmp.h"

class ParametrizationHelperInterface
{
  public:
    virtual void parameterize_UV_mesh(bool use_uniform_weights = false) = 0;
};
