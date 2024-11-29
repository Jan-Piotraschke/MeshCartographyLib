#pragma once

#include "MeshDefinition.h"

class ParametrizationHelperInterface
{
  public:
    virtual void parameterize_UV_mesh(bool use_uniform_weights = false) = 0;
};
