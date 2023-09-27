#pragma once

#include "MeshDefinition.h"

class CutLineHelperInterface {
public:
    virtual void cut_mesh_open() = 0;
};

class ParametrizationHelperInterface {
public:
    virtual void parameterize_UV_mesh(bool use_uniform_weights = false) = 0;
};
