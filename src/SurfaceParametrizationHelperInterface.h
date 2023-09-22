#pragma once

#include "MeshDefinition.h"

class CutLineHelperInterface {
public:
    virtual std::pair<std::vector<_3D::edge_descriptor>, std::vector<_3D::edge_descriptor>> set_UV_border_edges() = 0;
};

class ParametrizationHelperInterface {
public:
    virtual SMP::Error_code parameterize_UV_mesh() = 0;
};
