#pragma once

#include "MeshDefinition.h"

class SurfaceParametrizationHelperInterface {
public:
    virtual std::pair<std::vector<_3D::edge_descriptor>, std::vector<_3D::edge_descriptor>> set_UV_border_edges() = 0;
};
