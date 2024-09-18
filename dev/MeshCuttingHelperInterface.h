#pragma once

#include "MeshDefinition.h"
#include <Eigen/Dense>
#include <vector>

class MeshCuttingHelperInterface
{
  public:
    virtual UV::Mesh cut_mesh_open(const std::vector<_3D::edge_descriptor> calc_edges) = 0;
};
