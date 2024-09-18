#pragma once

#include "MeshDefinition.h"
#include <cmath>
#include <vector>

class LengthDistortionHelper
{
  public:
    LengthDistortionHelper(_3D::Mesh& mesh_open, UV::Mesh& mesh_UV);

    double computeLengthDistortion();

  private:
    _3D::Mesh& mesh_open;
    UV::Mesh& mesh_UV;

    template <typename MeshType>
    double edge_length(const MeshType& mesh, const typename MeshType::Edge_index& e);
};
