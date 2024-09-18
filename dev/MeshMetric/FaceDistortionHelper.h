#pragma once

#include "MeshDefinition.h"
#include <vector>

class FaceDistortionHelper
{
  public:
    FaceDistortionHelper(_3D::Mesh& mesh_open, UV::Mesh& mesh_UV);

    double computeFaceDistortion();

  private:
    _3D::Mesh& mesh_open;
    UV::Mesh& mesh_UV;

    template <typename MeshType>
    double triangle_area(const MeshType& mesh, const typename MeshType::Face_index& f);
};
