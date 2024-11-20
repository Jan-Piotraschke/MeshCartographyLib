#pragma once

#include "MeshDefinition.h"

class FaceDistortionHelper
{
  public:
    FaceDistortionHelper(pmp::SurfaceMesh& mesh_open, pmp::SurfaceMesh& mesh_UV);

    double computeFaceDistortion();

  private:
    pmp::SurfaceMesh& mesh_open;
    pmp::SurfaceMesh& mesh_UV;

    double triangle_area(const pmp::SurfaceMesh& mesh, const pmp::Face& f);
};
