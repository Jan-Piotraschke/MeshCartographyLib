#pragma once

#include "MeshDefinition.h"
#include <cmath>
#include <vector>

class AngleDistortionHelper
{
  public:
    AngleDistortionHelper(_3D::Mesh& mesh_open, UV::Mesh& mesh_UV);

    double computeAngleDistortion();

  private:
    _3D::Mesh& mesh_open;
    UV::Mesh& mesh_UV;

    template <typename MeshType>
    std::vector<double> triangle_angles(const MeshType& mesh, const typename MeshType::Face_index& f);

    double computeAngle(const Point_3& A, const Point_3& B, const Point_3& C);
};
