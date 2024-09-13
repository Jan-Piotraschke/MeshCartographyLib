#pragma once

#include "MeshDefinition.h"

class SquareBorderHelper
{
  public:
    // Constructor to initialize the mesh, halfedge descriptor, and UV parameter map
    SquareBorderHelper(UV::Mesh& mesh, UV::halfedge_descriptor bhd, _3D::UV_pmap& uvmap);

    // Function to return the Square border parameterizer
    CGAL::Surface_mesh_parameterization::Square_border_uniform_parameterizer_3<UV::Mesh>
    get_square_border_parameterizer();

  private:
    UV::Mesh& mesh;
    UV::halfedge_descriptor bhd;
    _3D::UV_pmap& uvmap;
};
