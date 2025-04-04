#pragma once

#include "MeshDefinition/pmp.h"

class GaussianCutLineHelper
{
  public:
    GaussianCutLineHelper(pmp::SurfaceMesh& mesh, pmp::Vertex& start_vertex);

    std::vector<pmp::Edge> get_gaussian_cutline();

  private:
    pmp::SurfaceMesh& mesh;
    pmp::Vertex& start_vertex;

    pmp::Vertex get_gaussian_vertex();
    void validate_edges(const std::vector<pmp::Edge>& edge_path);
};
