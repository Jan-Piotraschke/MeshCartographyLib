#pragma once

#include <Eigen/Dense>
#include <string>
#include <vector>

#include "MeshCuttingHelperInterface.h"
#include "SurfaceParametrizationHelperInterface.h"

class MeshCutHelper : public MeshCuttingHelperInterface
{
  public:
    MeshCutHelper(pmp::SurfaceMesh& mesh, pmp::Vertex& start_vertex);

    void cut_mesh_open() override;

  private:
    pmp::SurfaceMesh& mesh;
    pmp::Vertex& start_vertex;
    std::vector<pmp::Vertex> cut_line_vertices;

    std::map<pmp::Vertex, int> get_vertex_neighbors_count() const;
    void open_mesh_along_seam(const std::vector<pmp::Edge>& seamEdges);
};
