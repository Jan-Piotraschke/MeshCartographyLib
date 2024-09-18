#pragma once

#include <Eigen/Dense>
#include <string>
#include <vector>

#include "MeshCuttingHelperInterface.h"
#include "MeshDefinition.h"
#include "SurfaceParametrizationHelperInterface.h"

class MeshCutHelper : public MeshCuttingHelperInterface
{
  public:
    MeshCutHelper(_3D::Mesh& mesh, _3D::vertex_descriptor& start_vertex);

    UV::Mesh cut_mesh_open(const std::vector<_3D::edge_descriptor> calc_edges) override;

  private:
    _3D::Mesh& mesh;
    _3D::vertex_descriptor& start_vertex;
    std::vector<_3D::vertex_descriptor> cut_line_vertices;

    std::map<_3D::vertex_descriptor, int> get_vertex_neighbors_count() const;
    void open_mesh_along_seam(const std::vector<_3D::edge_descriptor>& seamEdges);
};
