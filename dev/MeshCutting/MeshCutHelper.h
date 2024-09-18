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
    MeshCutHelper(_3D::Mesh& mesh, const std::string mesh_3D_file_path, _3D::vertex_descriptor& start_vertex);

    UV::Mesh cut_mesh_open() override;

  private:
    _3D::Mesh& mesh;
    const std::string mesh_3D_file_path;
    _3D::vertex_descriptor& start_vertex;
    std::vector<_3D::vertex_descriptor> cut_line_vertices;

    std::map<_3D::vertex_descriptor, int> get_vertex_neighbors_count() const;
    UV::Mesh open_mesh_along_seam(const std::vector<_3D::edge_descriptor>& seamEdges);
};
