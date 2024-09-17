#pragma once

#include <vector>
#include <string>
#include <Eigen/Dense>

#include "SurfaceParametrizationHelperInterface.h"
#include "MeshCuttingHelperInterface.h"
#include "MeshDefinition.h"

class MeshCutHelper : public MeshCuttingHelperInterface {
public:
    MeshCutHelper(_3D::Mesh& mesh, _3D::vertex_descriptor& start_vertex);

    void cut_mesh_open() override;

private:
    _3D::Mesh& mesh;
    _3D::vertex_descriptor& v;
    std::vector<_3D::vertex_descriptor> cut_line_vertices;

    std::map<_3D::vertex_descriptor, int> get_vertex_neighbors_count() const;
    void open_mesh_along_seam(const std::vector<_3D::edge_descriptor>& seamEdges);
};
