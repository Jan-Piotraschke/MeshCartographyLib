#pragma once

#include <vector>
#include <string>
#include <Eigen/Dense>

#include "SurfaceParametrizationHelperInterface.h"

class CutLineHelper : public CutLineHelperInterface {
public:
    CutLineHelper(
        pmp::SurfaceMesh& mesh,
        pmp::Vertex start_vertex
    );

    void cut_mesh_open() override;

private:
    pmp::SurfaceMesh& mesh;
    pmp::Vertex start_vertex;
    std::vector<pmp::Vertex> cut_line_vertices;

    std::map<pmp::Vertex, int> get_vertex_neighbors_count() const;
    void cut_the_mesh(pmp::Vertex current_vertex);
    pmp::Vertex find_farthest_vertex();
    void open_mesh_along_seam(const std::vector<pmp::Edge>& seamEdges);
};
