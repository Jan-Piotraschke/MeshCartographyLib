#pragma once

#include <vector>
#include <string>
#include <Eigen/Dense>

#include "SurfaceParametrizationHelperInterface.h"

class CutLineHelper : public CutLineHelperInterface {
public:
    CutLineHelper(
        const std::string mesh_3D_file_path,
        pmp::Vertex start_vertex
    );

    std::vector<_3D::edge_descriptor> set_UV_border_edges() override;

private:
    const std::string mesh_3D_file_path;
    pmp::Vertex start_vertex;

    std::vector<pmp::Edge> get_cut_line(
        const pmp::SurfaceMesh& mesh,
        pmp::Vertex current_vertex
    );

    pmp::Vertex find_farthest_vertex(
        const pmp::SurfaceMesh& mesh
    );
};
