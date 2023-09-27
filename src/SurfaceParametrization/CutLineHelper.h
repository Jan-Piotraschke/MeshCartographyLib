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
    bool has_boundary();
    void openMeshAlongSeam(const std::vector<pmp::Vertex>& seamVertices, const std::vector<pmp::Edge>& seamEdges);

private:
    pmp::SurfaceMesh& mesh;
    pmp::Vertex start_vertex;

    std::vector<pmp::Edge> get_cut_line(pmp::Vertex current_vertex);
    pmp::Vertex find_farthest_vertex();
};
