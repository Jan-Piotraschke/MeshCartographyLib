#pragma once

#include <vector>
#include <string>
#include <Eigen/Dense>
#include <gtest/gtest_prod.h>

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
    pmp::Vertex find_farthest_vertex();
    void open_mesh_along_seam(const std::vector<pmp::Edge>& seamEdges);

FRIEND_TEST(SurfaceParametrizationTest, FarthestVertex);
};
