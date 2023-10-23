#pragma once

#include <Eigen/Dense>

#include "MeshDefinition.h"
#include "GeodesicDistanceHelperInterface.h"

class DijkstraDistanceHelper : public GeodesicDistanceHelperInterface {
public:
    DijkstraDistanceHelper(fs::path mesh_path);

    Eigen::MatrixXd get_mesh_distance_matrix() override;

    void fill_distance_matrix(
        pmp::SurfaceMesh& mesh,
        Eigen::MatrixXd& distance_matrix,
        pmp::Vertex vertex
    );

private:
    fs::path mesh_path;

    std::vector<double> calculate_geodesic_distance(
        pmp::SurfaceMesh& mesh,
        pmp::Vertex start_vertex
    );
};
