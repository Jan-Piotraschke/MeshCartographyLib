#pragma once

#include <Eigen/Dense>

#include "MeshDefinition.h"
#include "GeodesicDistanceHelperInterface.h"

class GeodesicDistanceHelper : public GeodesicDistanceHelperInterface {
public:
    GeodesicDistanceHelper(fs::path mesh_path, std::vector<std::vector<int64_t>>& equivalent_vertices);

    Eigen::MatrixXd get_mesh_distance_matrix() override;

private:
    fs::path mesh_path;
    std::vector<std::vector<int64_t>>& equivalent_vertices;

    void fill_distance_matrix(
        pmp::SurfaceMesh& mesh,
        Eigen::MatrixXd& distance_matrix,
        pmp::Vertex vertex
    );

    std::vector<double> calculate_geodesic_distance(
        pmp::SurfaceMesh& mesh,
        pmp::Vertex start_vertex
    );

    Eigen::MatrixXd filter_matrix(Eigen::MatrixXd& distance_matrix);
};
