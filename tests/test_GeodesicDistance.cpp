/**
 * @file        test_GeodesicDistance.cpp
 * @brief       Testings for the GeodesicDistance class
 *
 * @author      Jan-Piotraschke
 * @date        2023-Sep-08
 * @license     Apache License 2.0
 *
 * @bug         -
 * @todo        -
 */

#include <gtest/gtest.h>
#include <fstream>
#include <filesystem>
#include <memory>
namespace fs = std::filesystem;

#include "SurfaceParametrization/TessellationHelper.h"
#include "SurfaceParametrization/SurfaceParametrization.h"

#include "GeodesicDistance/CachedGeodesicDistanceHelper.h"
#include "GeodesicDistance/CachedTessellationDistanceHelper.h"

fs::path MESH_CARTOGRAPHY = MeshCartographyLib_SOURCE_DIR;
fs::path mesh = MESH_CARTOGRAPHY / "meshes/ellipsoid_x4.off";
fs::path mesh_open = MESH_CARTOGRAPHY / "meshes/ellipsoid_x4_open.off";
fs::path mesh_kachelmuster = MESH_CARTOGRAPHY / "meshes/ellipsoid_x4_uv_kachelmuster.off";

SurfaceParametrization _surface_parametrization = SurfaceParametrization();
Tessellation tessellation(_surface_parametrization);

class GeodesicDistanceTest : public ::testing::Test {
protected:
    Eigen::MatrixXd distance_matrix_tessellation;
    Eigen::MatrixXd distance_matrix_3D;

    void SetUp() override {
        _surface_parametrization.create_uv_surface(mesh, 0);
        // Original Ellipsoid mesh has 4670 vertices
        int n_mesh = 4670;

        std::vector<std::vector<int64_t>> equivalent_vertices = tessellation.create_kachelmuster();

        CachedTessellationDistanceHelper helper = CachedTessellationDistanceHelper(mesh_kachelmuster, equivalent_vertices);
        GeodesicDistanceHelperInterface& geodesic_distance_helper = helper;
        distance_matrix_tessellation = geodesic_distance_helper.get_mesh_distance_matrix();

        // Slice the matrix to only contain the original vertices
        Eigen::MatrixXd slicedMatrix = distance_matrix_tessellation.block(0, 0, n_mesh, n_mesh);
        distance_matrix_tessellation = slicedMatrix;

        CachedGeodesicDistanceHelper helper_3D = CachedGeodesicDistanceHelper(mesh);
        GeodesicDistanceHelperInterface& geodesic_distance_helper_3D = helper_3D;
        distance_matrix_3D = geodesic_distance_helper_3D.get_mesh_distance_matrix();
    }

    double calculateSimilarityPercentage() {
        const double THRESHOLD = 1e-5;

        // Calculate the absolute difference between the two matrices
        Eigen::MatrixXd difference = (distance_matrix_tessellation - distance_matrix_3D).cwiseAbs();

        // Count the number of similar elements (difference below threshold)
        int countSimilar = (difference.array() < THRESHOLD).count();

        int totalElements = distance_matrix_tessellation.rows() * distance_matrix_tessellation.cols();
        return static_cast<double>(countSimilar) / totalElements * 100;
    }
};

TEST_F(GeodesicDistanceTest, DistanceMatrixSaved) {
    fs::path distance_matrix_path = MESH_CARTOGRAPHY / "meshes/data/ellipsoid_x4_distance_matrix_static.csv";
    std::ifstream infile(distance_matrix_path.string());
    ASSERT_TRUE(infile.good());
}

TEST_F(GeodesicDistanceTest, MatricesSimilarity) {
    double similarity = calculateSimilarityPercentage();
    std::cout << "Similarity: " << similarity << "%" << std::endl;
    ASSERT_GT(similarity, 90.0);
}
