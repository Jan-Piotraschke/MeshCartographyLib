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

// fs::path mesh = MESH_CARTOGRAPHY / "meshes/bear.off";
// fs::path mesh_open = MESH_CARTOGRAPHY / "meshes/bear_open.off";
// fs::path mesh_kachelmuster = MESH_CARTOGRAPHY / "meshes/bear_uv_kachelmuster.off";

// fs::path mesh = MESH_CARTOGRAPHY / "meshes/bunny.off";
// fs::path mesh_open = MESH_CARTOGRAPHY / "meshes/bunny_open.off";
// fs::path mesh_kachelmuster = MESH_CARTOGRAPHY / "meshes/bunny_uv_kachelmuster.off";

SurfaceParametrization _surface_parametrization = SurfaceParametrization();
Tessellation tessellation(_surface_parametrization);

class GeodesicDistanceTest : public ::testing::Test {
protected:
    static Eigen::MatrixXd distance_matrix_tessellation;
    static Eigen::MatrixXd distance_matrix_3D;

    static void SetUpTestCase() {
         _surface_parametrization.create_uv_surface(mesh, 0);
        // Original Ellipsoid mesh has 4670 vertices
        // Bear: 13826
        // Bunny: 34835
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

    // If you need any per-test setup
    void SetUp() override {}

    static void TearDownTestCase() {
        distance_matrix_tessellation.resize(0, 0);
        distance_matrix_3D.resize(0, 0);
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

// Static members initialization
Eigen::MatrixXd GeodesicDistanceTest::distance_matrix_tessellation;
Eigen::MatrixXd GeodesicDistanceTest::distance_matrix_3D;

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

TEST_F(GeodesicDistanceTest, DistanceMatrixNonNegativity) {
    // Count the number of negative elements in both matrices
    int countNegativeTessellation = (distance_matrix_tessellation.array() < 0.0).count();
    int countNegative3D = (distance_matrix_3D.array() < 0.0).count();

    ASSERT_EQ(countNegativeTessellation, 0);
    ASSERT_EQ(countNegative3D, 0);
}

TEST_F(GeodesicDistanceTest, DistanceMatrixNoNaN) {
    // Check if any element is NaN
    int countNaNInTessellation = (distance_matrix_tessellation.array().isNaN()).count();
    int countNaNIn3D = (distance_matrix_3D.array().isNaN()).count();

    ASSERT_EQ(countNaNInTessellation, 0);
    ASSERT_EQ(countNaNIn3D, 0);
}

TEST_F(GeodesicDistanceTest, DistanceMatrixDiagonalCheck) {
    for (int i = 0; i < distance_matrix_tessellation.rows(); ++i) {
        ASSERT_DOUBLE_EQ(distance_matrix_tessellation(i, i), 0.0);
        ASSERT_DOUBLE_EQ(distance_matrix_3D(i, i), 0.0);
    }
}
