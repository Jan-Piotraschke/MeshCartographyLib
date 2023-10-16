/**
 * @file        test_SurfaceParametrization.cpp
 * @brief       Testings for the SurfaceParametrization and CutLineHelper classes
 *
 * @author      Jan-Piotraschke
 * @date        2023-Jul-18
 * @license     Apache License 2.0
 *
 * @bug         -
 * @todo        -
 */

#include <gtest/gtest.h>
#include <fstream>
#include <boost/filesystem.hpp>
namespace fs = boost::filesystem;
#include <memory>
#include <random>

#include <SurfaceParametrization/SurfaceParametrization.h>
#include <SurfaceParametrization/CutLineHelper.h>

const fs::path PROJECT_PATH = MeshCartographyLib_SOURCE_DIR;

SurfaceParametrization surface_parametrization;

class SurfaceParametrizationTest : public ::testing::Test {
protected:
    static pmp::SurfaceMesh mesh;
    static pmp::Vertex start_node;
    static std::string mesh_file_path;

    static void SetUpTestCase() {
        mesh_file_path = (PROJECT_PATH / "meshes/ellipsoid_x4.off").string();
        pmp::read_off(mesh, mesh_file_path);

        int start_node_int = 0;
        start_node = pmp::Vertex(start_node_int);

        // Create UV surface
        auto result = surface_parametrization.create_uv_surface(mesh_file_path, start_node_int);
    }

    // If you need any per-test setup
    void SetUp() override {}
};

// Static members initialization
pmp::SurfaceMesh SurfaceParametrizationTest::mesh;
pmp::Vertex SurfaceParametrizationTest::start_node;
std::string SurfaceParametrizationTest::mesh_file_path;

// SurfaceParametrization tests
TEST_F(SurfaceParametrizationTest, MeshNameTestNormalFileName) {
    std::string path = "/path/to/mesh_file.off";
    std::string expected = "mesh_file";
    EXPECT_EQ(surface_parametrization.get_mesh_name(path), expected);
}

TEST_F(SurfaceParametrizationTest, CheckPointInPolygon) {
    // 1. Points clearly inside the square
    std::vector<Point_2_eigen> inside_points = {
        {0.5, 0.5},
        {0.25, 0.25},
        {0.75, 0.75},
        {0.4, 0.6}
    };
    for (const auto& point : inside_points) {
        EXPECT_TRUE(surface_parametrization.check_point_in_polygon(point));
    }
}

TEST_F(SurfaceParametrizationTest, CheckPointOutsidePolygon) {
    // 2. Points clearly outside the square
    std::vector<Point_2_eigen> outside_points = {
        {-0.1, 0.5},
        {1.1, 0.5},
        {0.5, -0.1},
        {0.5, 1.1}
    };
    for (const auto& point : outside_points) {
        EXPECT_FALSE(surface_parametrization.check_point_in_polygon(point));
    }
}

TEST_F(SurfaceParametrizationTest, CheckPointOnBoundary) {
    // 3. Points right on the boundary
    std::vector<Point_2_eigen> boundary_points = {
        {0, 0},
        {1, 0},
        {0, 1},
        {1, 1},
        {0.5, 0},
        {0.5, 1},
        {0, 0.5},
        {1, 0.5}
    };
    for (const auto& point : boundary_points) {
        EXPECT_TRUE(surface_parametrization.check_point_in_polygon(point));
    }
}

TEST_F(SurfaceParametrizationTest, CheckPointInPolygonGrid) {
    // 4. Grid of points covering the region
    double step = 0.01;
    for (double x = -0.2; x <= 1.2; x += step) {
        for (double y = -0.2; y <= 1.2; y += step) {
            bool is_inside = (x >= 0 && x <= 1 && y >= 0 && y <= 1);
            EXPECT_EQ(surface_parametrization.check_point_in_polygon({x, y}), is_inside);
        }
    }
}

TEST_F(SurfaceParametrizationTest, CheckPointInPolygonRandom) {
    // 5. Randomly generated points
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(-0.2, 1.2);  // random numbers between -0.2 to 1.2
    for (int i = 0; i < 1000; ++i) {
        double x = dis(gen);
        double y = dis(gen);
        bool is_inside = (x >= 0 && x <= 1 && y >= 0 && y <= 1);
        EXPECT_EQ(surface_parametrization.check_point_in_polygon({x, y}), is_inside);
    }
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
