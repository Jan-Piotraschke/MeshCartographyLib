/**
 * @file        test_SurfaceParametrization.cpp
 * @brief       Testings for the SurfaceParametrization class
 *
 * @author      Jan-Piotraschke
 * @date        2023-Jul-18
 * @version     0.1.0
 * @license     Apache License 2.0
 *
 * @bug         -
 * @todo        -
 */

#include <gtest/gtest.h>
#include <fstream>
#include <boost/filesystem.hpp>
#include <memory>

#include <SurfaceParametrization.h>

bool free_boundary = false;
SurfaceParametrization surface_parametrization(free_boundary);
const boost::filesystem::path PROJECT_PATH = MeshCartographyLib_SOURCE_DIR;

class SurfaceParametrizationTest : public ::testing::Test {
protected:
    _3D::Mesh mesh;
    _3D::vertex_descriptor start_node;
    std::vector<_3D::vertex_descriptor> predecessor_pmap;
    std::vector<int> distance;
    _3D::vertex_descriptor target_node;
    std::string mesh_file_path;

    void SetUp() override {
        mesh_file_path = (PROJECT_PATH / "meshes/ellipsoid_x4.off").string();

        // Load the 3D mesh
        std::ifstream in(CGAL::data_file_path(mesh_file_path));
        in >> mesh;

        int start_node_int = 0;
        start_node = _3D::Mesh::Vertex_index(start_node_int);

        // Create vectors to store the predecessors (p) and the distances from the root (d)
        predecessor_pmap = std::vector<_3D::vertex_descriptor>(num_vertices(mesh));
        distance = std::vector<int>(num_vertices(mesh));

        // Calculate the distances from the start node to all other vertices
        surface_parametrization.calculate_distances(mesh, start_node, predecessor_pmap, distance);

        // Find the farthest vertex from the start node
        target_node = surface_parametrization.find_farthest_vertex(mesh, start_node, distance);
    }
};


TEST_F(SurfaceParametrizationTest, MeshNameTestNormalFileName) {
    std::string path = "/path/to/mesh_file.off";
    std::string expected = "mesh_file";
    EXPECT_EQ(surface_parametrization.get_mesh_name(path), expected);
}

TEST_F(SurfaceParametrizationTest, MeshNameTesTFileNameWithMultiplePeriods) {
    std::string path = "/path/to/mesh_file.part1.off";
    std::string expected = "mesh_file.part1";
    EXPECT_EQ(surface_parametrization.get_mesh_name(path), expected);
}

TEST_F(SurfaceParametrizationTest, MeshNameTestNoFileName) {
    std::string path = "/path/to/";
    std::string expected = ".";
    EXPECT_EQ(surface_parametrization.get_mesh_name(path), expected);
}

TEST_F(SurfaceParametrizationTest, MeshNameTest1) {
    auto result = surface_parametrization.set_UV_border_edges(mesh_file_path, start_node);
     std::vector<_3D::edge_descriptor> path_list = result.first;
    // Check the length of the result
    int expected = 44;
    EXPECT_EQ(path_list.size(), expected);
}

TEST_F(SurfaceParametrizationTest, FarthestVertex) {
    int expected = 1798;
    _3D::vertex_descriptor vertex_expected(expected);
    EXPECT_EQ(target_node, vertex_expected);
}

TEST_F(SurfaceParametrizationTest, GetCutLine_length) {
    bool bool_reverse = false;
    auto results = surface_parametrization.get_cut_line(mesh, start_node, target_node, predecessor_pmap, bool_reverse);
    std::vector<_3D::edge_descriptor> path_list = results.first;
    _3D::vertex_descriptor virtual_mesh_start = results.second;

    // Check the length of the result
    int expected = 44;
    EXPECT_EQ(path_list.size(), expected);
}

TEST_F(SurfaceParametrizationTest, CalculateDistances) {
    int node_index = 42;
    int expected_distance = 32;
    _3D::vertex_descriptor node(node_index);
    // Distance from start node to node 42
    EXPECT_EQ(distance[node], expected_distance);
}

TEST_F(SurfaceParametrizationTest, MaxDistance) {
    // Find the max distance
    auto max_iter = std::max_element(distance.begin(), distance.end());
    int max_distance = *max_iter;

    int expected_distance = 45;

    // Distance from start node to node 42
    EXPECT_EQ(max_distance, expected_distance);
}


int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
