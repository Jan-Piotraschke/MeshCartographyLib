// /**
//  * @file        test_GeodesicDistance.cpp
//  * @brief       Testings for the GeodesicDistance class
//  *
//  * @author      Jan-Piotraschke
//  * @date        2023-Sep-08
//  * @license     Apache License 2.0
//  *
//  * @bug         -
//  * @todo        -
//  */

// #include <gtest/gtest.h>
// #include <fstream>
// #include <boost/filesystem.hpp>
// #include <memory>
// namespace fs = boost::filesystem;

// #include "GeodesicDistance/CachedGeodesicDistanceHelper.h"

// fs::path MESH_CARTOGRAPHY = MeshCartographyLib_SOURCE_DIR;
// fs::path mesh_file_ellipsoid = MESH_CARTOGRAPHY / "meshes/sphere.off";
// CachedGeodesicDistanceHelper geodesic_distance_helper(mesh_file_ellipsoid);

// class GeodesicDistanceTest : public ::testing::Test {
// protected:
//     void SetUp() override {
//     }
// };

// TEST_F(GeodesicDistanceTest, GetAllDistances) {
//     ASSERT_NO_THROW(geodesic_distance_helper.get_mesh_distance_matrix());
// }

// TEST_F(GeodesicDistanceTest, DistanceMatrixSaved) {
//     fs::path distance_matrix_path = MESH_CARTOGRAPHY / "meshes/data/sphere_distance_matrix_static.csv";
//     std::ifstream infile(distance_matrix_path.string());
//     ASSERT_TRUE(infile.good());
// }
