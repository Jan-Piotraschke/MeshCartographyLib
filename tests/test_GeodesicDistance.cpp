// /**
//  * @file        test_GeodesicDistance.cpp
//  * @brief       Testings for the GeodesicDistance class
//  *
//  * @author      Jan-Piotraschke
//  * @date        2023-Sep-08
//  * @version     0.1.0
//  * @license     Apache License 2.0
//  *
//  * @bug         -
//  * @todo        -
//  */

// #include <gtest/gtest.h>
// #include <fstream>
// #include <boost/filesystem.hpp>
// #include <memory>

// #include "GeodesicDistance.h"

// std::string mesh_file_ellipsoid = (PROJECT_PATH_GD / "meshes/sphere.off").string();
// GeodesicDistance geodesic_distance(mesh_file_ellipsoid);

// class GeodesicDistanceTest : public ::testing::Test {
// protected:
//     void SetUp() override {
//     }
// };

// TEST_F(GeodesicDistanceTest, GetAllDistances) {
//     ASSERT_NO_THROW(geodesic_distance.get_all_distances());
// }

// TEST_F(GeodesicDistanceTest, DistanceMatrixSaved) {
//     std::string distance_matrix_path = PROJECT_PATH_GD.string() + "/meshes/data/sphere_distance_matrix_static.csv";
//     std::ifstream infile(distance_matrix_path);
//     ASSERT_TRUE(infile.good());
// }
