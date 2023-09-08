/**
 * @file        test_EuclideanTiling.cpp
 * @brief       Testings for the EuclideanTiling class
 *
 * @author      Jan-Piotraschke
 * @date        2023-Sep-08
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
#include <Eigen/Dense>
#include <SurfaceParametrization.h>
#include <EuclideanTiling.h>

bool free_boundary_tiling = false;
SurfaceParametrization surface_parametrization_tiling(free_boundary_tiling);

class EuclideanTilingTest : public ::testing::Test {
protected:
    Eigen::Matrix<double, Eigen::Dynamic, 2> r_UV;
    Eigen::Matrix<double, Eigen::Dynamic, 2> r_UV_old;
    Eigen::VectorXi n;
    EuclideanTiling euclidean_tiling;

public:
    EuclideanTilingTest() : euclidean_tiling(surface_parametrization_tiling, r_UV, r_UV_old, n) {}

    void SetUp() override {
        r_UV.resize(3, 2);
        r_UV << 1.5, 1.5,
                1.1, 1.5,
                1.5, 1.7;

        r_UV_old.resize(3, 2);
        r_UV_old << 0.5, 0.6,
                    0.7, 0.8,
                    0.9, 0.4;

        n.resize(3);
        n << 80, 120, 42;
    }
};

TEST_F(EuclideanTilingTest, TestOppositeSeamEdgesSquareBorder) {
    euclidean_tiling.opposite_seam_edges_square_border();

    const double EPSILON = 1e-9;

    ASSERT_NEAR(r_UV(0, 0), 0.5, EPSILON);
    ASSERT_NEAR(r_UV(0, 1), 0.5, EPSILON);
    ASSERT_NEAR(r_UV(1, 0), 0.1, EPSILON);
    ASSERT_NEAR(r_UV(1, 1), 0.5, EPSILON);
    ASSERT_NEAR(r_UV(2, 0), 0.5, EPSILON);
    ASSERT_NEAR(r_UV(2, 1), 0.7, EPSILON);
}
