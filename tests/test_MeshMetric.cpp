/**
 * @file        test_MeshMetric.cpp
 * @brief       Testing the DistortionHelper classes
 *
 * @author      Jan-Piotraschke
 * @date        2023-Sep-08
 * @license     Apache License 2.0
 *
 * @bug         -
 * @todo        - investigate the behaviour of TestAngleDistortionRotatedUV
 */

#include <gtest/gtest.h>
#include <fstream>
#include <filesystem>
#include <memory>
namespace fs = std::filesystem;

#include "SurfaceParametrization/SurfaceParametrization.h"
#include "SurfaceParametrization/TessellationHelper.h"

#include "MeshMetric/AngleDistortionHelper.h"
#include "MeshMetric/FaceDistortionHelper.h"
#include "MeshMetric/LengthDistortionHelper.h"

fs::path _MESH_CARTOGRAPHY = MeshCartographyLib_SOURCE_DIR;
fs::path mesh_path = _MESH_CARTOGRAPHY / "meshes/ellipsoid_x4.off";
fs::path mesh_open_path = _MESH_CARTOGRAPHY / "meshes/ellipsoid_x4_open.off";
fs::path mesh_uv_path = _MESH_CARTOGRAPHY / "meshes/ellipsoid_x4_uv.off";

SurfaceParametrization surface_parametrization_metric = SurfaceParametrization();
Tessellation tessellation_metric(surface_parametrization_metric);

class MeshMetricTest : public ::testing::Test {
protected:
    static pmp::SurfaceMesh mesh_open;
    static pmp::SurfaceMesh mesh_uv;

    static void SetUpTestCase() {
         surface_parametrization_metric.create_uv_surface(mesh_path, 0);

        // Load the meshes
        pmp::read_off(mesh_open, mesh_open_path.string());
        pmp::read_off(mesh_uv, mesh_uv_path.string());
    }

    // If you need any per-test setup
    void SetUp() override {}
};

pmp::SurfaceMesh MeshMetricTest::mesh_open;
pmp::SurfaceMesh MeshMetricTest::mesh_uv;

TEST_F(MeshMetricTest, TestAngleDistortion)
{
    AngleDistortionHelper angleDistHelper(mesh_open, mesh_open);
    double angleDistortion = angleDistHelper.computeAngleDistortion();

    // Assuming no distortion would give a value of 0
    ASSERT_EQ(angleDistortion, 0.0);
}

TEST_F(MeshMetricTest, TestLengthDistortion)
{
    LengthDistortionHelper lengthDistHelper(mesh_open, mesh_open);
    double lengthDistortion = lengthDistHelper.computeLengthDistortion();

    ASSERT_EQ(lengthDistortion, 0.0);
}

TEST_F(MeshMetricTest, TestFaceDistortion)
{
    FaceDistortionHelper faceDistHelper(mesh_open, mesh_open);
    double faceDistortion = faceDistHelper.computeFaceDistortion();

    ASSERT_EQ(faceDistortion, 0.0);
}

TEST_F(MeshMetricTest, TestAngleDistortionUV)
{
    AngleDistortionHelper angleDistHelper(mesh_uv, mesh_uv);
    double angleDistortion = angleDistHelper.computeAngleDistortion();

    ASSERT_EQ(angleDistortion, 0.0);
}

TEST_F(MeshMetricTest, TestLengthDistortionUV)
{
    LengthDistortionHelper lengthDistHelper(mesh_uv, mesh_uv);
    double lengthDistortion = lengthDistHelper.computeLengthDistortion();

    ASSERT_EQ(lengthDistortion, 0.0);
}

TEST_F(MeshMetricTest, TestFaceDistortionUV)
{
    FaceDistortionHelper faceDistHelper(mesh_uv, mesh_uv);
    double faceDistortion = faceDistHelper.computeFaceDistortion();

    ASSERT_EQ(faceDistortion, 0.0);
}
