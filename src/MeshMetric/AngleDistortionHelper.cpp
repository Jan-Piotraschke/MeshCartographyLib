/**
 * @file        AngleDistortionHelper.cpp
 * @brief       Calculate the angle distortion of the mesh
 * @details     The angle is preserved if the first fundamental form is a multiple of the identity, i.e., I(u) = η(u)Id
 *
 * @author      Jan-Piotraschke
 * @date        2023-Oct-25
 * @license     Apache License 2.0
 *
 * @bug         -
 * @todo        -
 */

#include "AngleDistortionHelper.h"

AngleDistortionHelper::AngleDistortionHelper(
    pmp::SurfaceMesh& mesh_open,
    pmp::SurfaceMesh& mesh_UV
)
    : mesh_open(mesh_open),
      mesh_UV(mesh_UV)
{}

// ========================================
// Public Functions
// ========================================

double AngleDistortionHelper::computeAngleDistortion()
{
    double totalDistortion = 0.0;

    for (auto f : mesh_open.faces())
    {
        std::vector<double> anglesOpen = triangle_angles(mesh_open, f);
        std::vector<double> anglesUV = triangle_angles(mesh_UV, f);

        for (int i = 0; i < 3; i++)
        {
            totalDistortion += std::fabs(anglesOpen[i] - anglesUV[i]);
        }
    }

    return totalDistortion / (3 * mesh_open.n_faces());  // Average angle distortion
}


// ========================================
// Private Functions
// ========================================

std::vector<double> AngleDistortionHelper::triangle_angles(const pmp::SurfaceMesh& mesh, const pmp::Face& f)
{
    std::vector<pmp::Point> pts;
    for (auto v : mesh.vertices(f))
    {
        pts.push_back(mesh.position(v));
    }

    std::vector<double> angles(3);
    angles[0] = computeAngle(pts[0], pts[1], pts[2]);
    angles[1] = computeAngle(pts[1], pts[2], pts[0]);
    angles[2] = computeAngle(pts[2], pts[0], pts[1]);

    return angles;
}

// Compute angle given three points A, B, and C
double AngleDistortionHelper::computeAngle(const pmp::Point& A, const pmp::Point& B, const pmp::Point& C)
{
    pmp::Point CA = A - C;
    pmp::Point CB = B - C;

    double dotProduct = CA[0] * CB[0] + CA[1] * CB[1] + CA[2] * CB[2];
    double magnitudeProduct = norm(CA) * norm(CB);  // norm computes the magnitude

    return std::acos(dotProduct / magnitudeProduct);
}
