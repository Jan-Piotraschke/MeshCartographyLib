/**
 * @file        FaceDistortionHelper.cpp
 * @brief       Calculate the face distortion of the mesh
 *
 * @author      Jan-Piotraschke
 * @date        2023-Sep-27
 * @license     Apache License 2.0
 *
 * @bug         -
 * @todo        -
 */

#include "FaceDistortionHelper.h"

FaceDistortionHelper::FaceDistortionHelper(
    pmp::SurfaceMesh& mesh_open,
    pmp::SurfaceMesh& mesh_UV
)
    : mesh_open(mesh_open),
      mesh_UV(mesh_UV)
{}

// ========================================
// Public Functions
// ========================================

double FaceDistortionHelper::computeFaceDistortion()
{
    double totalDistortion = 0.0;

    for (auto f : mesh_open.faces())
    {
        double areaOpen = triangle_area(mesh_open, f);
        double areaUV = triangle_area(mesh_UV, f);
        totalDistortion += std::fabs(areaOpen - areaUV);
    }

    return totalDistortion / mesh_open.n_faces();  // Average face distortion
}


// ========================================
// Private Functions
// ========================================

double FaceDistortionHelper::triangle_area(const pmp::SurfaceMesh& mesh, const pmp::Face& f)
{
    std::vector<pmp::Point> pts;
    for (auto v : mesh.vertices(f))
    {
        pts.push_back(mesh.position(v));
    }

    // Compute the vectors representing two sides of the triangle
    pmp::Point v1 = pts[1] - pts[0];
    pmp::Point v2 = pts[2] - pts[0];

    // Compute the cross product of the two vectors
    pmp::Point crossProduct;
    crossProduct[0] = v1[1]*v2[2] - v1[2]*v2[1];
    crossProduct[1] = v1[2]*v2[0] - v1[0]*v2[2];
    crossProduct[2] = v1[0]*v2[1] - v1[1]*v2[0];

    return 0.5 * norm(crossProduct);  // The magnitude of this cross product is twice the area of the triangle
}
