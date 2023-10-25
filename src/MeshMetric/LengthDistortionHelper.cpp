/**
 * @file        LengthDistortionHelper.cpp
 * @brief       Calculate the length distortion of the mesh
 *
 * @author      Jan-Piotraschke
 * @date        2023-Sep-27
 * @license     Apache License 2.0
 *
 * @bug         -
 * @todo        -
 */

#include "LengthDistortionHelper.h"

LengthDistortionHelper::LengthDistortionHelper(
    pmp::SurfaceMesh& mesh_open,
    pmp::SurfaceMesh& mesh_UV
)
    : mesh_open(mesh_open),
      mesh_UV(mesh_UV)
{}

// ========================================
// Public Functions
// ========================================

double LengthDistortionHelper::computeLengthDistortion()
{
    double totalDistortion = 0.0;

    for (auto e : mesh_open.edges())
    {
        double lengthOpen = edge_length(mesh_open, e);
        double lengthUV = edge_length(mesh_UV, e);

        totalDistortion += std::fabs(lengthOpen - lengthUV);
    }

    return totalDistortion / mesh_open.n_edges();  // Average length distortion
}


// ========================================
// Private Functions
// ========================================

double LengthDistortionHelper::edge_length(const pmp::SurfaceMesh& mesh, const pmp::Edge& e)
{
    pmp::Vertex v1 = mesh.vertex(e, 0);
    pmp::Vertex v2 = mesh.vertex(e, 1);

    pmp::Point p1 = mesh.position(v1);
    pmp::Point p2 = mesh.position(v2);

    return norm(p1 - p2);  // norm computes the magnitude or length
}
