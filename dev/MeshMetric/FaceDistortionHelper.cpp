#include "FaceDistortionHelper.h"
#include <cmath>

FaceDistortionHelper::FaceDistortionHelper(_3D::Mesh& mesh_open, UV::Mesh& mesh_UV)
    : mesh_open(mesh_open), mesh_UV(mesh_UV)
{
}

double FaceDistortionHelper::computeFaceDistortion()
{
    double totalDistortion = 0.0;

    for (const auto& f : mesh_open.faces())
    {
        // Calculate areas for both the 3D mesh and UV mesh
        double areaOpen = triangle_area(mesh_open, f);

        // Extract the underlying mesh from the UV seam mesh and calculate the area
        const auto& underlying_mesh = mesh_UV.mesh();
        double areaUV = triangle_area(underlying_mesh, f);

        totalDistortion += std::fabs(areaOpen - areaUV);
    }

    return totalDistortion / mesh_open.number_of_faces(); // Average face distortion
}

template <typename MeshType>
double FaceDistortionHelper::triangle_area(const MeshType& mesh, const typename MeshType::Face_index& f)
{
    std::vector<Point_3> pts;

    // Collect the vertices of the face
    for (const auto& halfedge : CGAL::halfedges_around_face(mesh.halfedge(f), mesh))
    {
        pts.push_back(mesh.point(CGAL::target(halfedge, mesh)));
    }

    // Compute vectors representing two sides of the triangle
    Kernel::Vector_3 v1 = pts[1] - pts[0];
    Kernel::Vector_3 v2 = pts[2] - pts[0];

    // Compute the cross product of the two vectors
    Kernel::Vector_3 crossProduct = CGAL::cross_product(v1, v2);

    // The magnitude of the cross product is twice the area of the triangle
    return 0.5 * std::sqrt(crossProduct.squared_length());
}
