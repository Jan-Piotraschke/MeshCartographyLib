#include "CurvatureCalculator.h"

CurvatureCalculator::CurvatureCalculator(Mesh& mesh) : mesh_(mesh)
{
    curvature_map_ = mesh_.add_property_map<vertex_descriptor, double>("v:curvature", 0.0).first;
}

void CurvatureCalculator::compute_vertex_curvatures()
{
    compute_vertex_gaussian_curvature();
}

void CurvatureCalculator::compute_vertex_gaussian_curvature()
{
    namespace PMP = CGAL::Polygon_mesh_processing;

    // Compute face normals (required for angle calculations)
    auto face_normals = mesh_.add_property_map<_3D::face_descriptor, Kernel::Vector_3>("f:normals").first;
    PMP::compute_face_normals(mesh_, face_normals);

    // Iterate over all vertices to compute Gaussian curvature
    for (const auto& vd : vertices(mesh_))
    {
        double angle_sum = 0.0;
        double area_sum = 0.0;

        // Get halfedges around the vertex
        auto h = halfedge(vd, mesh_);
        if (h == boost::graph_traits<Mesh>::null_halfedge())
            continue; // Skip isolated vertices

        auto done = h;
        do
        {
            if (is_border(h, mesh_))
            {
                h = opposite(next(h, mesh_), mesh_);
                continue;
            }

            // Get the face adjacent to the halfedge
            auto fd = face(h, mesh_);
            if (fd == boost::graph_traits<Mesh>::null_face())
                continue;

            // Compute the angle at the vertex in this face
            auto a = target(next(h, mesh_), mesh_);
            auto b = source(h, mesh_);
            auto c = source(next(h, mesh_), mesh_);

            auto va = mesh_.point(a);
            auto vb = mesh_.point(b);
            auto vc = mesh_.point(c);

            double angle = CGAL::approximate_dihedral_angle(va, vb, vc, va);

            angle_sum += angle;

            // Compute area of the face
            double area = PMP::face_area(fd, mesh_);
            area_sum += area / 3.0; // Attribute one-third of the face area to the vertex

            h = opposite(next(h, mesh_), mesh_);
        } while (h != done);

        double angle_deficit = (2.0 * CGAL_PI) - angle_sum;
        double curvature = angle_deficit / area_sum;

        curvature_map_[vd] = curvature;
    }
}

const CurvatureCalculator::Mesh::Property_map<CurvatureCalculator::vertex_descriptor, double>&
CurvatureCalculator::get_curvature_map() const
{
    return curvature_map_;
}
