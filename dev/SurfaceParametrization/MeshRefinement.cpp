#include "MeshRefinement.h"
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polygon_mesh_processing/IO/polygon_mesh_io.h>
#include <CGAL/Polygon_mesh_processing/angle_and_area_smoothing.h>
#include <CGAL/Polygon_mesh_processing/detect_features.h>
#include <CGAL/Polygon_mesh_processing/remesh.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/property_map.h>
#include <iostream>
#include <string>
typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef CGAL::Surface_mesh<K::Point_3> Mesh;
typedef boost::graph_traits<Mesh>::edge_descriptor edge_descriptor;
namespace PMP = CGAL::Polygon_mesh_processing;

MeshRefinement::MeshRefinement(Mesh& mesh, const Mesh::Property_map<vertex_descriptor, double>& curvature_map)
    : mesh_(mesh), curvature_map_(curvature_map)
{
}

#include <CGAL/Polygon_mesh_processing/triangulate_faces.h>

void MeshRefinement::refine_mesh()
{
    // Analyze curvature to set target edge length
    double min_curvature = std::numeric_limits<double>::max();
    double max_curvature = std::numeric_limits<double>::lowest();

    for (const auto& vd : vertices(mesh_))
    {
        double curvature = curvature_map_[vd];
        if (curvature < min_curvature)
            min_curvature = curvature;
        if (curvature > max_curvature)
            max_curvature = curvature;
    }

    // Set target edge length inversely proportional to average curvature
    double average_curvature = (std::abs(min_curvature) + std::abs(max_curvature)) / 2.0;
    double scaling_factor = 1.0; // Adjust as needed

    double target_edge_length = 0.05; // Default value
    if (average_curvature > 0)
    {
        target_edge_length = 1.0 / (scaling_factor * average_curvature);
    }

    // Clamp target edge length to reasonable values
    double min_length = 0.01;
    double max_length = 0.1;
    target_edge_length = std::clamp(target_edge_length, min_length, max_length);

    // Perform isotropic remeshing
    // Mark boundary edges as constrained to preserve the mesh boundary
    auto edge_is_constrained_map = mesh_.add_property_map<EdgeDescriptor, bool>("e:is_constrained", false).first;
    for (auto ed : edges(mesh_))
    {
        edge_is_constrained_map[ed] = is_border(ed, mesh_);
    }

    PMP::isotropic_remeshing(
        faces(mesh_),
        target_edge_length,
        mesh_,
        PMP::parameters::number_of_iterations(1)
            .edge_is_constrained_map(edge_is_constrained_map)
            .relax_constraints(true));
}

// void MeshRefinement::refine_mesh()
// {
//     // Constrain edges with a dihedral angle over 60°
//     typedef boost::property_map<Mesh, CGAL::edge_is_feature_t>::type EIFMap;
//     EIFMap eif = get(CGAL::edge_is_feature, mesh_);
//     PMP::detect_sharp_edges(mesh_, 60, eif);
//     int sharp_counter = 0;
//     for (edge_descriptor e : edges(mesh_))
//         if (get(eif, e))
//             ++sharp_counter;
//     std::cout << sharp_counter << " sharp edges" << std::endl;
//     const unsigned int nb_iterations = 10;
//     std::cout << "Smoothing mesh... (" << nb_iterations << " iterations)" << std::endl;
//     // Smooth with both angle and area criteria + Delaunay flips
//     PMP::angle_and_area_smoothing(
//         mesh_,
//         CGAL::parameters::number_of_iterations(nb_iterations)
//             .use_safety_constraints(false) // authorize all moves
//             .edge_is_constrained_map(eif));
//     CGAL::IO::write_polygon_mesh("mesh_smoothed.off", mesh_, CGAL::parameters::stream_precision(17));
// }
