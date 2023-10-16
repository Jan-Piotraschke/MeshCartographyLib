/**
 * @file        GaussianCutLineHelper.cpp
 * @brief       Create a cut line based on the Gaussian curvature
 *
 * @author      Jan-Piotraschke
 * @date        2023-Oct-16
 * @license     Apache License 2.0
 *
 * @bug         -
 * @todo        -
 */

#include "GaussianCutLineHelper.h"
#include "pmp/algorithms/curvature.h"

GaussianCutLineHelper::GaussianCutLineHelper(
    pmp::SurfaceMesh& mesh,
    pmp::Vertex& start_vertex
)
    : mesh(mesh),
    start_vertex(start_vertex)
{};

// ========================================
// Public Functions
// ========================================

/**
* The size of the path_list multiplied with 2 is the number of vertices on the border of the UV mesh
*
* So, if you want something like an inverse 'Poincaré disk' you have to really shorten the path_list
* The same is true if you reverse the logic: If you create a spiral-like seam edge path, your mesh will results in something like a 'Poincaré disk'
*/
std::vector<pmp::Edge> GaussianCutLineHelper::get_gaussian_cutline() {
    auto current_vertex = get_gaussian_vertex();

    pmp::VertexProperty<pmp::Scalar> distance_pmp = mesh.get_vertex_property<pmp::Scalar>("geodesic:distance");

    std::vector<pmp::Edge> edge_path;

    while (current_vertex != start_vertex) {

        double min_distance = std::numeric_limits<double>::infinity();
        pmp::Vertex next_vertex;

        for (auto neighbor_vertex : mesh.vertices(current_vertex)) {
            double d = distance_pmp[neighbor_vertex];
            if (d < min_distance) {
                min_distance = d;
                next_vertex = neighbor_vertex;
            }
        }

        for (auto halfedge : mesh.halfedges(current_vertex)) {
            pmp::Vertex neighbor_vertex = mesh.to_vertex(halfedge);
            if (neighbor_vertex == next_vertex) {
                edge_path.push_back(mesh.edge(halfedge));
                break;
            }
        }

        current_vertex = next_vertex;
    }

    std::reverse(edge_path.begin(), edge_path.end());

    if (edge_path.size() % 2 != 0) {
        edge_path.pop_back();
    }

    // To check if the edges are valid:
    for (const auto& edge : edge_path) {
        if (!mesh.is_valid(edge)) {
            std::cerr << "Invalid edge found" << std::endl;
        }
    }

    return edge_path;
}



// ========================================
// Private Functions
// ========================================

/**
 * @brief Based on the Gaussian curvature K = k1*k2 (k1 and k2 are the principal curvatures (minimum and maximum normal curvature))
*/
pmp::Vertex GaussianCutLineHelper::get_gaussian_vertex(){
    // Compute vertex curvature
    pmp::curvature(mesh, pmp::Curvature::gauss, 1);
    auto curvatures = mesh.get_vertex_property<pmp::Scalar>("v:curv");

    // Sort vertices based on curvature
    std::vector<pmp::Vertex> sorted_vertices;
    for (auto v : mesh.vertices()) {
        sorted_vertices.push_back(v);
    }
    std::sort(sorted_vertices.begin(), sorted_vertices.end(), [&](const pmp::Vertex& a, const pmp::Vertex& b) {
        return curvatures[a] > curvatures[b];
    });

    pmp::Vertex first_high_curvature_vertex = sorted_vertices[0];

    // Compute geodesic distance from vertex with highest curvature
    std::vector<pmp::Vertex> seeds{first_high_curvature_vertex};
    pmp::geodesics(mesh, seeds);
    pmp::VertexProperty<pmp::Scalar> distance = mesh.get_vertex_property<pmp::Scalar>("geodesic:distance");

    // Find another vertex with high curvature and farthest from first_high_curvature_vertex
    pmp::Scalar max_distances = std::numeric_limits<pmp::Scalar>::lowest();
    pmp::Vertex second_high_curvature_vertex;

    for (size_t i = 1; i < sorted_vertices.size(); ++i) {
        auto v = sorted_vertices[i];
        if (distance[v] > max_distances) {
            max_distances = distance[v];
            second_high_curvature_vertex = v;
        }
    }
    start_vertex = first_high_curvature_vertex;
    pmp::Vertex target_vertex = second_high_curvature_vertex;

    return target_vertex;
}
