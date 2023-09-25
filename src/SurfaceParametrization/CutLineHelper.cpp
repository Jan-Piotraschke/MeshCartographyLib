/**
 * @file        CutLineHelper.cpp
 * @brief       Calculate the cut line of the mesh
 *
 * @author      Jan-Piotraschke
 * @date        2023-Sep-22
 * @license     Apache License 2.0
 *
 * @bug         -
 * @todo        replace calculate_distances() with a function within the GeodesicDistance class
 */

#include "CutLineHelper.h"

CutLineHelper::CutLineHelper(
    const std::string mesh_3D_file_path,
    _3D::vertex_descriptor start_node
)
    : mesh_3D_file_path(mesh_3D_file_path),
    start_node(start_node)
{};

// ========================================
// Public Functions
// ========================================

/**
* @brief Calculate the border of the mesh
*
* @info: Unittested
*/
std::vector<_3D::edge_descriptor> CutLineHelper::set_UV_border_edges(){
    _3D_pmp::Mesh mesh_pmp;
    pmp::read_off(mesh_pmp, mesh_3D_file_path);

    // Compute geodesic distance from first vertex using breadth first search
    std::vector<pmp::Vertex> seeds{pmp::Vertex(0)};
    pmp::geodesics(mesh_pmp, seeds);

    // Find the target node (farthest from the start node)
    pmp::Vertex target_node = find_farthest_vertex(mesh_pmp);
    pmp::Vertex start_vertex = seeds[0];

    // Get the edges of the path between the start and the target node
    std::vector<pmp::Edge> path_list = get_cut_line(mesh_pmp, start_vertex, target_node);

    // ! Temp: convert path_list to _3D::edge_descriptor
    _3D::Mesh mesh;
    std::ifstream in(CGAL::data_file_path(mesh_3D_file_path));
    in >> mesh;

    std::vector<_3D::edge_descriptor> path_list_3D;
    for (auto e : path_list) {
        _3D::vertex_descriptor v1(mesh_pmp.vertex(e, 0).idx());
        _3D::vertex_descriptor v2(mesh_pmp.vertex(e, 1).idx());

        std::pair<_3D::edge_descriptor, bool> edge_pair = edge(v1, v2, mesh);
        path_list_3D.push_back(edge_pair.first);
    }

    return path_list_3D;
}



// ========================================
// Private Functions
// ========================================

/**
* @brief Create a path of vertices from the start node to the target node
*
* The size of the path_list multiplied with 2 is the number of vertices on the border of the UV mesh
*
* So, if you want something like an inverse 'Poincaré disk' you have to really shorten the path_list
* The same is true if you reverse the logic: If you create a spiral-like seam edge path, your mesh will results in something like a 'Poincaré disk'
*/
std::vector<pmp::Edge> CutLineHelper::get_cut_line(
    const _3D_pmp::Mesh& mesh,
    const pmp::Vertex& start_vertex,
    pmp::Vertex current_vertex
){
    pmp::VertexProperty<pmp::Scalar> distance_pmp = mesh.get_vertex_property<pmp::Scalar>("geodesic:distance");

    std::vector<pmp::Vertex> path;
    std::vector<pmp::Edge> path_edges;

    while (current_vertex != start_vertex) {
        path.push_back(current_vertex);

        double min_distance = std::numeric_limits<double>::infinity();
        pmp::Vertex next_vertex;

        for (auto neighbor_vertex : mesh.vertices(current_vertex)) {
            double d = distance_pmp[neighbor_vertex];
            if (d < min_distance) {
                min_distance = d;
                next_vertex = neighbor_vertex;
            }
        }

        current_vertex = next_vertex;
    }

    path.push_back(start_vertex);

    for (size_t i = 0; i < path.size() - 1; ++i) {
        auto edge = mesh.find_edge(path[i], path[i + 1]);
        if (edge.is_valid()) {
            path_edges.push_back(edge);
        }
    }

    if (path_edges.size() % 2 != 0) {
        path_edges.pop_back();
    }

    return path_edges;
}



/**
 * @brief Find the farthest vertex from a given start vertex
 *
 * @info: Unittested
*/
pmp::Vertex CutLineHelper::find_farthest_vertex(
    const _3D_pmp::Mesh mesh
){
    pmp::Scalar max_distances(0);
    pmp::VertexProperty<pmp::Scalar> distance = mesh.get_vertex_property<pmp::Scalar>("geodesic:distance");
    pmp::Vertex target_node;

    for (auto v : mesh.vertices()) {
        if (distance[v] > max_distances) {
            max_distances = distance[v];
            target_node = v;
        }
    }

    return target_node;
}
