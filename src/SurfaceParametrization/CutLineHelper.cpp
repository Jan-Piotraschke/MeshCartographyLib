/**
 * @file        CutLineHelper.cpp
 * @brief       Calculate the cut line of the mesh
 *
 * @author      Jan-Piotraschke
 * @date        2023-Sep-22
 * @license     Apache License 2.0
 *
 * @bug         -
 * @todo        -
 */

#include "CutLineHelper.h"

CutLineHelper::CutLineHelper(
    pmp::SurfaceMesh& mesh,
    pmp::Vertex start_vertex
)
    : mesh(mesh),
    start_vertex(start_vertex)
{};

// ========================================
// Public Functions
// ========================================

/**
* @brief Calculate the border of the mesh
*/
void CutLineHelper::cut_mesh_open(){
    // Compute geodesic distance from first vertex using breadth first search
    std::vector<pmp::Vertex> seeds{start_vertex};
    pmp::geodesics(mesh, seeds);

    // Find the target node (farthest from the start node)
    pmp::Vertex target_node = find_farthest_vertex();

    // Get the edges of the path between the start and the target node
    std::vector<pmp::Edge> path_list = get_cut_line(target_node);

    // Open the mesh along the path

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
std::vector<pmp::Edge> CutLineHelper::get_cut_line(pmp::Vertex current_vertex){
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

    open_mesh_along_seam(mesh, path, path_edges);
    pmp::write(mesh, "path_to_opened_mesh.off");

    return path_edges;
}


/**
 * @brief Find the farthest vertex from a given start vertex
*/
pmp::Vertex CutLineHelper::find_farthest_vertex(){
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


void CutLineHelper::open_mesh_along_seam(const std::vector<pmp::Vertex>& seamVertices, const std::vector<pmp::Edge>& seamEdges)
{
    if (seamVertices.size() < 2) return; // We need at least two vertices to define a seam

    std::vector<pmp::Vertex> duplicatedVertices;
    for (auto v : seamVertices) {
        pmp::Point pos = mesh.position(v);
        pmp::Vertex newVertex = mesh.add_vertex(pos);
        duplicatedVertices.push_back(newVertex);
    }

    for (size_t i = 0; i < seamEdges.size(); i++) {
        pmp::Edge e = seamEdges[i];
        pmp::Halfedge he = mesh.halfedge(e, 0); // Get one of the halfedges of the edge
        pmp::Vertex v0 = mesh.from_vertex(he); // Vertex at the start of this halfedge
        pmp::Vertex v1 = mesh.to_vertex(he);   // Vertex at the end of this halfedge

        // Find the corresponding duplicated vertices
        pmp::Vertex dup_v0 = duplicatedVertices[std::distance(seamVertices.begin(), std::find(seamVertices.begin(), seamVertices.end(), v0))];
        pmp::Vertex dup_v1 = duplicatedVertices[std::distance(seamVertices.begin(), std::find(seamVertices.begin(), seamVertices.end(), v1))];

        // Adjust the connectivity of adjacent faces
        // Replace the vertex v0 with its duplicate for the face adjacent to he.
        mesh.set_vertex(he, dup_v0);

        // Likewise, for the opposite halfedge replace v1 with its duplicate
        mesh.set_vertex(mesh.opposite_halfedge(he), dup_v1);

        // Remove the original seam edge
        mesh.delete_edge(e);
    }

    if (!has_boundary(mesh)) {
        std::cout << "Mesh is still closed!" << std::endl;
    }
}


bool CutLineHelper::has_boundary()
{
    for (auto v : mesh.vertices())
        if (mesh.is_boundary(v))
            return true;
    return false;
}
