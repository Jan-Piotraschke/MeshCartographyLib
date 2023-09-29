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
    cut_the_mesh(target_node);
    std::cout << "Mesh cut open!" << std::endl;
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
void CutLineHelper::cut_the_mesh(pmp::Vertex current_vertex){
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
    std::cout << "Path edges: " << path_edges.size() << std::endl;
    open_mesh_along_seam(path, path_edges);
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
#include <unordered_map>
#include <algorithm>

void CutLineHelper::open_mesh_along_seam(const std::vector<pmp::Vertex>& seamVertices, const std::vector<pmp::Edge>& seamEdges) {
    if (seamVertices.size() < 2) return; // We need at least two vertices to define a seam

    pmp::SurfaceMesh mesh_uv;

    // 0. Get the mapping of halfedge to_vertex
    // std::unordered_map<pmp::Halfedge, pmp::Vertex> halfedgeToVertexMap;

    // for (auto h : mesh.halfedges()) {
    //     halfedgeToVertexMap[h] = mesh.to_vertex(h);
    // }

    // 0.1 Get the mapping of seamHalfedge for the seamVertices
    std::vector<pmp::Halfedge> halfedgesPointingToSeam;
    for (auto h : mesh.halfedges()) {
        if (std::find(seamVertices.begin(), seamVertices.end(), mesh.to_vertex(h)) != seamVertices.end()) {
            halfedgesPointingToSeam.push_back(h);
        }
    }

    // 0.2 Initialize the mesh: Add all the vertices of the mesh to the mesh_uv
    for (auto v : mesh.vertices()) {
        mesh_uv.add_vertex(mesh.position(v));
    }

    // 1. Iterate over the faces of "mesh"
    for (auto f : mesh.faces()) {
        // 2. Get the three halfedges of the face
        auto h0 = mesh.halfedge(f);
        auto h1 = mesh.next_halfedge(h0);
        auto h2 = mesh.next_halfedge(h1);

        // 2.1 Get the vertices of the halfedges
        pmp::Vertex v0 = mesh.to_vertex(h0);
        pmp::Vertex v1 = mesh.to_vertex(h1);
        pmp::Vertex v2 = mesh.to_vertex(h2);



        // The following if conditions "cut" the mesh along the seam
        // 3. If one of the halfedges is in the halfedgesPointingToSeam, add a new vertex to the mesh_uv and overwrite the vertex
        if (std::find(halfedgesPointingToSeam.begin(), halfedgesPointingToSeam.end(), h0) != halfedgesPointingToSeam.end()) {
            v0 = mesh_uv.add_vertex(mesh.position(v0));
        }
        if (std::find(halfedgesPointingToSeam.begin(), halfedgesPointingToSeam.end(), h1) != halfedgesPointingToSeam.end()) {
            v1 = mesh_uv.add_vertex(mesh.position(v1));
        }
        if (std::find(halfedgesPointingToSeam.begin(), halfedgesPointingToSeam.end(), h2) != halfedgesPointingToSeam.end()) {
            v2 = mesh_uv.add_vertex(mesh.position(v2));
        }



        // 4. Add a new triangle based on the new vertices
        mesh_uv.add_triangle(v0, v1, v2);
    }

    // Save the remeshed model
    pmp::write(mesh_uv, "mesh_uv.off");
    mesh.clear();
    mesh = mesh_uv;

    std::vector<pmp::Vertex> seam_vertices_unique;
    if (has_boundary()) {
        for (auto v : mesh.vertices()) {
            if (mesh.is_boundary(v)) {
                seam_vertices_unique.push_back(v);
            }
        }
    }

    // ! da sind ja extrem viele mehrfach drin, weit über 2 mal
    std::sort(seam_vertices_unique.begin(), seam_vertices_unique.end());
    std::cout << "size of border vertices unique : " << seam_vertices_unique.size() << std::endl;
    // for (auto v : seam_vertices_unique) {
    //     std::cout << mesh_uv.position(v) << std::endl;
    // }
}


bool CutLineHelper::has_boundary()
{
    for (auto v : mesh.vertices())
        if (mesh.is_boundary(v))
            return true;
    return false;
}
