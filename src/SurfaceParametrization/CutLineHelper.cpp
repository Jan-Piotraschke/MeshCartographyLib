/**
 * @file        CutLineHelper.cpp
 * @brief       Calculate the cut line of the mesh
 *
 * @author      Jan-Piotraschke
 * @date        2023-Sep-22
 * @license     Apache License 2.0
 *
 * @bug         ich bekomme nicht alle Dreiecks-punkte bei mesh_uv, die ich brauche
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

    // To check if the edges are valid:
    for (const auto& edge : edge_path) {
        if (!mesh.is_valid(edge)) {
            std::cout << "Invalid edge found" << std::endl;
        }
    }

    open_mesh_along_seam(edge_path);
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

#include <algorithm>

std::vector<pmp::Vertex> CutLineHelper::get_neighbors(pmp::Vertex v) const {
    std::vector<pmp::Vertex> neighbors;
    auto vv = mesh.vertices(v);
    for (auto it = vv.begin(); it != vv.end(); ++it) {
        neighbors.push_back(*it);
    }
    return neighbors;
}


void CutLineHelper::open_mesh_along_seam(const std::vector<pmp::Edge>& seamEdges) {
    pmp::SurfaceMesh mesh_uv;

    std::vector<pmp::Vertex> edge_direction;
    edge_direction.push_back(mesh.vertex(seamEdges[0], 0));
    edge_direction.push_back(mesh.vertex(seamEdges[0], 1));
    edge_direction.push_back(mesh.vertex(seamEdges[1], 0));
    edge_direction.push_back(mesh.vertex(seamEdges[1], 1));

    pmp::Vertex heading_towards;
    bool found = false;

    for (size_t i = 0; i < edge_direction.size() && !found; ++i) {
        for (size_t j = i + 1; j < edge_direction.size(); ++j) {
            if (edge_direction[i] == edge_direction[j]) {
                heading_towards = edge_direction[i];
                found = true;
                break;
            }
        }
    }

    pmp::Vertex heading_from;
    auto option_a = mesh.vertex(seamEdges[0], 0);
    auto option_b = mesh.vertex(seamEdges[0], 1);
    if (option_a == heading_towards) {
        heading_from = option_b;
    } else {
        heading_from = option_a;
    }
    std::cout << "heading_from : " << heading_from << std::endl;

    // 0.1 Get the mapping of seamHalfedge for the seamEdges
    std::vector<pmp::Halfedge> halfedgesPointingToSeam;

    for (size_t i = 0; i < seamEdges.size(); ++i) {
        // Get the vertices of the seamEdge
        pmp::Vertex v0 = mesh.vertex(seamEdges[i], 0);
        pmp::Vertex v1 = mesh.vertex(seamEdges[i], 1);
        pmp::Halfedge h;

        if (v0 == heading_from) {
            h = mesh.find_halfedge(v0, v1);
            if (!mesh.is_valid(h)) {
                std::cerr << "Invalid halfedge for vertices: " << v0 << ", " << v1 << std::endl;
                continue;
            }
            halfedgesPointingToSeam.push_back(h);
            // std::cout << v0 << " -> " << v1 << std::endl;
            heading_from = v1;
        } else if (v1 == heading_from) {
            h = mesh.find_halfedge(v1, v0);
            if (!mesh.is_valid(h)) {
                std::cerr << "Invalid halfedge for vertices: " << v1 << ", " << v0 << std::endl;
                continue;
            }
            // std::cout << v1 << " -> " << v0 << std::endl;
            halfedgesPointingToSeam.push_back(h);
            heading_from = v0;
        } else {
            std::cerr << "Neither vertex matches heading_from!" << std::endl;
            continue;
        }
    }

    std::set<pmp::Vertex> seam_vertices;

    // Collect the special vertices
    for (auto h : halfedgesPointingToSeam) {
        seam_vertices.insert(mesh.to_vertex(h));
    }

    std::vector<int> neighbors_count;

    for (auto v : mesh.vertices()) {
        if (seam_vertices.find(v) == seam_vertices.end()) {
            int count = 0;

            auto neighbors = get_neighbors(v);
            for (auto n : neighbors) {
                if (seam_vertices.find(n) == seam_vertices.end()) {
                    count++;
                }
            }

            neighbors_count.push_back(count);
        }
    }

    for (auto n : neighbors_count) {
        std::cout << n << std::endl;
    }


    // 0.2 Initialize the mesh: Add all the vertices of the mesh to the mesh_uv
    for (auto v : mesh.vertices()) {
        mesh_uv.add_vertex(mesh.position(v));
    }

    std::vector<pmp::Vertex> old_vertex;
    std::vector<pmp::Vertex> new_vertex;

    // 0.3 Add the vertices of the seam to the mesh_uv
    for (size_t i = 0; i < halfedgesPointingToSeam.size() - 1 ; ++i) {
        auto v = mesh.to_vertex(halfedgesPointingToSeam[i]);
        old_vertex.push_back(v);
        v = mesh_uv.add_vertex(mesh.position(v));
        new_vertex.push_back(v);
    }

    // 1. Iterate over the faces of "mesh"
    for (auto f : mesh.faces()) {
        // 2. Get the three halfedges of the face
        auto h0 = mesh.halfedge(f);
        auto h1 = mesh.next_halfedge(h0);
        auto h2 = mesh.prev_halfedge(h0);

        // 2.1 Get the vertices of the halfedges
        pmp::Vertex v0 = mesh.to_vertex(h0);
        pmp::Vertex v1 = mesh.to_vertex(h1);
        pmp::Vertex v2 = mesh.to_vertex(h2);

        // find if h0, h1 or h2 is in halfedgesPointingToSeam
        bool h0Exists = std::find(halfedgesPointingToSeam.begin(), halfedgesPointingToSeam.end(), h0) != halfedgesPointingToSeam.end();
        bool h1Exists = std::find(halfedgesPointingToSeam.begin(), halfedgesPointingToSeam.end(), h1) != halfedgesPointingToSeam.end();
        bool h2Exists = std::find(halfedgesPointingToSeam.begin(), halfedgesPointingToSeam.end(), h2) != halfedgesPointingToSeam.end();

        // if any of the halfedges is in halfedgesPointingToSeam
        if (h0Exists || h1Exists || h2Exists) {
            // find if v1 inside old_vertex
            auto it = std::find(old_vertex.begin(), old_vertex.end(), v0);
            if (it != old_vertex.end()) {
                size_t index = std::distance(old_vertex.begin(), it);
                v0 = new_vertex[index];
            }

            it = std::find(old_vertex.begin(), old_vertex.end(), v1);
            if (it != old_vertex.end()) {
                size_t index = std::distance(old_vertex.begin(), it);
                v1 = new_vertex[index];
            }

            it = std::find(old_vertex.begin(), old_vertex.end(), v2);
            if (it != old_vertex.end()) {
                size_t index = std::distance(old_vertex.begin(), it);
                v2 = new_vertex[index];
            }
        }

        // 4. Add a new triangle based on the new vertices
        auto new_face = mesh_uv.add_triangle(v0, v1, v2);

        // 5. Check if the new face is valid
        if (mesh_uv.is_isolated(v0) || mesh_uv.is_isolated(v1) || mesh_uv.is_isolated(v2)) {
            std::cout << "isolated vertex found" << std::endl;
        }
        if (!mesh_uv.is_valid(new_face)) {
            std::cout << "invalid face found" << std::endl;
        }
        if (!mesh_uv.is_valid(mesh_uv.find_edge(v0, v1))) {
            std::cout << "invalid edge found" << std::endl;
        }
        if (!mesh_uv.is_valid(mesh_uv.find_edge(v1, v2))) {
            std::cout << "invalid edge found" << std::endl;
        }
        if (!mesh_uv.is_valid(mesh_uv.find_edge(v2, v0))) {
            std::cout << "invalid edge found" << std::endl;
        }
    }
    if (!mesh_uv.is_triangle_mesh()) {
        std::cout << "mesh_uv is not a triangle mesh" << std::endl;
    }

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

    std::sort(seam_vertices_unique.begin(), seam_vertices_unique.end());
    std::cout << "size of border vertices unique : " << seam_vertices_unique.size() << std::endl;

    // for (auto v : seam_vertices_unique) {
    //     // std::cout << v << std::endl;
    //     // std::cout << mesh.position(v) << std::endl;
    // }
}


bool CutLineHelper::has_boundary()
{
    for (auto v : mesh.vertices())
        if (mesh.is_boundary(v))
            return true;
    return false;
}
