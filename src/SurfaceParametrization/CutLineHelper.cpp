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

void CutLineHelper::open_mesh_along_seam(const std::vector<pmp::Vertex>& seamVertices, const std::vector<pmp::Edge>& seamEdges) {
    if (seamVertices.size() < 2) return; // We need at least two vertices to define a seam

    // Initialize v_prev as the first vertex in the seamVertices list
    pmp::Vertex v_prev = seamVertices[0];
    pmp::Vertex v_mapped;

    Eigen::Matrix<pmp::Vertex, Eigen::Dynamic, 3> not_mapped_side(seamVertices.size() - 1, 3);

    for (int i = 0; i < seamVertices.size() - 1; ++i) {
        auto v0 = seamVertices[i];
        auto v1 = seamVertices[i + 1];
        auto h0 = mesh.find_halfedge(v0, v1);
        auto h = mesh.opposite_halfedge(h0);

        // get the face that contains the halfedge h
        pmp::Face f = mesh.face(h);

        int j = 0;
        for (auto h_face : mesh.halfedges(f)) {
            // Avoid overflow
            if (j >= 3) {
                std::cerr << "Warning: More than 3 vertices found in face. Skipping remaining vertices." << std::endl;
                break;
            }

            not_mapped_side(i, j) = mesh.to_vertex(h_face);
            ++j;
        }
    }


    std::cout << "Not mapped side:" << std::endl;
    for (int i = 0; i < not_mapped_side.rows(); ++i) {
        for (int j = 0; j < not_mapped_side.cols(); ++j) {
            std::cout << not_mapped_side(i, j) << " ";
        }
        std::cout << std::endl;
    }

    // Iterate over seamVertices starting from the second vertex
    std::vector<pmp::Vertex> mapped_side;
    mapped_side.push_back(seamVertices[0]);

    for (size_t i = 1; i < seamVertices.size(); ++i) {
        pmp::Vertex v_curr = seamVertices[i];
        pmp::Halfedge h = mesh.find_halfedge(v_prev, v_curr);

        // Get the face associated with this halfedge
        pmp::Face f = mesh.face(h);

        // Add a new vertex with the same position as v_curr
        pmp::Point pos = mesh.position(v_curr);
        pmp::Vertex v_new = mesh.add_vertex(pos);
        mapped_side.push_back(v_new);

        // Replace occurrences of v_curr with v_new and v_prev with v_mapped in the face f
        for (auto h_face : mesh.halfedges(f)) {
            if (mesh.to_vertex(h_face) == v_curr) {
                mesh.set_vertex(h_face, v_new);
            }
            // If v_mapped is valid and current vertex in halfedge is v_prev, replace with v_mapped
            if (v_mapped.is_valid() && mesh.to_vertex(h_face) == v_prev) {
                mesh.set_vertex(h_face, v_mapped);
            }
        }

        v_prev = v_curr;
        v_mapped = v_new;
    }

    std::cout << "Mapped side:" << std::endl;
    for (auto v : mapped_side) {
        std::cout << v << " ";
    }
    // ! wir nähern uns langsam der Lösung
    if (has_boundary()) {
        std::cout << "Mesh has boundary!" << std::endl;
        // print the boundary vertices
        for (auto v : mesh.vertices()) {
            if (mesh.is_boundary(v)) {
                // std::cout << "Boundary vertex: " << v << std::endl;
            }
        }
    }
}


// void CutLineHelper::open_mesh_along_seam(const std::vector<pmp::Vertex>& seamVertices, const std::vector<pmp::Edge>& seamEdges) {
//     if (seamVertices.size() < 2) return; // We need at least two vertices to define a seam

//     // get the halfedge that goes from the first item of the seamVertices to the second
//     pmp::Vertex v_prev = seamVertices[0];
//     pmp::Vertex v_mapped;

//     for (size_t i = 1; i < seamVertices.size(); ++i) {
//         pmp::Vertex v1 = seamVertices[i];
//         pmp::Halfedge h = mesh.find_halfedge(v_prev, v1);
//         std::cout << "Halfedge: " << h << std::endl;

//         // get the face that contains the halfedge h
//         pmp::Face f = mesh.face(h);

//         // add the new vertex to the mesh
//         pmp::Point pos = mesh.position(v1);
//         pmp::Vertex v_new = mesh.add_vertex(pos);

//         // replace v1 with v_new in the face f
//         for (auto h : mesh.halfedges(f)) {
//             // Check if this halfedge points to v1
//             if (mesh.to_vertex(h) == v1) {
//                 mesh.set_vertex(h, v_new);
//             }
//         }
//         v_prev = v1;
//         v_mapped = v_new;
//     }
// }


bool CutLineHelper::has_boundary()
{
    for (auto v : mesh.vertices())
        if (mesh.is_boundary(v))
            return true;
    return false;
}
