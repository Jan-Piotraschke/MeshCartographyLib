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

    // Initialize v_start as the first vertex in the seamVertices list
    pmp::Vertex v_start = seamVertices[0];
    pmp::Vertex v_stop = seamVertices[seamVertices.size() - 1];
    pmp::Vertex v_seam_prev;
    std::vector<pmp::Vertex> seam_vertices;

    for (size_t i = 1; i < seamVertices.size(); ++i) {
        pmp::Vertex v_next = seamVertices[i];
        pmp::Halfedge h_seam = mesh.find_halfedge(v_start, v_next);

        // Get the face associated with this halfedge
        pmp::Face f_seam = mesh.face(h_seam);

        // Add a new vertex with the same position as v_next
        pmp::Point pos = mesh.position(v_next);
        pmp::Vertex v_seam = mesh.add_vertex(pos);

        for (auto h_face : mesh.halfedges(f_seam)) {
            // If current vertex in halfedge is v_next in the cut-line, replace with v_seam
            if (mesh.to_vertex(h_face) == v_next && mesh.to_vertex(h_face) != v_stop) {
                mesh.set_vertex(h_face, v_seam);
            }
            if (v_seam_prev.is_valid() && mesh.to_vertex(h_face) == v_start) {
                mesh.set_vertex(h_face, v_seam_prev);
            }
        }
        auto h_opposite = mesh.opposite_halfedge(h_seam);

        seam_vertices.push_back(v_seam);
        seam_vertices.push_back(mesh.to_vertex(h_opposite));

        v_start = v_next;
        v_seam_prev = v_seam;
    }
    std::cout << "\n\n" << std::endl;

    std::sort(seam_vertices.begin(), seam_vertices.end());

    for (auto v : seam_vertices) {
        std::cout << v << std::endl;
    }

    pmp::SurfaceMesh mesh_uv;
    for (auto v : mesh.vertices()) {
        mesh_uv.add_vertex(mesh.position(v));
    }

    for (auto f : mesh.faces()) {
        std::vector<pmp::Vertex> face_vertices;
        for (auto v : mesh.vertices(f)) {
            face_vertices.push_back(v);
        }
        mesh_uv.add_face(face_vertices);
    }

    // pmp::write(mesh_uv, "mesh_uv.off");

    mesh = mesh_uv;
    std::cout << "Mesh vertices: " << mesh.n_vertices() << std::endl;
    std::cout << std::endl;

    std::vector<pmp::Vertex> seam_vertices_unique;

    if (has_boundary()) {
        int boundary_vertices = 0;
        for (auto v : mesh.vertices()) {
            if (mesh.is_boundary(v)) {
                seam_vertices_unique.push_back(v);
                boundary_vertices++;
            }
        }
        std::cout << "Boundary vertices: " << boundary_vertices << std::endl;
    }
    for (auto v : seam_vertices_unique) {
        std::cout << v << std::endl;
    }
}


// void CutLineHelper::open_mesh_along_seam(const std::vector<pmp::Vertex>& seamVertices, const std::vector<pmp::Edge>& seamEdges) {
//     if (seamVertices.size() < 2) return; // We need at least two vertices to define a seam

//     // get the halfedge that goes from the first item of the seamVertices to the second
//     pmp::Vertex v_start = seamVertices[0];
//     pmp::Vertex v_seam_prev;

//     for (size_t i = 1; i < seamVertices.size(); ++i) {
//         pmp::Vertex v1 = seamVertices[i];
//         pmp::Halfedge h = mesh.find_halfedge(v_start, v1);
//         std::cout << "Halfedge: " << h << std::endl;

//         // get the face that contains the halfedge h
//         pmp::Face f = mesh.face(h);

//         // add the new vertex to the mesh
//         pmp::Point pos = mesh.position(v1);
//         pmp::Vertex v_seam = mesh.add_vertex(pos);

//         // replace v1 with v_seam in the face f
//         for (auto h : mesh.halfedges(f)) {
//             // Check if this halfedge points to v1
//             if (mesh.to_vertex(h) == v1) {
//                 mesh.set_vertex(h, v_seam);
//             }
//         }
//         v_start = v1;
//         v_seam_prev = v_seam;
//     }
// }


bool CutLineHelper::has_boundary()
{
    for (auto v : mesh.vertices())
        if (mesh.is_boundary(v))
            return true;
    return false;
}
