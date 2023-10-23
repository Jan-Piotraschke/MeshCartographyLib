/**
 * @file        HeatDistanceHelper.cpp
 * @brief       Zuständing für die Berechnung
 *
 * @author      Jan-Piotraschke
 * @date        2023-Sep-15
 * @license     Apache License 2.0
 *
 * @bug         -
 * @todo        - check, ob man nicht hier Eigen:: durch std::vector ersetzen kann
 */

#include "HeatDistanceHelper.h"

HeatDistanceHelper::HeatDistanceHelper(fs::path mesh_path) : mesh_path(mesh_path) {}


// ========================================
// Public Functions
// ========================================

/**
 * @brief Calculate the distance using the Heat Method
*/
Eigen::MatrixXd HeatDistanceHelper::get_mesh_distance_matrix() {
    pmp::SurfaceMesh mesh;
    pmp::read_off(mesh, mesh_path.string());

    size_t numVerts = mesh.n_vertices();
    Eigen::MatrixXd distance_matrix_v(numVerts, numVerts);

    // loop over all vertices and fill the distance matrix
    for (auto vi : mesh.vertices()) {
        fill_distance_matrix(mesh, distance_matrix_v, vi);
    }

    return distance_matrix_v;
}

/**
 * @brief Variable to keep track of the current index of the vector of distances, and each thread processes a
 * different index until all the distances have been added to the distance matrix.
*/
void HeatDistanceHelper::fill_distance_matrix(
    pmp::SurfaceMesh& mesh,
    Eigen::MatrixXd& distance_matrix,
    pmp::Vertex vertex
){
    if (distance_matrix.row(vertex.idx()).head(2).isZero()) {
        // get the distance of all vertices to all other vertices
        std::vector<double> vertices_3D_distance_map = calculate_geodesic_distance(mesh, vertex);
        distance_matrix.row(vertex.idx()) = Eigen::Map<Eigen::VectorXd>(vertices_3D_distance_map.data(), vertices_3D_distance_map.size());
    }
}


// ========================================
// Private Functions
// ========================================

std::vector<double> HeatDistanceHelper::calculate_geodesic_distance(
    pmp::SurfaceMesh& mesh,
    pmp::Vertex start_vertex
){
    std::vector<pmp::Vertex> seeds{start_vertex};
    pmp::geodesics_heat(mesh, seeds);

    pmp::VertexProperty<pmp::Scalar> distance_pmap = mesh.get_vertex_property<pmp::Scalar>("geodesic:distance");

    std::vector<double> distances;
    distances.reserve(mesh.n_vertices());

    for (pmp::Vertex vertex : mesh.vertices()) {
        distances.push_back(distance_pmap[vertex]);
    }

    return distances;
}
