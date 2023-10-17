/**
 * @file        TessellationDistanceHelper.cpp
 * @brief       Calculate the distance between mesh vertices
 *
 * @author      Jan-Piotraschke
 * @date        2023-Oct-17
 * @license     Apache License 2.0
 *
 * @bug         -
 * @todo        -
 */

#include "TessellationDistanceHelper.h"

TessellationDistanceHelper::TessellationDistanceHelper(fs::path mesh_path, std::vector<std::vector<int64_t>> equivalent_vertices
) : mesh_path(mesh_path),
    equivalent_vertices(equivalent_vertices)
{}


// ========================================
// Public Functions
// ========================================

/**
 * @brief Calculate the distance using the Heat Method
*/
Eigen::MatrixXd TessellationDistanceHelper::get_mesh_distance_matrix() {
    pmp::SurfaceMesh mesh;
    pmp::read_off(mesh, mesh_path.string());

    size_t numVerts = mesh.n_vertices();
    Eigen::MatrixXd distance_matrix_v(numVerts, numVerts);

    // loop over all vertices and fill the distance matrix
    for (auto vi : mesh.vertices()) {
        fill_distance_matrix(mesh, distance_matrix_v, vi);
    }

    distance_matrix_v = filter_matrix(distance_matrix_v);

    return distance_matrix_v;
}



// ========================================
// Private Functions
// ========================================

/**
 * @brief Variable to keep track of the current index of the vector of distances, and each thread processes a
 * different index until all the distances have been added to the distance matrix.
*/
void TessellationDistanceHelper::fill_distance_matrix(
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


std::vector<double> TessellationDistanceHelper::calculate_geodesic_distance(
    pmp::SurfaceMesh& mesh,
    pmp::Vertex start_vertex
){
    std::vector<pmp::Vertex> seeds{start_vertex};
    pmp::geodesics_heat(mesh, seeds);

    pmp::VertexProperty<pmp::Scalar> distance_pmap = mesh.get_vertex_property<pmp::Scalar>("geodesic:distance");

    std::vector<double> distances;
    for (pmp::Vertex vertex : mesh.vertices()) {
        distances.push_back(distance_pmap[vertex]);
    }

    return distances;
}


Eigen::MatrixXd TessellationDistanceHelper::filter_matrix(Eigen::MatrixXd& distance_matrix) {
    const size_t numVerts = equivalent_vertices.size();
    const size_t numVerts_kachelmuster = distance_matrix.rows();
    Eigen::MatrixXd distance_matrix_v(numVerts, numVerts);

    for (int vertice_id = 0; vertice_id < numVerts; ++vertice_id) {
        auto equivalent_vertices_selected = equivalent_vertices[vertice_id];
        equivalent_vertices_selected.push_back(vertice_id);

        Eigen::MatrixXd filtered_matrix = distance_matrix.block(0, 0, numVerts_kachelmuster, numVerts);
        Eigen::VectorXi keep_cols = Eigen::VectorXi::LinSpaced(filtered_matrix.cols(), 0, filtered_matrix.cols());
        filtered_matrix = filtered_matrix(equivalent_vertices_selected, keep_cols);

        Eigen::VectorXd minValues(numVerts);

        for (int i = 0; i < filtered_matrix.cols(); ++i) {
            distance_matrix_v(vertice_id, i) = filtered_matrix.col(i).minCoeff();
        }
    }

    return distance_matrix_v;
}
