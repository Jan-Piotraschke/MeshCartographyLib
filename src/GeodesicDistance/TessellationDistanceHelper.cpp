/**
 * @file        TessellationDistanceHelper.cpp
 * @brief       Calculate the distance between mesh vertices
 *
 * @author      Jan-Piotraschke
 * @date        2023-Oct-17
 * @license     Apache License 2.0
 *
 * @bug         -
 * @todo        - OpenMP parallelization
*/

#include "TessellationDistanceHelper.h"
#include "HeatDistanceHelper.h"

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
    const size_t original_vertices_num = equivalent_vertices.size();
    Eigen::MatrixXd distance_matrix_v(original_vertices_num, numVerts);
    HeatDistanceHelper geodesic_distance_helper(mesh_path);

    // loop over the original vertices and fill the distance matrix
    for (int i = 0; i < original_vertices_num; ++i) {
        if (i % 100 == 0) std::cout << "i: " << i << std::endl;
        geodesic_distance_helper.fill_distance_matrix(mesh, distance_matrix_v, pmp::Vertex(i));
    }

    distance_matrix_v = filter_matrix(distance_matrix_v);

    return distance_matrix_v;
}



// ========================================
// Private Functions
// ========================================

Eigen::MatrixXd TessellationDistanceHelper::filter_matrix(Eigen::MatrixXd& distance_matrix) {
    const size_t numVerts = equivalent_vertices.size();
    const size_t numVerts_kachelmuster = distance_matrix.cols();
    Eigen::MatrixXd distance_matrix_v(numVerts, numVerts);
    Eigen::VectorXi keep_rows = Eigen::VectorXi::LinSpaced(numVerts, 0, numVerts);

    for (int vertice_id = 0; vertice_id < numVerts; ++vertice_id) {
        auto equivalent_vertices_selected = equivalent_vertices[vertice_id];
        equivalent_vertices_selected.push_back(vertice_id);

        Eigen::MatrixXd filtered_matrix = distance_matrix(keep_rows, equivalent_vertices_selected);

        for (int i = 0; i < filtered_matrix.rows(); ++i) {
            auto result = filtered_matrix.row(i).minCoeff();
            distance_matrix_v(i, vertice_id) = result;
        }
    }

    make_symmetric(distance_matrix_v);

    return distance_matrix_v;
}


void TessellationDistanceHelper::make_symmetric(Eigen::MatrixXd& distance_matrix) {
    const size_t numVerts = distance_matrix.rows();
    for (size_t i = 0; i < numVerts; ++i) {
        for (size_t j = i + 1; j < numVerts; ++j) {
            if (distance_matrix(i, j) > distance_matrix(j, i)) {
                distance_matrix(i, j) = distance_matrix(j, i);
            } else {
                distance_matrix(j, i) = distance_matrix(i, j);
            }
        }
    }
}
