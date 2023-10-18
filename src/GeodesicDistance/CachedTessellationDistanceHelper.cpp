/**
 * @file        CachedTessellationDistanceHelper.cpp
 * @brief       Zuständing für das caching der berechneten Daten des TessellationDistanceHelper
 *
 * @author      Jan-Piotraschke
 * @date        2023-Oct-17
 * @license     Apache License 2.0
 *
 * @bug         -
 * @todo        -
 */

#include "CachedTessellationDistanceHelper.h"

CachedTessellationDistanceHelper::CachedTessellationDistanceHelper(fs::path mesh_path, std::vector<std::vector<int64_t>> equivalent_vertices
) :
    mesh_path(mesh_path),
    equivalent_vertices(equivalent_vertices),
    geodesic_distance_helper(mesh_path, equivalent_vertices)
{}


// ========================================
// Public Functions
// ========================================

/**
 * @brief Calculate the distance using the Heat Method
*/
Eigen::MatrixXd CachedTessellationDistanceHelper::get_mesh_distance_matrix() {
    fs::path cache_path = mesh_path.parent_path() / "data" / (mesh_path.stem().string() + "_distance_matrix_static.csv");
    if (!boost::filesystem::exists(cache_path)) {
        // Calculate the distance matrix of the static 3D mesh
        Eigen::MatrixXd distance_matrix = geodesic_distance_helper.get_mesh_distance_matrix();

        save_csv(distance_matrix, cache_path);
    }
    Eigen::MatrixXd distance_matrix = load_csv<Eigen::MatrixXd>(cache_path);

    return distance_matrix;
}
