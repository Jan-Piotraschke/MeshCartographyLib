/**
 * @file        CachedGeodesicDistanceHelper.cpp
 * @brief       Zuständing für das caching der berechneten Daten des GeodesicDistanceHelper
 *
 * @author      Jan-Piotraschke
 * @date        2023-Sep-15
 * @license     Apache License 2.0
 *
 * @bug         -
 * @todo        -
 */

#include "CachedGeodesicDistanceHelper.h"

CachedGeodesicDistanceHelper::CachedGeodesicDistanceHelper(fs::path mesh_path)
    : mesh_path(mesh_path), geodesic_distance_helper(mesh_path)
{
}

// ========================================
// Public Functions
// ========================================

/**
 * @brief Calculate the distance using the Heat Method
 */
Eigen::MatrixXd CachedGeodesicDistanceHelper::get_mesh_distance_matrix()
{
    fs::path cache_path = mesh_path.parent_path() / (mesh_path.filename().string() + "_distance_matrix_static.csv");
    if (!boost::filesystem::exists(cache_path))
    {
        // Calculate the distance matrix of the static 3D mesh
        Eigen::MatrixXd distance_matrix = geodesic_distance_helper.get_mesh_distance_matrix();

        save_csv(distance_matrix, cache_path);
    }
    Eigen::MatrixXd distance_matrix = load_csv<Eigen::MatrixXd>(cache_path);

    return distance_matrix;
}
