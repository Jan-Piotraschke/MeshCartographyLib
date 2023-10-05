/**
 * @file        GeodesicDistanceHelper.cpp
 * @brief       Zuständing für die Berechnung
 *
 * @author      Jan-Piotraschke
 * @date        2023-Sep-15
 * @license     Apache License 2.0
 *
 * @bug         -
 * @todo        - check, ob man das Mesh nicht auch als Referenz übergeben kann
 *              - check, ob man nicht hier Eigen:: durch std::vector ersetzen kann
 */

#include "GeodesicDistanceHelper.h"

GeodesicDistanceHelper::GeodesicDistanceHelper(fs::path mesh_path) : mesh_path(mesh_path) {}


// ========================================
// Public Functions
// ========================================

/**
 * @brief Calculate the distance using the Heat Method
*/
Eigen::MatrixXd GeodesicDistanceHelper::get_mesh_distance_matrix() {
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



// ========================================
// Private Functions
// ========================================

/**
 * @brief Variable to keep track of the current index of the vector of distances, and each thread processes a
 * different index until all the distances have been added to the distance matrix.
*/
void GeodesicDistanceHelper::fill_distance_matrix(
    pmp::SurfaceMesh mesh,
    Eigen::MatrixXd& distance_matrix,
    pmp::Vertex closest_vertice
){

    if (distance_matrix.row(closest_vertice.idx()).head(2).isZero()) {
        // get the distance of all vertices to all other vertices
        std::vector<double> vertices_3D_distance_map = geo_distance(mesh, closest_vertice);
        distance_matrix.row(closest_vertice.idx()) = Eigen::Map<Eigen::VectorXd>(vertices_3D_distance_map.data(), vertices_3D_distance_map.size());
    }
}


std::vector<double> GeodesicDistanceHelper::geo_distance(
    pmp::SurfaceMesh mesh,
    pmp::Vertex start_node
){
    std::vector<pmp::Vertex> seeds{start_node};
    pmp::geodesics_heat(mesh, seeds);

    //property map for the distance values to the source set
    pmp::VertexProperty<pmp::Scalar> distance_pmap = mesh.get_vertex_property<pmp::Scalar>("geodesic:distance");

    std::vector<double> distances_list;
    for (pmp::Vertex vd : mesh.vertices()) {
        distances_list.push_back(distance_pmap[vd]);
    }

    return distances_list;
}
