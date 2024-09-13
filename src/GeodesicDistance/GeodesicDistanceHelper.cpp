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

GeodesicDistanceHelper::GeodesicDistanceHelper(fs::path mesh_path) : mesh_path(mesh_path)
{
}

// ========================================
// Public Functions
// ========================================

/**
 * @brief Calculate the distance using the Heat Method
 */
Eigen::MatrixXd GeodesicDistanceHelper::get_mesh_distance_matrix()
{
    Triangle_mesh mesh;
    std::ifstream filename(CGAL::data_file_path(mesh_path.string()));
    filename >> mesh;

    Eigen::MatrixXd distance_matrix_v(num_vertices(mesh), num_vertices(mesh));

    // loop over all vertices and fill the distance matrix
    for (auto vi = vertices(mesh).first; vi != vertices(mesh).second; ++vi)
    {
        fill_distance_matrix(mesh, distance_matrix_v, *vi);
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
    Triangle_mesh mesh, Eigen::MatrixXd& distance_matrix, int closest_vertice)
{
    if (distance_matrix.row(closest_vertice).head(2).isZero())
    {
        // get the distance of all vertices to all other vertices
        std::vector<double> vertices_3D_distance_map = geo_distance(mesh, closest_vertice);
        distance_matrix.row(closest_vertice)
            = Eigen::Map<Eigen::VectorXd>(vertices_3D_distance_map.data(), vertices_3D_distance_map.size());
    }
}

std::vector<double> GeodesicDistanceHelper::geo_distance(Triangle_mesh mesh, int32_t start_node)
{
    // property map for the distance values to the source set
    Vertex_distance_map vertex_distance = mesh.add_property_map<vertex_descriptor, double>("v:distance", 0).first;

    // pass in the idt object and its vertex_distance_map
    Heat_method hm_idt(mesh);

    // add the first vertex as the source set
    vertex_descriptor source = *(vertices(mesh).first + start_node);
    hm_idt.add_source(source);
    hm_idt.estimate_geodesic_distances(vertex_distance);

    std::vector<double> distances_list;
    for (vertex_descriptor vd : vertices(mesh))
    {
        distances_list.push_back(get(vertex_distance, vd));
    }

    return distances_list;
}
