#include "AdaptedDijkstraDistanceHelper.h"

// Define the necessary CGAL types for the shortest path calculation
typedef CGAL::Surface_mesh_shortest_path_traits<Kernel, _3D::Mesh> Traits;
typedef CGAL::Surface_mesh_shortest_path<Traits> Surface_mesh_shortest_path;

// Constructor
AdaptedDijkstraDistanceHelper::AdaptedDijkstraDistanceHelper(fs::path mesh_path) : mesh_path(mesh_path)
{
}

// Function to get the distance matrix
Eigen::MatrixXd AdaptedDijkstraDistanceHelper::get_mesh_distance_matrix()
{
    _3D::Mesh mesh;
    std::ifstream filename(mesh_path.string());
    if (!filename || !(filename >> mesh))
    {
        throw std::runtime_error("Error reading mesh file.");
    }

    size_t numVerts = num_vertices(mesh);
    Eigen::MatrixXd distance_matrix_v(numVerts, numVerts);

    // Loop over all vertices to fill the distance matrix
    for (auto vi = vertices(mesh).first; vi != vertices(mesh).second; ++vi)
    {
        fill_distance_matrix(mesh, distance_matrix_v, *vi);
    }

    return distance_matrix_v;
}

// Function to fill the distance matrix for a specific vertex
void AdaptedDijkstraDistanceHelper::fill_distance_matrix(
    _3D::Mesh& mesh, Eigen::MatrixXd& distance_matrix, _3D::vertex_descriptor vertex)
{
    if (distance_matrix.row(vertex).head(2).isZero())
    {
        std::vector<double> vertices_3D_distance_map = calculate_geodesic_distance(mesh, vertex);
        distance_matrix.row(vertex)
            = Eigen::Map<Eigen::VectorXd>(vertices_3D_distance_map.data(), vertices_3D_distance_map.size());
    }
}

// Private method to calculate distances using `CGAL::Surface_mesh_shortest_path`
std::vector<double> AdaptedDijkstraDistanceHelper::calculate_geodesic_distance(
    _3D::Mesh& mesh, _3D::vertex_descriptor start_vertex)
{
    // Initialize the shortest path query object
    Surface_mesh_shortest_path shortest_paths(mesh);

    // Set the source point as the current vertex
    shortest_paths.add_source_point(start_vertex);

    // Initialize distances vector
    size_t numVerts = num_vertices(mesh);
    std::vector<double> distances(numVerts, std::numeric_limits<double>::max());

    // Calculate shortest path distances from the source to all other vertices
    for (auto vi = vertices(mesh).first; vi != vertices(mesh).second; ++vi)
    {
        // Compute the distance to the source point
        auto result = shortest_paths.shortest_distance_to_source_points(*vi);

        if (result.first != std::numeric_limits<double>::infinity())
        {
            distances[*vi] = result.first;
        }
    }

    return distances;
}
