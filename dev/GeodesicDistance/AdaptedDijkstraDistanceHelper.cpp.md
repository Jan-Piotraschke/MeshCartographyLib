---
output_filename: "AdaptedDijkstraDistanceHelper"

brief: "Zuständing für die Berechnung basierend auf den Dijkstra-Algorithmus"
details: "Dijkstra's algorithm calculates shortest paths in a graph G = (V, E) with non-negative edge weights, yielding real numbers for path lengths.
            The optimal run-time here is O(|E| + |V|log|V|)."
---

# Adapted Dijkstra Distance Helper

The purpose of this module is to provide functions for calculating the distance between mesh vertices using an adapted Dijkstra algorithm.

This implementation adapts Dijkstra's algorithm for 3D surface meshes. Instead of simple edge traversal, it considers the geometric distances on the mesh. The shortest_paths object uses an internal priority queue to explore the shortest paths efficiently, respecting the surface's topology. This ensures that distances reflect the true geodesic path over the curved surface rather than merely counting edges.

```cpp
#include "AdaptedDijkstraDistanceHelper.h"

AdaptedDijkstraDistanceHelper::AdaptedDijkstraDistanceHelper(fs::path mesh_path) : mesh_path(mesh_path)
{
}

// ========================================
// Public Functions
// ========================================

/**
 * @brief Calculate the distance using the Heat Method
 */
Eigen::MatrixXd AdaptedDijkstraDistanceHelper::get_mesh_distance_matrix()
{
    pmp::SurfaceMesh mesh;
    pmp::read_off(mesh, mesh_path.string());

    size_t numVerts = mesh.n_vertices();
    Eigen::MatrixXd distance_matrix_v(numVerts, numVerts);

    // loop over all vertices and fill the distance matrix
    for (auto vi : mesh.vertices())
    {
        fill_distance_matrix(mesh, distance_matrix_v, vi);
    }

    return distance_matrix_v;
}

/**
 * @brief Variable to keep track of the current index of the vector of distances, and each thread processes a
 * different index until all the distances have been added to the distance matrix.
 */
void AdaptedDijkstraDistanceHelper::fill_distance_matrix(
    pmp::SurfaceMesh& mesh, Eigen::MatrixXd& distance_matrix, pmp::Vertex vertex)
{
    if (distance_matrix.row(vertex.idx()).head(2).isZero())
    {
        // get the distance of all vertices to all other vertices
        std::vector<double> vertices_3D_distance_map = calculate_edge_count_distance(mesh, vertex);
        distance_matrix.row(vertex.idx())
            = Eigen::Map<Eigen::VectorXd>(vertices_3D_distance_map.data(), vertices_3D_distance_map.size());
    }
}
```

## Dijkstra Algorithm

Get the distance of all vertices to all other vertices using an adapted Dijkstra algorithm.

```cpp
std::vector<double> AdaptedDijkstraDistanceHelper::calculate_geodesic_distance(pmp::SurfaceMesh& mesh, pmp::Vertex start_vertex)
{
    std::vector<pmp::Vertex> seeds{start_vertex};
    pmp::geodesics(mesh, seeds);

    pmp::VertexProperty<pmp::Scalar> distance_pmap = mesh.get_vertex_property<pmp::Scalar>("geodesic:distance");

    std::vector<double> distances;
    for (pmp::Vertex vertex : mesh.vertices())
    {
        distances.push_back(distance_pmap[vertex]);
    }

    return distances;
}

std::vector<double> AdaptedDijkstraDistanceHelper::calculate_edge_count_distance(
    pmp::SurfaceMesh& mesh, pmp::Vertex start_vertex)
{
    std::vector<double> distances(mesh.n_vertices(), std::numeric_limits<double>::max());
    distances[start_vertex.idx()] = 0;

    std::set<std::pair<double, pmp::Vertex>> vertexQueue;
    vertexQueue.insert(std::make_pair(0, start_vertex));

    while (!vertexQueue.empty())
    {
        double currentDistance = vertexQueue.begin()->first;
        pmp::Vertex currentVertex = vertexQueue.begin()->second;
        vertexQueue.erase(vertexQueue.begin());

        for (auto halfedge : mesh.halfedges(currentVertex))
        {
            pmp::Vertex neighborVertex = mesh.to_vertex(halfedge);

            double tentativeDistance = currentDistance + 1; // As each edge counts as 1
            if (tentativeDistance < distances[neighborVertex.idx()])
            {
                vertexQueue.erase(std::make_pair(distances[neighborVertex.idx()], neighborVertex));

                distances[neighborVertex.idx()] = tentativeDistance;
                vertexQueue.insert(std::make_pair(tentativeDistance, neighborVertex));
            }
        }
    }

    return distances;
}
```
