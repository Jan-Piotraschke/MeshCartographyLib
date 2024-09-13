/**
 * @file        GaussianCutLineHelper.cpp
 * @brief       Create a cut line based on the Gaussian curvature
 * @details     The idea is to concenctrate all the 3D curvature to a small number of mesh vertices (so called Cones)
 *              and then cut the mesh through those vertices to obtain disk topology.
 *              Cone singularities can be used to control and minimize distortions (-> see MeshMetric/ for the disortions).
 *
 * @author      Jan-Piotraschke
 * @date        2023-Sep-22
 * @license     Apache License 2.0
 *
 * @bug         -
 * @todo        replace calculate_distances() with a function within the GeodesicDistance class
 */

#include "GaussianCutLineHelper.h"

CutLineHelper::CutLineHelper(const std::string mesh_3D_file_path, _3D::vertex_descriptor start_node)
    : mesh_3D_file_path(mesh_3D_file_path), start_node(start_node) {};

// ========================================
// Public Functions
// ========================================

/**
 * @brief Calculate the border of the mesh
 *
 * @info: Unittested
 */
std::vector<_3D::edge_descriptor> CutLineHelper::set_UV_border_edges()
{
    // Load the mesh from the file
    _3D::Mesh mesh;
    std::ifstream in(CGAL::data_file_path(mesh_3D_file_path));
    in >> mesh;

    // Create vectors to store the predecessors (p) and the distances from the root (d)
    std::vector<_3D::vertex_descriptor> predecessor_pmap(num_vertices(mesh)); // record the predecessor of each vertex
    std::vector<int> distance(num_vertices(mesh));                            // record the distance from the root

    // Calculate the distances from the start node to all other vertices
    calculate_distances(mesh, start_node, predecessor_pmap, distance);

    // Find the target node (farthest from the start node)
    _3D::vertex_descriptor target_node = find_farthest_vertex(mesh, start_node, distance);

    // Get the edges of the path between the start and the target node
    std::vector<_3D::edge_descriptor> path_list = get_cut_line(mesh, start_node, target_node, predecessor_pmap);

    return path_list;
}

// ========================================
// Private Functions
// ========================================

/**
 * @brief Create a path of vertices from the start node to the target node
 *
 * @info: Unittested
 *
 * The size of the path_list multiplied with 2 is the number of vertices on the border of the UV mesh
 *
 * So, if you want something like an inverse 'Poincaré disk' you have to really shorten the path_list
 * The same is true if you reverse the logic: If you create a spiral-like seam edge path, your mesh will results in
 * something like a 'Poincaré disk'
 */
std::vector<_3D::edge_descriptor> CutLineHelper::get_cut_line(
    const _3D::Mesh mesh,
    const _3D::vertex_descriptor start_node,
    _3D::vertex_descriptor current,
    const std::vector<_3D::vertex_descriptor> predecessor_pmap)
{
    std::vector<_3D::edge_descriptor> path_list;

    while (current != start_node)
    {
        _3D::vertex_descriptor predecessor = predecessor_pmap[current];
        std::pair<_3D::edge_descriptor, bool> edge_pair = edge(predecessor, current, mesh);
        path_list.push_back(edge_pair.first);
        current = predecessor;
    }

    std::vector<_3D::edge_descriptor> longest_mod_two;
    size_t size = path_list.size();
    size_t max_length_mod_two = size % 2 == 0 ? size : size - 1;
    size_t half_length_mod_two
        = (max_length_mod_two / 2) % 2 == 0 ? max_length_mod_two / 2 : (max_length_mod_two / 2) - 1;
    longest_mod_two = std::vector<_3D::edge_descriptor>(path_list.begin(), path_list.begin() + max_length_mod_two);

    // for(const auto& edge : longest_mod_two) {
    //     std::cout << mesh.point(source(edge, mesh)) << std::endl;
    // }

    return longest_mod_two;
}

/**
 * @brief Calculate the distances from a given start vertex to all other vertices
 *
 * @info: Unittested
 */
void CutLineHelper::calculate_distances(
    _3D::Mesh mesh,
    _3D::vertex_descriptor start_node,
    std::vector<_3D::vertex_descriptor>& predecessor_pmap,
    std::vector<int>& distance)
{
    auto indexmap = get(boost::vertex_index, mesh);
    auto dist_pmap = boost::make_iterator_property_map(distance.begin(), indexmap);

    auto vis = boost::make_bfs_visitor(std::make_pair(
        boost::record_distances(dist_pmap, boost::on_tree_edge{}),
        boost::record_predecessors(&predecessor_pmap[0], boost::on_tree_edge{})));

    boost::breadth_first_search(mesh, start_node, visitor(vis));
}

/**
 * @brief Find the farthest vertex from a given start vertex
 *
 * @info: Unittested
 */
_3D::vertex_descriptor CutLineHelper::find_farthest_vertex(
    const _3D::Mesh mesh, _3D::vertex_descriptor start_node, const std::vector<int> distance)
{
    int max_distances = 0;
    _3D::vertex_descriptor target_node;

    for (_3D::vertex_descriptor vd : vertices(mesh))
    {
        if (vd != boost::graph_traits<_3D::Mesh>::null_vertex())
        {
            if (distance[vd] > max_distances)
            {
                max_distances = distance[vd];
                target_node = vd;
            }
        }
    }

    return target_node;
}
