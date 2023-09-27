/**
 * @file        SurfaceParametrization.cpp
 * @brief       Create the 2D maps based on the 3D mesh
 *
 * @author      Jan-Piotraschke
 * @date        2023-Jul-19
 * @license     Apache License 2.0
 *
 * @bug         -
 * @todo        -
 */

#include "SurfaceParametrization.h"

#include "CutLineHelper.h"
#include "HarmonicParametrizationHelper.h"

SurfaceParametrization::SurfaceParametrization(){}


// ========================================
// Public Functions
// ========================================

/**
 * @brief Extract the mesh name (without extension) from its file path
 *
 * @info: Unittested
*/
std::string SurfaceParametrization::get_mesh_name(
   const std::string mesh_3D_path
){
    fs::path path(mesh_3D_path);
    return path.stem().string();
}


/**
 * @brief Check if a given point is inside our polygon border
*/
bool SurfaceParametrization::check_point_in_polygon(
    const Point_2_eigen& point,
    bool is_original_mesh
){
    Point_2 cgal_point(point[0], point[1]);

    auto result = CGAL::bounded_side_2(polygon.vertices_begin(), polygon.vertices_end(), cgal_point, Kernel());

    return result == CGAL::ON_BOUNDED_SIDE || result == CGAL::ON_BOUNDARY;
}


/**
 * @brief Create the UV surface
*/
std::tuple<std::vector<int64_t>, Eigen::MatrixXd, Eigen::MatrixXd, std::string> SurfaceParametrization::create_uv_surface(
    std::string mesh_path,
    int32_t start_node_int
){
    _3D::vertex_descriptor start_node(start_node_int);
    mesh_3D_file_path = mesh_path;
    auto h_v_mapping_vector = calculate_uv_surface(start_node);

    std::string mesh_uv_file_path = meshmeta.mesh_path;

    extract_polygon_border_edges(mesh_uv_file_path);

    return {h_v_mapping_vector, vertice_UV, vertice_3D, mesh_uv_file_path};
}



// ========================================
// Private Functions
// ========================================

/**
 * @brief Calculate the UV coordinates of the 3D mesh and also return their mapping to the 3D coordinates
*/
std::vector<int64_t> SurfaceParametrization::calculate_uv_surface(
    _3D::vertex_descriptor start_node
){
    pmp::Vertex start_vertex(start_node.idx());

    pmp::SurfaceMesh mesh;
    pmp::read_off(mesh, mesh_3D_file_path);

    // Set the border edges of the UV mesh
    CutLineHelper helper = CutLineHelper(mesh, start_vertex);
    CutLineHelperInterface& cutline_helper = helper;
    cutline_helper.cut_mesh_open();

    // Perform the parameterization
    HarmonicParametrizationHelper helper_p = HarmonicParametrizationHelper(mesh);
    ParametrizationHelperInterface& parametrization_helper = helper_p;
    parametrization_helper.parameterize_UV_mesh();

    // Save the uv mesh
    fs::path mesh_uv_path = MESH_FOLDER / (get_mesh_name(mesh_3D_file_path) + "_uv.off");
    save_uv_as_mesh(mesh, mesh_uv_path);

    std::vector<int64_t> h_v_mapping_vector;
    // int number_of_vertices = size(vertices(mesh));
    // vertice_3D.resize(number_of_vertices, 3);
    // vertice_UV.resize(number_of_vertices, 3);

    // int i = 0;
    // for (UV::vertex_descriptor vd : vertices(mesh)) {
    //     auto [point_3D, uv, target_vertice] = getMeshData(vd, mesh, sm, uvmap);

    //     h_v_mapping_vector.push_back(target_vertice);

    //     // Get the points
    //     vertice_3D(i, 0) = point_3D.x();
    //     vertice_3D(i, 1) = point_3D.y();
    //     vertice_3D(i, 2) = point_3D.z();

    //     // Get the uv points
    //     vertice_UV(i, 0) = uv.x();
    //     vertice_UV(i, 1) = uv.y();
    //     vertice_UV(i, 2) = 0;
    //     i++;
    // }

    return h_v_mapping_vector;
}


std::tuple<Point_3, Point_2, int64_t> SurfaceParametrization::getMeshData(
    const UV::vertex_descriptor& vd,
    const UV::Mesh& mesh,
    const _3D::Mesh& sm,
    _3D::UV_pmap& _uvmap
){
    int64_t target_vertice = target(vd, sm);
    Point_3 point_3D = sm.point(target(vd, sm));
    Point_2 uv = get(_uvmap, halfedge(vd, mesh));
    return {point_3D, uv, target_vertice};
}


void SurfaceParametrization::save_uv_as_mesh(const pmp::SurfaceMesh& mesh, const fs::path& filename)
{
    pmp::SurfaceMesh uvMesh;

    // Get UV coordinates property
    auto UV_coord = mesh.get_vertex_property<pmp::TexCoord>("v:tex");
    if (!UV_coord)
        throw std::runtime_error("UV coordinates not found!");

    // Create a mapping from the old vertices to the new vertices
    std::map<pmp::Vertex, pmp::Vertex> vertexMapping;

    // Add vertices to uvMesh using UV coordinates as positions
    for (auto v : mesh.vertices())
    {
        pmp::Point uvPoint(UV_coord[v][0], UV_coord[v][1], 0.0); // We set the Z coordinate to 0
        vertexMapping[v] = uvMesh.add_vertex(uvPoint);
    }

    // Add faces to uvMesh
    for (auto f : mesh.faces())
    {
        std::vector<pmp::Vertex> uvFaceVertices;
        for (auto v : mesh.vertices(f))
        {
            uvFaceVertices.push_back(vertexMapping[v]);
        }
        uvMesh.add_face(uvFaceVertices);
    }

    // Write uvMesh to an .off file
    pmp::write(uvMesh, filename.string());
}


void SurfaceParametrization::extract_polygon_border_edges(
    const std::string& mesh_uv_path
){
    std::ifstream input(CGAL::data_file_path(mesh_uv_path));
    _3D::Mesh mesh;
    input >> mesh;

    // Find the border edges of the mesh
    std::vector<_3D::halfedge_descriptor> border_edges;
    CGAL::Polygon_mesh_processing::border_halfedges(mesh, std::back_inserter(border_edges));

    // Create a map from source vertex to border halfedge
    std::unordered_map<_3D::vertex_descriptor, _3D::halfedge_descriptor> source_to_halfedge;
    for (const _3D::halfedge_descriptor& h : border_edges) {
        source_to_halfedge[mesh.source(h)] = h;
    }

    // Extract the coordinates of the vertices in the correct order
    std::unordered_set<_3D::vertex_descriptor> visited;
    _3D::vertex_descriptor v = mesh.source(border_edges[0]);
    for (std::size_t i = 0; i < border_edges.size(); i++) {
        polygon_v.push_back(v);
        polygon.push_back(Point_2(mesh.point(v).x(), mesh.point(v).y()));

        visited.insert(v);

        _3D::halfedge_descriptor next_h = source_to_halfedge[mesh.target(source_to_halfedge[v])];
        v = mesh.source(next_h);

        // Ensure that we don't visit the same vertex again
        if (visited.count(v)) {
            break;
        }
    }
}
