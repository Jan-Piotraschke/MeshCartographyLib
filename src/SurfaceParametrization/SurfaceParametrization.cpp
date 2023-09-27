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
#include "SquareBorderParametrizationHelper.h"

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

bool SurfaceParametrization::has_boundary(const pmp::SurfaceMesh& mesh)
{
    for (auto v : mesh.vertices())
        if (mesh.is_boundary(v))
            return true;
    return false;
}

void SurfaceParametrization::setup_boundary_constraints(pmp::SurfaceMesh& mesh)
{
    // get properties
    auto points = mesh.vertex_property<pmp::Point>("v:point");
    auto tex = mesh.vertex_property<pmp::TexCoord>("v:tex");

    pmp::SurfaceMesh::VertexIterator vit, vend = mesh.vertices_end();
    pmp::Vertex vh;
    pmp::Halfedge hh;
    std::vector<pmp::Vertex> loop;

    // Initialize all texture coordinates to the origin.
    for (auto v : mesh.vertices())
        tex[v] = pmp::TexCoord(0.5, 0.5);

    // find 1st boundary vertex
    for (vit = mesh.vertices_begin(); vit != vend; ++vit)
        if (mesh.is_boundary(*vit))
            break;

    // collect boundary loop
    vh = *vit;
    hh = mesh.halfedge(vh);
    do
    {
        loop.push_back(mesh.to_vertex(hh));
        hh = mesh.next_halfedge(hh);
    } while (hh != mesh.halfedge(vh));

    // map boundary loop to unit circle in texture domain
    unsigned int i, n = loop.size();
    pmp::Scalar angle, l, length;
    pmp::TexCoord t;

    // compute length of boundary loop
    for (i = 0, length = 0.0; i < n; ++i)
        length += distance(points[loop[i]], points[loop[(i + 1) % n]]);

    // map length intervalls to unit circle intervals
    for (i = 0, l = 0.0; i < n;)
    {
        // go from 2pi to 0 to preserve orientation
        angle = 2.0 * M_PI * (1.0 - l / length);

        t[0] = 0.5 + 0.5 * cosf(angle);
        t[1] = 0.5 + 0.5 * sinf(angle);

        tex[loop[i]] = t;

        ++i;
        if (i < n)
        {
            l += distance(points[loop[i]], points[loop[(i + 1) % n]]);
        }
    }
}

void SurfaceParametrization::harmonic_parameterization(pmp::SurfaceMesh& mesh, bool use_uniform_weights)
{
    // check precondition
    if (!has_boundary(mesh))
    {
        auto what = std::string{__func__} + ": Mesh has no boundary.";
        throw pmp::InvalidInputException(what);
    }

    // map boundary to circle
    setup_boundary_constraints(mesh);

    // get properties
    auto tex = mesh.vertex_property<pmp::TexCoord>("v:tex");

    // build system matrix (clamp negative cotan weights to zero)
    pmp::SparseMatrix L;
    if (use_uniform_weights)
        pmp::uniform_laplace_matrix(mesh, L);
    else
        pmp::laplace_matrix(mesh, L, true);

    // build right-hand side B and inject boundary constraints
    pmp::DenseMatrix B(mesh.n_vertices(), 2);
    B.setZero();
    for (auto v : mesh.vertices())
        if (mesh.is_boundary(v))
            B.row(v.idx()) = static_cast<Eigen::Vector2d>(tex[v]);

    // solve system
    auto is_constrained = [&](unsigned int i) {
        return mesh.is_boundary(pmp::Vertex(i));
    };
    pmp::DenseMatrix X = pmp::cholesky_solve(L, B, is_constrained, B);

    // copy solution
    for (auto v : mesh.vertices())
        if (!mesh.is_boundary(v))
            tex[v] = X.row(v.idx());
}


/**
 * @brief Calculate the UV coordinates of the 3D mesh and also return their mapping to the 3D coordinates
*/
std::vector<int64_t> SurfaceParametrization::calculate_uv_surface(
    _3D::vertex_descriptor start_node
){
    pmp::Vertex start_vertex(start_node.idx());

    pmp::SurfaceMesh mesh_pmp;
    pmp::read_off(mesh_pmp, mesh_3D_file_path);

    // Set the border edges of the UV mesh
    CutLineHelper helper = CutLineHelper(mesh_pmp, start_vertex);
    CutLineHelperInterface& cutline_helper = helper;
    cutline_helper.cut_mesh_open();

    std::cout << "Cutting done." << std::endl;
    // Perform the parameterization
    harmonic_parameterization(mesh_pmp);
    std::cout << "Parameterization done." << std::endl;

    // print the vertex coordinates of the mesh
    // for (auto v : mesh_pmp.vertices()) {
    //     std::cout << "v[" << v << "] = " << mesh_pmp.position(v) << std::endl;
    // }
    pmp::write(mesh_pmp, "test.off");

    // Triangle_mesh mesh;
    // std::ifstream in(CGAL::data_file_path(mesh_3D_file_path));
    // in >> mesh;

    // // Choose a halfedge on the border
    // halfedge_descriptor bhd = CGAL::Polygon_mesh_processing::longest_border(mesh).first;

    // // // Canonical Halfedges Representing a Vertex
    // // ! _3D::UV_pmap uvmap = mesh.add_property_map<_3D::halfedge_descriptor, Point_2>("h:uv").first;
    // // The 2D points of the uv parametrisation will be written into this map
    // UV_pmap uvmap = mesh.add_property_map<vertex_descriptor, Point_2>("v:uv").first;

    // // Perform parameterization
    // SquareBorderParametrizationHelper square_helper = SquareBorderParametrizationHelper(mesh, bhd, uvmap);
    // ParametrizationHelperInterface& square_border_parametrization_helper = square_helper;
    // square_border_parametrization_helper.parameterize_UV_mesh();

    // // Save the uv mesh
    // save_UV_mesh(mesh, bhd, uvmap, mesh_3D_file_path);

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


/**
 * @brief Save the generated UV mesh to a file
*/
void SurfaceParametrization::save_UV_mesh(
    Triangle_mesh _mesh,
    halfedge_descriptor _bhd,
    UV_pmap _uvmap,
    const std::string mesh_path
){
    // Get the mesh name without the extension
    auto mesh_3D_name = get_mesh_name(mesh_path);

    // Create the output file path based on uv_mesh_number
    fs::path output_file_path;
    std::string output_file_path_str;

    output_file_path = MESH_FOLDER / (mesh_3D_name + "_uv.off");
    output_file_path_str = output_file_path.string();
    meshmeta.mesh_path = output_file_path_str;

    std::ofstream out(output_file_path_str);
    SMP::IO::output_uvmap_to_off(_mesh, _bhd, _uvmap, out);
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
