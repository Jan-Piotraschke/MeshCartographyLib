/**
 * @file        SurfaceParametrization.cpp
 * @brief       Create the 2D maps based on the 3D mesh
 *
 * @author      Jan-Piotraschke
 * @date        2023-Jul-19
 * @version     0.1.0
 * @license     Apache License 2.0
 *
 * @bug         -
 * @todo        improve the find_vertex_by_coordinates function, as we only have to check on the border of the mesh
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


/**
 * @brief Create the Kachelmuster
*/
void SurfaceParametrization::create_kachelmuster() {
    Tessellation tessellation(*this);  // Pass the current instance of SurfaceParametrization
    tessellation.create_kachelmuster();
    auto result = tessellation.get_sides();
    left = std::get<0>(result);
    right = std::get<1>(result);
    up = std::get<2>(result);
    down = std::get<3>(result);
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

    // Set the border edges of the UV mesh
    CutLineHelper helper = CutLineHelper(mesh_3D_file_path, start_node);
    CutLineHelperInterface& cutline_helper = helper;
    auto border_edges = cutline_helper.set_UV_border_edges();

    _3D::Mesh sm;
    std::ifstream in(CGAL::data_file_path(mesh_3D_file_path));
    in >> sm;

    // Canonical Halfedges Representing a Vertex
    _3D::UV_pmap uvmap = sm.add_property_map<_3D::halfedge_descriptor, Point_2>("h:uv").first;

    // Create the seam mesh
    UV::Mesh mesh = create_UV_mesh(sm, border_edges);

    // Choose a halfedge on the border
    UV::halfedge_descriptor bhd = CGAL::Polygon_mesh_processing::longest_border(mesh).first;

    // Perform parameterization
    SquareBorderParametrizationHelper square_helper = SquareBorderParametrizationHelper(mesh, bhd, uvmap);
    ParametrizationHelperInterface& square_border_parametrization_helper = square_helper;
    square_border_parametrization_helper.parameterize_UV_mesh();

    // Save the uv mesh
    save_UV_mesh(mesh, bhd, uvmap, mesh_3D_file_path);

    std::vector<int64_t> h_v_mapping_vector;
    int number_of_vertices = size(vertices(mesh));
    vertice_3D.resize(number_of_vertices, 3);
    vertice_UV.resize(number_of_vertices, 3);

    int i = 0;
    for (UV::vertex_descriptor vd : vertices(mesh)) {
        auto [point_3D, uv, target_vertice] = getMeshData(vd, mesh, sm, uvmap);

        h_v_mapping_vector.push_back(target_vertice);

        // Get the points
        vertice_3D(i, 0) = point_3D.x();
        vertice_3D(i, 1) = point_3D.y();
        vertice_3D(i, 2) = point_3D.z();

        // Get the uv points
        vertice_UV(i, 0) = uv.x();
        vertice_UV(i, 1) = uv.y();
        vertice_UV(i, 2) = 0;
        i++;
    }

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
 * @brief Create the UV mesh
*/
UV::Mesh SurfaceParametrization::create_UV_mesh(
    _3D::Mesh& mesh,
    const std::vector<_3D::edge_descriptor> calc_edges
){
    // Create property maps to store seam edges and vertices
    _3D::Seam_edge_pmap seam_edge_pm = mesh.add_property_map<_3D::edge_descriptor, bool>("e:on_seam", false).first;   // if not false -> we can't add seam edges
    _3D::Seam_vertex_pmap seam_vertex_pm = mesh.add_property_map<_3D::vertex_descriptor, bool>("v:on_seam", false).first;  // if not false -> we can't run the parameterization part

    UV::Mesh UV_mesh(mesh, seam_edge_pm, seam_vertex_pm);

    for (_3D::edge_descriptor e : calc_edges) {
        UV_mesh.add_seam(source(e, mesh), target(e, mesh));
    }

    return UV_mesh;
}


/**
 * @brief Save the generated UV mesh to a file
*/
void SurfaceParametrization::save_UV_mesh(
    UV::Mesh _mesh,
    UV::halfedge_descriptor _bhd,
    _3D::UV_pmap _uvmap,
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



// ========================================
// Tessellation - Public Functions
// ========================================

void SurfaceParametrization::Tessellation::create_kachelmuster() {
    analyseSides();

    std::string mesh_uv_path = parent.meshmeta.mesh_path;
    auto mesh_3D_name = parent.get_mesh_name(mesh_uv_path);

    _3D::Mesh mesh_original;
    std::ifstream in_original(CGAL::data_file_path(mesh_uv_path));
    in_original >> mesh_original;  // position 2 2

    docking_side = "left";
    process_mesh(CGAL::data_file_path(mesh_uv_path), mesh_original, 90.0, 0, 0);   // position 2 (row) 3 (column)  -> left

    docking_side = "right";
    process_mesh(CGAL::data_file_path(mesh_uv_path), mesh_original, 90.0, 2, 0);  // position 2 1  -> right

    docking_side = "up";
    process_mesh(CGAL::data_file_path(mesh_uv_path), mesh_original, 270.0, 0, 2);  // position 1 2 -> up

    docking_side = "down";
    process_mesh(CGAL::data_file_path(mesh_uv_path), mesh_original, 270.0, 0, 0);  // position 3 2 -> down

    std::string output_path = (MESH_FOLDER / (mesh_3D_name + "_kachelmuster.off")).string();
    std::ofstream out(output_path);
    out << mesh_original;
}



// ========================================
// Tessellation - Private Functions
// ========================================

void SurfaceParametrization::Tessellation::analyseSides() {
    // Check each vertex
    for(std::size_t i = 0; i < parent.polygon.size(); ++i) {
        auto vertex = parent.polygon.vertex(i);
        auto eigen_point = Eigen::Vector2d(parent.polygon[i].x(), parent.polygon[i].y()) ;

        // Check for left side
        if(CGAL::abs(vertex.x()) < 1e-9) {
            left.push_back(parent.polygon_v[i]);
            left_border.push_back(eigen_point);
        }

        // Check for right side
        if(CGAL::abs(vertex.x() - 1) < 1e-9) {
            right.push_back(parent.polygon_v[i]);
            right_border.push_back(eigen_point);
        }

        // Check for bottom side
        if(CGAL::abs(vertex.y()) < 1e-9) {
            down.push_back(parent.polygon_v[i]);
            down_border.push_back(eigen_point);
        }

        // Check for top side
        if(CGAL::abs(vertex.y() - 1) < 1e-9) {
            up.push_back(parent.polygon_v[i]);
            up_border.push_back(eigen_point);
        }
    }
}



void SurfaceParametrization::Tessellation::process_mesh(
    const std::string& mesh_path,
    _3D::Mesh& mesh_original,
    double rotation_angle,
    int shift_x,
    int shift_y
) {
    _3D::Mesh mesh;
    std::ifstream in(mesh_path);
    in >> mesh;

    rotate_and_shift_mesh(mesh, rotation_angle, shift_x, shift_y);
    add_mesh(mesh, mesh_original);
}


void SurfaceParametrization::Tessellation::rotate_and_shift_mesh(
    _3D::Mesh& mesh,
    double angle_degrees,
    int shift_x_coordinates,
    int shift_y_coordinates
) {
    double angle_radians = CGAL_PI * angle_degrees / 180.0; // Convert angle to radians
    double threshold = 1e-10; // or any other small value you consider appropriate

    // Rotate and shift the mesh
    for (auto v : mesh.vertices()){
        Point_3 pt_3d = mesh.point(v);
        Point_2 pt_2d(pt_3d.x(), pt_3d.y());
        Point_2 transformed_2d = customRotate(pt_2d, angle_radians);

        // Remove the memory errors by setting the coordinates to 0
        if (std::abs(transformed_2d.x()) < threshold) {
            transformed_2d = Point_2(0, transformed_2d.y());
        }
        if (std::abs(transformed_2d.y()) < threshold) {
            transformed_2d = Point_2(transformed_2d.x(), 0);
        }

        Point_3 transformed_3d(transformed_2d.x() + shift_x_coordinates, transformed_2d.y() + shift_y_coordinates, 0.0);
        mesh.point(v) = transformed_3d;
    }
}


Point_3 SurfaceParametrization::Tessellation::get_point_3d(
    _3D::Mesh& mesh,
    _3D::vertex_descriptor& v,
    std::vector<_3D::vertex_descriptor>& border_list
) {
    Point_3 pt_3d;
    if (std::find(border_list.begin(), border_list.end(), v) != border_list.end()) {
        auto it = std::find(border_list.begin(), border_list.end(), v);
        int index = std::distance(border_list.begin(), it);
        pt_3d = mesh.point(border_list[index]);
    } else {
        pt_3d = mesh.point(v);
    }

    return pt_3d;
}


_3D::vertex_descriptor SurfaceParametrization::Tessellation::find_vertex_by_coordinates(
    const _3D::Mesh& m,
    const Point_3& pt
) {
    for (auto v : m.vertices()) {
        if (m.point(v) == pt) {
            return v;
        }
    }
    return _3D::Mesh::null_vertex();
}


void SurfaceParametrization::Tessellation::add_mesh(
    _3D::Mesh& mesh,
    _3D::Mesh& mesh_original
) {
    // A map to relate old vertex descriptors in mesh to new ones in mesh_original
    std::map<_3D::vertex_descriptor, _3D::vertex_descriptor> reindexed_vertices;
    for (auto v : mesh.vertices()) {

        std::vector<_3D::vertex_descriptor> border_list;
        if (docking_side == "left"){
            border_list = down;
        } else if (docking_side == "right"){
            border_list = up;
        } else if (docking_side == "up"){
            border_list = right;
        } else if (docking_side == "down"){
            border_list = left;
        }
        auto pt_3d = get_point_3d(mesh, v, border_list);

        _3D::vertex_descriptor shifted_v;
        // Check if the vertex already exists in the mesh
        _3D::vertex_descriptor existing_v = find_vertex_by_coordinates(mesh_original, pt_3d);

        if (existing_v == _3D::Mesh::null_vertex()) {
            shifted_v = mesh_original.add_vertex(pt_3d);
        } else {
            shifted_v = existing_v;
        }
        // _3D::vertex_descriptor shifted_v = mesh_original.add_vertex(pt_3d);

        // // If v is inside "border_list" vector than take its shifted_v
        // if (std::find(border_list.begin(), border_list.end(), v) != border_list.end()) {
        //     target_index = -1;

        //     // Find pt_3d in polygon and get the index
        //     Point_2 target(pt_3d.x(), pt_3d.y());

        //     find_vertex_index(target);
        //     // Switch the vertices, that will form the border of the mesh
        //     shifted_v = parent.polygon_v[target_index];
        // }

        reindexed_vertices[v] = shifted_v;
    }

    // Add faces from the rotated mesh to the original mesh
    for (auto f : mesh.faces()) {
        std::vector<_3D::vertex_descriptor> face_vertices;
        for (auto v : vertices_around_face(mesh.halfedge(f), mesh)) {
            face_vertices.push_back(reindexed_vertices[v]);
        }
        mesh_original.add_face(face_vertices);
    }
}


void SurfaceParametrization::Tessellation::find_vertex_index(const Point_2& target) {
    for (size_t i = 0; i < parent.polygon.size(); ++i) {

        if (are_almost_equal(parent.polygon.vertex(i).x(), target.x()) &&
            are_almost_equal(parent.polygon.vertex(i).y(), target.y())) {
            target_index = i;
            break;
        }
    }
}


Point_2 SurfaceParametrization::Tessellation::customRotate(const Point_2& pt, double angle_radians) {
    double cos_theta = std::cos(angle_radians);
    double sin_theta = std::sin(angle_radians);

    double x_prime = pt.x() * cos_theta - pt.y() * sin_theta;
    double y_prime = pt.x() * sin_theta + pt.y() * cos_theta;

    return Point_2(x_prime, y_prime);
}


bool SurfaceParametrization::Tessellation::are_almost_equal(float a, float b) {
    return std::fabs(a - b) < EPSILON;
}
