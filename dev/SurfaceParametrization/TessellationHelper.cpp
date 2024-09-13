/**
 * @file        TessellationHelper.cpp
 * @brief       Create the tesselation of the UV mesh monotile
 *
 * @author      Jan-Piotraschke
 * @date        2023-Sep-22
 * @license     Apache License 2.0
 *
 * @bug         -
 * @todo        improve the find_vertex_by_coordinates function, as we only have to check on the border of the mesh
 */

#include "TessellationHelper.h"

// ========================================
// Public Functions
// ========================================

void Tessellation::create_kachelmuster()
{
    analyseSides();

    std::string mesh_uv_path = parent.meshmeta.mesh_path;
    auto mesh_3D_name = parent.get_mesh_name(mesh_uv_path);

    _3D::Mesh mesh_original;
    std::ifstream in_original(CGAL::data_file_path(mesh_uv_path));
    in_original >> mesh_original; // position 2 2

    docking_side = "left";
    process_mesh(CGAL::data_file_path(mesh_uv_path), mesh_original, 90.0, 0, 0); // position 2 (row) 3 (column)  -> left

    docking_side = "right";
    process_mesh(CGAL::data_file_path(mesh_uv_path), mesh_original, 90.0, 2, 0); // position 2 1  -> right

    docking_side = "up";
    process_mesh(CGAL::data_file_path(mesh_uv_path), mesh_original, 270.0, 0, 2); // position 1 2 -> up

    docking_side = "down";
    process_mesh(CGAL::data_file_path(mesh_uv_path), mesh_original, 270.0, 0, 0); // position 3 2 -> down

    std::string output_path = (MESH_FOLDER / (mesh_3D_name + "_kachelmuster.off")).string();
    std::ofstream out(output_path);
    out << mesh_original;
}

// ========================================
// Tessellation - Private Functions
// ========================================

void Tessellation::analyseSides()
{
    // Check each vertex
    for (std::size_t i = 0; i < parent.polygon.size(); ++i)
    {
        auto vertex = parent.polygon.vertex(i);
        auto eigen_point = Eigen::Vector2d(parent.polygon[i].x(), parent.polygon[i].y());

        // Check for left side
        if (CGAL::abs(vertex.x()) < 1e-9)
        {
            left.push_back(parent.polygon_v[i]);
            left_border.push_back(eigen_point);
        }

        // Check for right side
        if (CGAL::abs(vertex.x() - 1) < 1e-9)
        {
            right.push_back(parent.polygon_v[i]);
            right_border.push_back(eigen_point);
        }

        // Check for bottom side
        if (CGAL::abs(vertex.y()) < 1e-9)
        {
            down.push_back(parent.polygon_v[i]);
            down_border.push_back(eigen_point);
        }

        // Check for top side
        if (CGAL::abs(vertex.y() - 1) < 1e-9)
        {
            up.push_back(parent.polygon_v[i]);
            up_border.push_back(eigen_point);
        }
    }
}

void Tessellation::process_mesh(
    const std::string& mesh_path, _3D::Mesh& mesh_original, double rotation_angle, int shift_x, int shift_y)
{
    _3D::Mesh mesh;
    std::ifstream in(mesh_path);
    in >> mesh;

    rotate_and_shift_mesh(mesh, rotation_angle, shift_x, shift_y);
    add_mesh(mesh, mesh_original);
}

void Tessellation::rotate_and_shift_mesh(
    _3D::Mesh& mesh, double angle_degrees, int shift_x_coordinates, int shift_y_coordinates)
{
    double angle_radians = CGAL_PI * angle_degrees / 180.0; // Convert angle to radians
    double threshold = 1e-10;                               // or any other small value you consider appropriate

    // Rotate and shift the mesh
    for (auto v : mesh.vertices())
    {
        Point_3 pt_3d = mesh.point(v);
        Point_2 pt_2d(pt_3d.x(), pt_3d.y());
        Point_2 transformed_2d = customRotate(pt_2d, angle_radians);

        // Remove the memory errors by setting the coordinates to 0
        if (std::abs(transformed_2d.x()) < threshold)
        {
            transformed_2d = Point_2(0, transformed_2d.y());
        }
        if (std::abs(transformed_2d.y()) < threshold)
        {
            transformed_2d = Point_2(transformed_2d.x(), 0);
        }

        Point_3 transformed_3d(transformed_2d.x() + shift_x_coordinates, transformed_2d.y() + shift_y_coordinates, 0.0);
        mesh.point(v) = transformed_3d;
    }
}

Point_3 Tessellation::get_point_3d(
    _3D::Mesh& mesh, _3D::vertex_descriptor& v, std::vector<_3D::vertex_descriptor>& border_list)
{
    Point_3 pt_3d;
    if (std::find(border_list.begin(), border_list.end(), v) != border_list.end())
    {
        auto it = std::find(border_list.begin(), border_list.end(), v);
        int index = std::distance(border_list.begin(), it);
        pt_3d = mesh.point(border_list[index]);
    }
    else
    {
        pt_3d = mesh.point(v);
    }

    return pt_3d;
}

_3D::vertex_descriptor Tessellation::find_vertex_by_coordinates(const _3D::Mesh& m, const Point_3& pt)
{
    for (auto v : m.vertices())
    {
        if (m.point(v) == pt)
        {
            return v;
        }
    }
    return _3D::Mesh::null_vertex();
}

void Tessellation::add_mesh(_3D::Mesh& mesh, _3D::Mesh& mesh_original)
{
    // A map to relate old vertex descriptors in mesh to new ones in mesh_original
    std::map<_3D::vertex_descriptor, _3D::vertex_descriptor> reindexed_vertices;
    for (auto v : mesh.vertices())
    {

        std::vector<_3D::vertex_descriptor> border_list;
        if (docking_side == "left")
        {
            border_list = down;
        }
        else if (docking_side == "right")
        {
            border_list = up;
        }
        else if (docking_side == "up")
        {
            border_list = right;
        }
        else if (docking_side == "down")
        {
            border_list = left;
        }
        auto pt_3d = get_point_3d(mesh, v, border_list);

        _3D::vertex_descriptor shifted_v;
        // Check if the vertex already exists in the mesh
        _3D::vertex_descriptor existing_v = find_vertex_by_coordinates(mesh_original, pt_3d);

        if (existing_v == _3D::Mesh::null_vertex())
        {
            shifted_v = mesh_original.add_vertex(pt_3d);
        }
        else
        {
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
    for (auto f : mesh.faces())
    {
        std::vector<_3D::vertex_descriptor> face_vertices;
        for (auto v : vertices_around_face(mesh.halfedge(f), mesh))
        {
            face_vertices.push_back(reindexed_vertices[v]);
        }
        mesh_original.add_face(face_vertices);
    }
}

void Tessellation::find_vertex_index(const Point_2& target)
{
    for (size_t i = 0; i < parent.polygon.size(); ++i)
    {

        if (are_almost_equal(parent.polygon.vertex(i).x(), target.x())
            && are_almost_equal(parent.polygon.vertex(i).y(), target.y()))
        {
            target_index = i;
            break;
        }
    }
}

Point_2 Tessellation::customRotate(const Point_2& pt, double angle_radians)
{
    double cos_theta = std::cos(angle_radians);
    double sin_theta = std::sin(angle_radians);

    double x_prime = pt.x() * cos_theta - pt.y() * sin_theta;
    double y_prime = pt.x() * sin_theta + pt.y() * cos_theta;

    return Point_2(x_prime, y_prime);
}

bool Tessellation::are_almost_equal(float a, float b)
{
    return std::fabs(a - b) < EPSILON;
}
