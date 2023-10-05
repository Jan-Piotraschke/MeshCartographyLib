/**
 * @file        TessellationHelper.cpp
 * @brief       Create the tesselation of the UV mesh monotile
 *
 * @author      Jan-Piotraschke
 * @date        2023-Sep-22
 * @license     Apache License 2.0
 *
 * @bug         - The mesh is sometimes not added correctly to the original mesh
 * @todo        - improve the find_vertex_by_coordinates function, as we only have to check on the border of the mesh
 */

#include <math.h>
#include "TessellationHelper.h"

// ========================================
// Public Functions
// ========================================

void Tessellation::create_kachelmuster() {
    analyseSides();

    std::string mesh_uv_path = parent.meshmeta.mesh_path;
    auto mesh_uv_name = parent.get_mesh_name(mesh_uv_path);

    pmp::SurfaceMesh mesh_original;
    pmp::read_off(mesh_original, mesh_uv_path);
    std::cout << "Mesh name: " << mesh_uv_name << std::endl;

    docking_side = "left";
    process_mesh(mesh_uv_path, mesh_original, 90.0, 0, 0);   // position 2 (row) 3 (column)  -> left

    docking_side = "right";
    process_mesh(mesh_uv_path, mesh_original, 90.0, 2, 0);  // position 2 1  -> right

    docking_side = "up";
    process_mesh(mesh_uv_path, mesh_original, 270.0, 0, 2);  // position 1 2 -> up

    docking_side = "down";
    process_mesh(mesh_uv_path, mesh_original, 270.0, 0, 0);  // position 3 2 -> down
    std::cout << "Finished creating the kachelmuster" << std::endl;
    std::string output_path = (MESH_FOLDER / (mesh_uv_name + "_kachelmuster.off")).string();
    pmp::write(mesh_original, output_path);
}



// ========================================
// Tessellation - Private Functions
// ========================================

void Tessellation::analyseSides() {
    // Check each vertex
    for(std::size_t i = 0; i < parent.polygon.size(); ++i) {
        Point_2_eigen eigen_point = parent.polygon[i];

        // Check for left side
        if(std::abs(eigen_point(0)) < 1e-9) {
            left.push_back(parent.polygon_v[i]);
            left_border.push_back(eigen_point);
        }

        // Check for right side
        if(std::abs(eigen_point(0) - 1) < 1e-9) {
            right.push_back(parent.polygon_v[i]);
            right_border.push_back(eigen_point);
        }

        // Check for bottom side
        if(std::abs(eigen_point(1)) < 1e-9) {
            down.push_back(parent.polygon_v[i]);
            down_border.push_back(eigen_point);
        }

        // Check for top side
        if(std::abs(eigen_point(1) - 1) < 1e-9) {
            up.push_back(parent.polygon_v[i]);
            up_border.push_back(eigen_point);
        }
    }
}


void Tessellation::process_mesh(
    const std::string& mesh_path,
    pmp::SurfaceMesh& mesh_original,
    double rotation_angle,
    int shift_x,
    int shift_y
) {
    pmp::SurfaceMesh mesh;
    pmp::read_off(mesh, mesh_path);

    rotate_and_shift_mesh(mesh, rotation_angle, shift_x, shift_y);
    add_mesh(mesh, mesh_original);
}


void Tessellation::rotate_and_shift_mesh(
    pmp::SurfaceMesh& mesh,
    double angle_degrees,
    int shift_x_coordinates,
    int shift_y_coordinates
) {
    double angle_radians = M_PI * angle_degrees / 180.0; // Convert angle to radians
    double threshold = 1e-10; // or any other small value you consider appropriate

    // Rotate and shift the mesh
    for (auto v : mesh.vertices()){
        Point_3_eigen pt_3d = mesh.position(v);
        Point_2_eigen pt_2d(pt_3d.x(), pt_3d.y());
        Point_2_eigen transformed_2d = customRotate(pt_2d, angle_radians);

        // Remove the memory errors by setting the coordinates to 0
        if (std::abs(transformed_2d.x()) < threshold) {
            transformed_2d = Point_2_eigen(0, transformed_2d.y());
        }
        if (std::abs(transformed_2d.y()) < threshold) {
            transformed_2d = Point_2_eigen(transformed_2d.x(), 0);
        }

        Point_3_eigen transformed_3d(transformed_2d.x() + shift_x_coordinates, transformed_2d.y() + shift_y_coordinates, 0.0);
        mesh.position(v) = transformed_3d;
    }
}


Point_3_eigen Tessellation::get_point_3d(
    pmp::SurfaceMesh& mesh,
    pmp::Vertex& v,
    std::vector<pmp::Vertex>& border_list
) {
    Point_3_eigen pt_3d;
    if (std::find(border_list.begin(), border_list.end(), v) != border_list.end()) {
        auto it = std::find(border_list.begin(), border_list.end(), v);
        int index = std::distance(border_list.begin(), it);
        pt_3d = mesh.position(border_list[index]);
    } else {
        pt_3d = mesh.position(v);
    }

    return pt_3d;
}


pmp::Vertex Tessellation::find_vertex_by_coordinates(
    const pmp::SurfaceMesh& m,
    const Point_3_eigen& pt
) {
    for (auto v : m.vertices()) {
        if (m.position(v) == pt) {
            return v;
        }
    }
    // If no vertex was found, return an empty vertex
    pmp::Vertex empty_vertex(-1);
    return empty_vertex;
}


void Tessellation::add_mesh(
    pmp::SurfaceMesh& mesh,
    pmp::SurfaceMesh& mesh_original
) {
    // A map to relate old vertex descriptors in mesh to new ones in mesh_original
    std::map<pmp::Vertex, pmp::Vertex> reindexed_vertices;
    for (auto v : mesh.vertices()) {

        std::vector<pmp::Vertex> border_list;
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

        pmp::Vertex shifted_v;
        // Check if the vertex already exists in the mesh
        pmp::Vertex existing_v = find_vertex_by_coordinates(mesh_original, pt_3d);

        if (existing_v == pmp::Vertex(-1)) {
            shifted_v = mesh_original.add_vertex(pt_3d);
        } else {
            shifted_v = existing_v;
        }

        reindexed_vertices[v] = shifted_v;
    }

    // Add faces from the rotated mesh to the original mesh
    for (auto f : mesh.faces()) {
        std::vector<pmp::Vertex> face_vertices;

        std::vector<pmp::Vertex> vertices_around_face;
        auto h0 = mesh.halfedge(f);
        auto h1 = mesh.next_halfedge(h0);
        auto h2 = mesh.prev_halfedge(h0);

        // 2.1 Get the vertices of the halfedges
        pmp::Vertex v0 = mesh.to_vertex(h0);
        pmp::Vertex v1 = mesh.to_vertex(h1);
        pmp::Vertex v2 = mesh.to_vertex(h2);

        vertices_around_face.push_back(v0);
        vertices_around_face.push_back(v1);
        vertices_around_face.push_back(v2);

        for (auto v : vertices_around_face) {
            face_vertices.push_back(reindexed_vertices[v]);
        }

        mesh_original.add_face(face_vertices);
    }
}


void Tessellation::find_vertex_index(const Point_2_eigen& target) {
    for (size_t i = 0; i < parent.polygon.size(); ++i) {

        if (are_almost_equal(parent.polygon[i](0), target(0)) &&
            are_almost_equal(parent.polygon[i](1), target(1))) {
            target_index = i;
            break;
        }
    }
}


bool Tessellation::are_almost_equal(float a, float b) {
    return std::fabs(a - b) < EPSILON;
}


Point_2_eigen Tessellation::customRotate(const Point_2_eigen& pt, double angle_radians) {
    double cos_theta = std::cos(angle_radians);
    double sin_theta = std::sin(angle_radians);

    double x_prime = pt.x() * cos_theta - pt.y() * sin_theta;
    double y_prime = pt.x() * sin_theta + pt.y() * cos_theta;

    return Point_2_eigen(x_prime, y_prime);
}
