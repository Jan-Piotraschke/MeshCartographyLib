/**
 * @file        TessellationHelper.cpp
 * @brief       Create the tesselation of the UV mesh monotile
 *
 * @author      Jan-Piotraschke
 * @date        2023-Sep-22
 * @license     Apache License 2.0
 *
 * @bug         -
 * @todo        -
 */

#include <math.h>
#include "TessellationHelper.h"

// ========================================
// Public Functions
// ========================================

std::vector<std::vector<int64_t>> Tessellation::create_kachelmuster() {
    std::string mesh_uv_path = parent.meshmeta.mesh_path;
    auto mesh_uv_name = parent.get_mesh_name(mesh_uv_path);

    pmp::SurfaceMesh mesh_original;
    pmp::read_off(mesh_original, mesh_uv_path);
    std::cout << "Mesh name: " << mesh_uv_name << std::endl;
    equivalent_vertices.resize(mesh_original.n_vertices());

    size_t size = parent.border_map.size();
    for (size_t i = 0; i < size; ++i) {
        docking_side = i;
        size_t nextIndex = (i + 1) % size;
        double rotation_angle = calculateAngle(parent.border_map[nextIndex], parent.border_map[i]);

        pmp::SurfaceMesh mesh;
        pmp::read_off(mesh, mesh_uv_path);

        rotate_and_shift_mesh(mesh, rotation_angle);
        add_mesh(mesh, mesh_original);
    }

    std::cout << "Finished creating the kachelmuster" << std::endl;
    std::string output_path = (MESH_FOLDER / (mesh_uv_name + "_kachelmuster.off")).string();
    pmp::write(mesh_original, output_path);

    return equivalent_vertices;
}


void Tessellation::rotate_and_shift_mesh(
    pmp::SurfaceMesh& mesh,
    double angle_degrees
) {
    double angle_radians = M_PI * angle_degrees / 180.0; // Convert angle to radians
    double threshold = 1e-10;

    // Get the border of the mesh
    std::vector<Point_2_eigen> main_border, connection_side;
    main_border = parent.border_map[docking_side];
    connection_side = parent.border_map[parent.twin_border_map[docking_side]];

    order_data(main_border);

    // collect all points of the twinborder as a Eigen::matrixXd object
    Eigen::MatrixXd main_border_matrix(main_border.size(), 2);
    for (size_t i = 0; i < main_border.size(); ++i) {
        main_border_matrix(i, 0) = main_border[i](0);
        main_border_matrix(i, 1) = main_border[i](1);
    }

    // pre-rotation: rotate only the border_vertices of the main and compare them to its twin border to get the shift
    std::vector<Eigen::Vector2d> vec;
    for (auto pt_2d : connection_side) {
        Point_2_eigen transformed_2d = customRotate(pt_2d, angle_radians);

        // Remove the memory errors by setting the coordinates to 0
        if (std::abs(transformed_2d.x()) < threshold) {
            transformed_2d = Point_2_eigen(0, transformed_2d.y());
        }
        if (std::abs(transformed_2d.y()) < threshold) {
            transformed_2d = Point_2_eigen(transformed_2d.x(), 0);
        }
        vec.push_back(transformed_2d);

    }

    order_data(vec);

    // transform vec into a Eigen::MatrixXd connection_matrix(connection_side.size(), 2)
    Eigen::MatrixXd connection_matrix(connection_side.size(), 2);
    for (size_t i = 0; i < vec.size(); ++i) {
        connection_matrix(i, 0) = vec[i](0);
        connection_matrix(i, 1) = vec[i](1);
    }

    float shift_x_coordinates, shift_y_coordinates;
    shift_x_coordinates = main_border_matrix(0, 0) - connection_matrix(0, 0);
    shift_y_coordinates = main_border_matrix(0, 1) - connection_matrix(0, 1);

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



// ========================================
// Tessellation - Private Functions
// ========================================

double Tessellation::calculateAngle(const std::vector<Eigen::Vector2d>& border1, const std::vector<Eigen::Vector2d>& border2) {
    Eigen::Vector2d dir1 = fitLine(border1);
    Eigen::Vector2d dir2 = fitLine(border2);

    dir1.normalize();
    dir2.normalize();

    double dot = dir1.dot(dir2);
    double det = dir1.x() * dir2.y() - dir1.y() * dir2.x();

    double angle = std::atan2(det, dot);
    double angleInDegrees = angle * (180.0 / M_PI);

    // Normalize to [0, 360)
    if (angleInDegrees < 0) {
        angleInDegrees += 360;
    }

    return angleInDegrees;
}


Eigen::Vector2d Tessellation::fitLine(const std::vector<Eigen::Vector2d>& points) {
    // Calculate the covariance matrix of the points
    Eigen::Vector2d mean = Eigen::Vector2d::Zero();
    for (const auto& p : points) {
        mean += p;
    }
    mean /= points.size();

    Eigen::Matrix2d cov = Eigen::Matrix2d::Zero();
    for (const auto& p : points) {
        Eigen::Vector2d centered = p - mean;
        cov += centered * centered.transpose();
    }
    cov /= points.size();

    // Find the eigenvector of the covariance matrix corresponding to the largest eigenvalue
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> solver(cov);
    Eigen::Vector2d dir = solver.eigenvectors().col(1);

    return dir;
}


void Tessellation::order_data(std::vector<Eigen::Vector2d>& vec) {
    Eigen::VectorXd X(vec.size());
    Eigen::VectorXd Y(vec.size());
    for (size_t i = 0; i < vec.size(); ++i) {
        X[i] = vec[i](0);
        Y[i] = vec[i](1);
    }

    Eigen::VectorXd A = Eigen::VectorXd::Ones(vec.size());
    Eigen::MatrixXd B(vec.size(), 2);
    B << X, A;
    Eigen::Vector2d coeffs = B.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(Y);
    double m = coeffs(0), b = coeffs(1);

    // Check if all x-values are the same (vertical line)
    bool verticalLine = (X.maxCoeff() - X.minCoeff()) < std::numeric_limits<double>::epsilon();

    // Sort the vector based on the parameter t
    std::sort(vec.begin(), vec.end(),
        [m, b, verticalLine](const Eigen::Vector2d& a, const Eigen::Vector2d& c) {
            if (verticalLine) {
                return a[1] < c[1];
            } else {
                double ta = (a[0] + m * (a[1] - b)) / std::sqrt(1 + m * m);
                double tc = (c[0] + m * (c[1] - b)) / std::sqrt(1 + m * m);
                return ta < tc;
            }
        });
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


// Das kann man gut zum Befüllen der Equivalenzliste nutzen
void Tessellation::add_mesh(
    pmp::SurfaceMesh& mesh,
    pmp::SurfaceMesh& mesh_original
) {
    // A map to relate old vertex descriptors in mesh to new ones in mesh_original
    std::map<pmp::Vertex, pmp::Vertex> reindexed_vertices;
    for (auto v : mesh.vertices()) {

        std::vector<int64_t>& kachelmuster_twin_v = equivalent_vertices[v.idx()];

        std::vector<pmp::Vertex> border_list;
        border_list = parent.border_v_map[parent.twin_border_map[docking_side]];

        auto pt_3d = get_point_3d(mesh, v, border_list);

        pmp::Vertex shifted_v;
        // Check if the vertex already exists in the mesh
        pmp::Vertex existing_v = find_vertex_by_coordinates(mesh_original, pt_3d);

        if (existing_v == pmp::Vertex(-1)) {
            shifted_v = mesh_original.add_vertex(pt_3d);
            kachelmuster_twin_v.push_back(shifted_v.idx());
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


Point_2_eigen Tessellation::customRotate(const Point_2_eigen& pt, double angle_radians) {
    double cos_theta = std::cos(angle_radians);
    double sin_theta = std::sin(angle_radians);

    double x_prime = pt.x() * cos_theta - pt.y() * sin_theta;
    double y_prime = pt.x() * sin_theta + pt.y() * cos_theta;

    return Point_2_eigen(x_prime, y_prime);
}
