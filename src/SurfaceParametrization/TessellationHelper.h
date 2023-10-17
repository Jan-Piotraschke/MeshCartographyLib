#pragma once

#include <vector>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <Eigen/Dense>

#include "SurfaceParametrization.h"

class Tessellation {
public:
    Tessellation(SurfaceParametrization& sp) : parent(sp) {}

    std::vector<std::vector<int64_t>> create_kachelmuster();

    std::tuple<
        std::vector<Point_2_eigen>,
        std::vector<Point_2_eigen>,
        std::vector<Point_2_eigen>,
        std::vector<Point_2_eigen>
    > get_borders(){
        return {left_border, right_border, up_border, down_border};
    };
    std::vector<Point_2_eigen> left_border, right_border, up_border, down_border;

private:
    SurfaceParametrization& parent;
    std::string docking_side;
    int target_index;
    std::vector<std::vector<int64_t>> equivalent_vertices;

    Point_3_eigen get_point_3d(
        pmp::SurfaceMesh& mesh,
        pmp::Vertex& v,
        std::vector<pmp::Vertex>& border_list
    );
    Point_2_eigen customRotate(const Point_2_eigen& pt, double angle_radians);
    void process_mesh(const std::string& mesh_path, pmp::SurfaceMesh& mesh_original, double rotation_angle, int shift_x, int shift_y);
    void find_vertex_index(const Point_2_eigen& target);
    void rotate_and_shift_mesh(pmp::SurfaceMesh& mesh, double angle_degrees, int shift_x_coordinates, int shift_y_coordinates);
    void add_mesh(pmp::SurfaceMesh& mesh, pmp::SurfaceMesh& mesh_original);
    bool are_almost_equal(float a, float b);
    pmp::Vertex find_vertex_by_coordinates(
        const pmp::SurfaceMesh& m,
        const Point_3_eigen& pt
    );
    void analyseSides();
    std::vector<pmp::Vertex> left, right, up, down;

    static constexpr double EPSILON = 1e-6;

friend class SurfaceParametrization;
};
