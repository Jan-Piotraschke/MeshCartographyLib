#pragma once

#include <Eigen/Dense>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "SurfaceParametrization.h"

class Tessellation
{
  public:
    Tessellation(SurfaceParametrization& sp) : parent(sp) {}

    void create_kachelmuster();

    std::tuple<
        std::vector<Point_2_eigen>,
        std::vector<Point_2_eigen>,
        std::vector<Point_2_eigen>,
        std::vector<Point_2_eigen>>
    get_borders()
    {
        return {left_border, right_border, up_border, down_border};
    };
    std::vector<Point_2_eigen> left_border, right_border, up_border, down_border;

  private:
    SurfaceParametrization& parent;
    std::string docking_side;
    int target_index;

    Point_3 get_point_3d(_3D::Mesh& mesh, _3D::vertex_descriptor& v, std::vector<_3D::vertex_descriptor>& border_list);
    Point_2 customRotate(const Point_2& pt, double angle_radians);
    void process_mesh(
        const std::string& mesh_path, _3D::Mesh& mesh_original, double rotation_angle, int shift_x, int shift_y);
    void find_vertex_index(const Point_2& target);
    void rotate_and_shift_mesh(_3D::Mesh& mesh, double angle_degrees, int shift_x_coordinates, int shift_y_coordinates);
    void add_mesh(_3D::Mesh& mesh, _3D::Mesh& mesh_original);
    bool are_almost_equal(float a, float b);
    _3D::vertex_descriptor find_vertex_by_coordinates(const _3D::Mesh& m, const Point_3& pt);
    void analyseSides();
    std::vector<_3D::vertex_descriptor> left, right, up, down;

    static constexpr double EPSILON = 1e-6;

    friend class SurfaceParametrization;
};
