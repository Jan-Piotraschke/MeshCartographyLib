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

    std::vector<std::vector<int64_t>> create_kachelmuster();

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
    void rotate_and_shift_mesh(pmp::SurfaceMesh& mesh, double angle_degrees);

  private:
    SurfaceParametrization& parent;
    int docking_side;
    int target_index;
    std::vector<std::vector<int64_t>> equivalent_vertices;

    Point_3_eigen get_point_3d(pmp::SurfaceMesh& mesh, pmp::Vertex& v, std::vector<pmp::Vertex>& border_list);

    std::map<int, std::vector<Eigen::Vector2d>> border_map;
    std::map<int, int> twin_border_map;
    std::map<int, std::vector<pmp::Vertex>> border_v_map;

    double calculateAngle(const std::vector<Eigen::Vector2d>& border1, const std::vector<Eigen::Vector2d>& border2);
    Eigen::Vector2d fitLine(const std::vector<Eigen::Vector2d>& points);
    void order_data(std::vector<Eigen::Vector2d>& vec);
    Point_2_eigen customRotate(const Point_2_eigen& pt, double angle_radians);
    void add_mesh(pmp::SurfaceMesh& mesh, pmp::SurfaceMesh& mesh_original);
    pmp::Vertex find_vertex_by_coordinates(const pmp::SurfaceMesh& m, const Point_3_eigen& pt);
    std::vector<pmp::Vertex> left, right, up, down;

    static constexpr double EPSILON = 1e-6;

    friend class SurfaceParametrization;
};
