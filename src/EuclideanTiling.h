// EuclideanTiling.h
#pragma once

#include <tuple>
#include <Eigen/Dense>

#include "SurfaceParametrization.h"

class EuclideanTiling {
public:
    EuclideanTiling(
        SurfaceParametrization& surface_parametrization,
        Eigen::Matrix<double, Eigen::Dynamic, 2>& r_UV,
        Eigen::Matrix<double, Eigen::Dynamic, 2>& r_UV_old,
        Eigen::VectorXi& n
    );

    void opposite_seam_edges_square_border();
    void diagonal_seam_edges_square_border();
    std::pair<std::string, Point_2_eigen> check_border_crossings(
        const Point_2_eigen& start_eigen,
        const Point_2_eigen& end_eigen
    );

private:
    SurfaceParametrization& surface_parametrization;

    Eigen::Matrix<double, Eigen::Dynamic, 2>& r_UV;
    Eigen::Matrix<double, Eigen::Dynamic, 2>& r_UV_old;
    Eigen::VectorXi& n;

    const bool original_mesh;
    std::vector<Point_2_eigen> left, right, up, down;

    std::tuple<Eigen::Vector2d, double, Eigen::Vector2d> processPoints(
        const Eigen::Vector2d& pointA,
        const Eigen::Vector2d& point_outside,
        double n
    );

    bool is_point_on_segment(const Point_2_eigen& P, const Point_2_eigen& A, const Point_2_eigen& B);
    boost::optional<Point_2_eigen> intersection_point(const Segment_2_eigen& line, const std::vector<Point_2_eigen>& border);

    static constexpr double KACHEL_ROTATION = 90.0;
};
