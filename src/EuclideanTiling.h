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

private:
    SurfaceParametrization& surface_parametrization;

    Eigen::Matrix<double, Eigen::Dynamic, 2>& r_UV;
    Eigen::Matrix<double, Eigen::Dynamic, 2>& r_UV_old;
    Eigen::VectorXi& n;

    const bool original_mesh;

    std::tuple<Eigen::Vector2d, double, Eigen::Vector2d> processPoints(
        const Eigen::Vector2d& pointA,
        const Eigen::Vector2d& point_outside,
        double n
    );

    static constexpr double KACHEL_ROTATION = 90.0;
};
