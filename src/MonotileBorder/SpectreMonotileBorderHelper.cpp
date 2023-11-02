/**
 * @file        SpectreMonotileHelper.cpp
 * @brief       Create the border of the spectre monotile
 *
 * @author      Jan-Piotraschke
 * @date        2023-Oct-26
 * @license     Apache License 2.0
 *
 * @bug         -
 * @todo        -
 */

#include "SpectreMonotileBorderHelper.h"

SpectreMonotileBorderHelper::SpectreMonotileBorderHelper(
    pmp::SurfaceMesh& mesh,
    pmp::Vertex& start_vertex
)
    : mesh(mesh),
    start_vertex(start_vertex)
{};

void SpectreMonotileBorderHelper::setup_spectre_monotile_boundary_constraints(double a, double b, double curve_strength)
{
    // get properties
    auto points = mesh.vertex_property<pmp::Point>("v:point");
    auto tex = mesh.vertex_property<pmp::TexCoord>("v:tex");

    // Initialize all texture coordinates to the origin.
    for (auto v : mesh.vertices()) {
        tex[v] = pmp::TexCoord(0.0, 0.0);
    }

    // Calculate spectre monotile border
    std::vector<double> x_vals, y_vals;
    spectre_border(a, b, curve_strength, x_vals, y_vals);

    // // Compute the total length of the border
    // double totalLength = 0.0;
    // for (size_t i = 0; i < x_vals.size() - 1; ++i) {
    //     double dx = x_vals[i+1] - x_vals[i];
    //     double dy = y_vals[i+1] - y_vals[i];
    //     totalLength += std::sqrt(dx * dx + dy * dy);
    // }

    // // Assuming the spectre has 14 edges
    // double sideLength = totalLength / 14.0;

    // Initialize corners
    initializeCorners(a, b, curve_strength);
}


void SpectreMonotileBorderHelper::initializeCorners(double a, double b, double curve_strength) {
    corners.clear();

    // Define the direction vectors for your spectre shape
    double cos_angle = ceres::cos(M_PI / 3.0);
    double sin_angle = ceres::sin(M_PI / 3.0);
    std::vector<std::pair<double, double>> direction_vectors = {
        {cos_angle * b, sin_angle * b},
        {b, 0},
        {0, a},
        {sin_angle * a, cos_angle * a},
        {cos_angle * b, -sin_angle * b},
        {-cos_angle * b, -sin_angle * b},
        {sin_angle * a, -cos_angle * a},
        {0, -a},
        {0, -a},
        {-sin_angle * a, -cos_angle * a},
        {-cos_angle * b, sin_angle * b},
        {-b, 0},
        {0, a},
        {-sin_angle * a, cos_angle * a},
    };

    // Push the initial point to the corners
    corners.push_back(Eigen::Vector2d(0.0, 0.0));

    // Calculate corner points based on the direction vectors
    for (const auto& [dx, dy] : direction_vectors) {
        Eigen::Vector2d corner_point = {corners.back().position[0] + dx, corners.back().position[1] + dy};
        corners.push_back(Eigen::Vector2d(corner_point[0], corner_point[1]));
    }

    // Round the corners values to 0 if they are close enough
    for (auto& corner : corners) {
        if (std::abs(corner.position[0]) < TOLERANCE) corner.position[0] = 0.0;
        if (std::abs(corner.position[1]) < TOLERANCE) corner.position[1] = 0.0;
    }

    // delete the last corner
    corners.pop_back();
}
