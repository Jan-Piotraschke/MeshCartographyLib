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

#include "SpectreMonotileHelper.h"

std::pair<std::vector<double>, std::vector<double>> calculate_control_points(const std::pair<double, double>& point, double curve_strength) {
    double x = point.first, y = point.second;
    double normal_x = y, normal_y = -x;
    std::vector<double> control1 = {-curve_strength * normal_x + x / 2, -curve_strength * normal_y + y / 2};
    std::vector<double> control2 = {curve_strength * normal_x + x / 2, curve_strength * normal_y + y / 2};
    return {control1, control2};
}

void draw_monotile(double a, double b, double curve_strength, std::vector<double>& x_vals, std::vector<double>& y_vals) {
    double cos_angle = std::cos(M_PI / 3);
    double sin_angle = std::sin(M_PI / 3);

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

    x_vals.push_back(0);
    y_vals.push_back(0);

    for (const auto& [dx, dy] : direction_vectors) {
        auto [control1, control2] = calculate_control_points({dx, dy}, curve_strength);
        std::vector<double> P0 = {x_vals.back(), y_vals.back()};
        std::vector<double> P1 = {control1[0] + x_vals.back(), control1[1] + y_vals.back()};
        std::vector<double> P2 = {control2[0] + x_vals.back(), control2[1] + y_vals.back()};
        std::vector<double> P3 = {dx + x_vals.back(), dy + y_vals.back()};

        for (double t = 0; t <= 1; t += 0.02) {
            double mt = 1 - t;
            double mt2 = mt * mt;
            double t2 = t * t;

            double x = mt2 * mt * P0[0] + 3 * mt2 * t * P1[0] + 3 * mt * t2 * P2[0] + t2 * t * P3[0];
            double y = mt2 * mt * P0[1] + 3 * mt2 * t * P1[1] + 3 * mt * t2 * P2[1] + t2 * t * P3[1];

            x_vals.push_back(x);
            y_vals.push_back(y);
        }
    }
}
