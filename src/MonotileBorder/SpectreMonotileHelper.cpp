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

// ! NOTE: just think of "T" as "double", because that's what it is in the end
template <typename T>
std::pair<std::vector<T>, std::vector<T>> calculate_control_points(const std::pair<T, T>& point, T curve_strength) {
    T x = point.first, y = point.second;
    T normal_x = y, normal_y = -x;
    std::vector<T> control1 = {-curve_strength * normal_x + x / T(2), -curve_strength * normal_y + y / T(2)};
    std::vector<T> control2 = {curve_strength * normal_x + x / T(2), curve_strength * normal_y + y / T(2)};
    return {control1, control2};
}

template <typename T>
void spectre_border(T a, T b, T curve_strength, std::vector<T>& x_vals, std::vector<T>& y_vals) {
    T cos_angle = ceres::cos(T(M_PI) / T(3));
    T sin_angle = ceres::sin(T(M_PI) / T(3));

    std::vector<std::pair<T, T>> direction_vectors = {
        {cos_angle * b, sin_angle * b},
        {b, T(0)},
        {T(0), a},
        {sin_angle * a, cos_angle * a},
        {cos_angle * b, -sin_angle * b},
        {-cos_angle * b, -sin_angle * b},
        {sin_angle * a, -cos_angle * a},
        {T(0), -a},
        {T(0), -a},
        {-sin_angle * a, -cos_angle * a},
        {-cos_angle * b, sin_angle * b},
        {-b, T(0)},
        {T(0), a},
        {-sin_angle * a, cos_angle * a},
    };

    x_vals.push_back(T(0));
    y_vals.push_back(T(0));

    for (const auto& [dx, dy] : direction_vectors) {
        auto [control1, control2] = calculate_control_points<T>({dx, dy}, curve_strength);
        std::vector<T> P0 = {x_vals.back(), y_vals.back()};
        std::vector<T> P1 = {control1[0] + x_vals.back(), control1[1] + y_vals.back()};
        std::vector<T> P2 = {control2[0] + x_vals.back(), control2[1] + y_vals.back()};
        std::vector<T> P3 = {dx + x_vals.back(), dy + y_vals.back()};

        for (double t = 0; t <= 1; t += 0.02) {
            T T_t = T(t);
            T mt = T(1) - T_t;
            T mt2 = mt * mt;
            T t2 = T_t * T_t;

            T x = mt2 * mt * P0[0] + T(3) * mt2 * T_t * P1[0] + T(3) * mt * t2 * P2[0] + t2 * T_t * P3[0];
            T y = mt2 * mt * P0[1] + T(3) * mt2 * T_t * P1[1] + T(3) * mt * t2 * P2[1] + t2 * T_t * P3[1];

            x_vals.push_back(x);
            y_vals.push_back(y);
        }
    }
}

// Explicit template instantiation
template void spectre_border<double>(double a, double b, double curve_strength, std::vector<double>& x_vals, std::vector<double>& y_vals);
template void spectre_border<ceres::Jet<double, 1>>(ceres::Jet<double, 1> a, ceres::Jet<double, 1> b, ceres::Jet<double, 1> curve_strength, std::vector<ceres::Jet<double, 1>>& x_vals, std::vector<ceres::Jet<double, 1>>& y_vals);

