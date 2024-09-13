#pragma once

#include <ceres/ceres.h>
#include <cmath>
#include <utility>
#include <vector>

template <typename T>
std::pair<std::vector<T>, std::vector<T>> calculate_control_points(const std::pair<T, T>& point, T curve_strength);

template <typename T>
void spectre_border(T a, T b, T curve_strength, std::vector<T>& x_vals, std::vector<T>& y_vals);
