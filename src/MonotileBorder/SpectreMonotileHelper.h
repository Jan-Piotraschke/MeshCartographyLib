#pragma once

#include <cmath>
#include <vector>
#include <utility>

std::pair<std::vector<double>, std::vector<double>> calculate_control_points(const std::pair<double, double>& point, double curve_strength);

void draw_monotile(double a, double b, double curve_strength, std::vector<double>& x_vals, std::vector<double>& y_vals);
