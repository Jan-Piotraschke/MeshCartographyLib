#pragma once

#include <ceres/ceres.h>
#include <cmath>
#include <iostream>
#include <vector>

class AreaCalculator
{
  public:
    template <typename T>
    T computeArea(const std::vector<T>& x_vals, const std::vector<T>& y_vals) const;
};

struct MonotileAreaCostFunction
{
    MonotileAreaCostFunction(double a, double b);

    template <typename T>
    bool operator()(const T* const curve_strength, T* residual) const;

    double computeArea(double curve_strength) const;

    mutable double final_area = 0.0;
    AreaCalculator area_calculator;
    double a_, b_;
};

class OptimizationProblem
{
  public:
    void run(double a, double b, double& curve_strength);
    void setBounds(double lower_bound, double upper_bound);

  private:
    double lower_bound_ = -18.0;
    double upper_bound_ = 3.0;
};
