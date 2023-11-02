/**
 * @file        AutoDiffMonotileOptimizerHelper.cpp
 * @brief       Optimize the border of the spectre monotile using automatic differentiation
 *
 * @author      Jan-Piotraschke
 * @date        2023-Oct-27
 * @license     Apache License 2.0
 *
 * @bug         -
 * @todo        -
 */

#include "AutoDiffMonotileOptimizerHelper.h"
#include "MonotileBorder/SpectreMonotileHelper.h"

// Utilizing a clamp function to ensure our values stay within a specific range
template <typename T>
T clamp(const T& value, const T& low, const T& high) {
    return std::max(low, std::min(value, high));
}

// ========================================
// AreaCalculator Implementation
// ========================================

// Optimization objective of the cost function: Area of the border
template <typename T>
T AreaCalculator::computeArea(const std::vector<T>& x_vals, const std::vector<T>& y_vals) const {
    T area = T(0);
    int n = x_vals.size();
    for (int i = 0; i < n - 1; ++i) {
        area += x_vals[i] * y_vals[i + 1];
        area -= y_vals[i] * x_vals[i + 1];
    }
    area += x_vals[n - 1] * y_vals[0];
    area -= y_vals[n - 1] * x_vals[0];
    return ceres::abs(area) / T(2);
}


// ========================================
// MonotileAreaCostFunction Implementation
// ========================================

// The cost function represents our optimization objective
MonotileAreaCostFunction::MonotileAreaCostFunction(double a, double b)
    : a_(a), b_(b) {}

// Operator() overloading to define the behavior of our cost function
template <typename T>
bool MonotileAreaCostFunction::operator()(const T* const curve_strength, T* residual) const {
    T cs = clamp(*curve_strength, T(0), T(3));  // Clamping to ensure curve_strength stays within [0, 3]

    std::vector<T> x_vals, y_vals;
    spectre_border(T(a_), T(b_), cs, x_vals, y_vals, 1000);
    T area = area_calculator.computeArea(x_vals, y_vals);

    // The next line involves Automatic Differentiation (AutoDiff) where gradients are calculated
    // Jet is an operation that takes a differentiable function f and produces a polynomial, the truncated Taylor polynomial of f
    final_area = ceres::Jet<double, 1>(area).a;

    residual[0] = -area;  // The residual, which the optimizer aims to minimize
    return true;
}

double MonotileAreaCostFunction::computeArea(double curve_strength) const {
    std::vector<double> x_vals, y_vals;
    spectre_border(a_, b_, curve_strength, x_vals, y_vals, 1000);
    return area_calculator.computeArea(x_vals, y_vals);
}


// ========================================
// OptimizationProblem Implementation
// ========================================

// Set up and run the optimization problem
void OptimizationProblem::run(double a, double b, double& curve_strength) {
    ceres::Problem problem;
    // Create a custom cost function
    MonotileAreaCostFunction* cost_function = new MonotileAreaCostFunction(a, b);

    // Automatically compute derivatives for optimization
    problem.AddResidualBlock(
        new ceres::AutoDiffCostFunction<MonotileAreaCostFunction, 1, 1>(cost_function),
        nullptr, &curve_strength);

    // Setting bounds for curve_strength
    problem.SetParameterLowerBound(&curve_strength, 0, lower_bound_);
    problem.SetParameterUpperBound(&curve_strength, 0, upper_bound_);

    // Configure the solver options
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;  // Use a dense QR decomposition based solver
    options.minimizer_progress_to_stdout = true;  // Print progress to stdout

    ceres::Solver::Summary summary;  // Store informations about the optimization process

    // Run the optimization
    ceres::Solve(options, &problem, &summary);

    std::cout << summary.BriefReport() << "\n";
    std::cout << "Final curve_strength: " << curve_strength << "\n";
    std::cout << "Final area: " << cost_function->final_area << "\n";
}

void OptimizationProblem::setBounds(double lower_bound, double upper_bound) {
    lower_bound_ = lower_bound;
    upper_bound_ = upper_bound;
}
