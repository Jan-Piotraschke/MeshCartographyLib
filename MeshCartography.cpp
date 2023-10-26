/**
 * @file        main.cpp
 * @brief       Main file of the MeshCartographyLib project
 *
 * @author      Jan-Piotraschke
 * @date        2023-Sep-08
 * @version     0.1.0
 * @license     Apache License 2.0
 *
 * @bug         -
 * @todo        -
 */

#include <iostream>
#include <cmath>
#include <ceres/ceres.h>
#include "MonotileBorder/SpectreMonotileHelper.h"

#include <vector>

template <typename T>
T Clamp(const T& value, const T& low, const T& high) {
    return std::max(low, std::min(value, high));
}

class AreaCalculator {
public:
    template <typename T>
    T ComputeArea(const std::vector<T>& x_vals, const std::vector<T>& y_vals) const {
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
};


struct MonotileAreaCostFunction {
    MonotileAreaCostFunction(double a, double b) : a_(a), b_(b) {}

    mutable double final_area = 0.0;
    AreaCalculator area_calculator;

    template <typename T>
    bool operator()(const T* const curve_strength, T* residual) const {
        // Make sure curve_strength is within the boundaries
        T cs = Clamp(*curve_strength, T(0), T(3));

        std::vector<T> x_vals, y_vals;
        // Collect the monotile borders
        spectre_border(T(a_), T(b_), cs, x_vals, y_vals);

        // Calculate the area using the Shoelace formula
        T area = area_calculator.ComputeArea(x_vals, y_vals);
        final_area = ceres::Jet<double, 1>(area).a;

        // Since Ceres performs minimization, we return the negative area to maximize it
        residual[0] = -area;
        return true;
    }

    double ComputeArea(double curve_strength) const {
        std::vector<double> x_vals, y_vals;
        spectre_border(a_, b_, curve_strength, x_vals, y_vals);
        return area_calculator.ComputeArea(x_vals, y_vals);
    }

    double a_, b_;
};


class OptimizationProblem {
public:
    void Run(double a, double b, double& curve_strength) {
        // Build the problem.
        ceres::Problem problem;

        // Set up the only cost function (also known as residual). This uses
        // auto-differentiation to obtain the derivative (jacobian).
        MonotileAreaCostFunction* cost_function = new MonotileAreaCostFunction(a, b);
        problem.AddResidualBlock(
            new ceres::AutoDiffCostFunction<MonotileAreaCostFunction, 1, 1>(cost_function),
            nullptr, &curve_strength);

        problem.SetParameterLowerBound(&curve_strength, 0, lower_bound_);
        problem.SetParameterUpperBound(&curve_strength, 0, upper_bound_);

        // Run the solver!
        ceres::Solver::Options options;
        options.linear_solver_type = ceres::DENSE_QR;
        options.minimizer_progress_to_stdout = true;

        // Get the final report
        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);

        std::cout << summary.BriefReport() << "\n";
        std::cout << "Final curve_strength: " << curve_strength << "\n";
        std::cout << "Final area: " << cost_function->final_area << "\n";
    }

    void SetBounds(double lower_bound, double upper_bound) {
        lower_bound_ = lower_bound;
        upper_bound_ = upper_bound;
    }

private:
    double lower_bound_ = -18.0;
    double upper_bound_ = 3.0;
};


int main() {
    double a = 1.0;
    double b = 1.0;
    double curve_strength = 1.0; // Initial guess

    MonotileAreaCostFunction cost_function(a, b);
    double initial_area = cost_function.ComputeArea(curve_strength);
    std::cout << "Initial area: " << initial_area << "\n";

    OptimizationProblem optimization_problem;
    optimization_problem.SetBounds(0.2, 2.0);
    optimization_problem.Run(a, b, curve_strength);

    return 0;
}


// int main()
// {
//     std::string mesh_path = PROJECT_PATH.string() + "/meshes/ellipsoid_x4.off";
//     bool free_boundary = false;

//     SurfaceParametrization surface_parametrization;
//     auto result = surface_parametrization.create_uv_surface(mesh_path, 0);
//     auto mesh_UV_path = std::get<3>(result);

//     auto mesh_UV_name = surface_parametrization.get_mesh_name(mesh_UV_path);

//     // Create the tessellation mesh
//     Tessellation tessellation(surface_parametrization);
//     tessellation.create_kachelmuster();

//     return 0;
// }
