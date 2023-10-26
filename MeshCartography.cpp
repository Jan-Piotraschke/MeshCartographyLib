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


struct MonotileAreaCostFunction {
    MonotileAreaCostFunction(double a, double b) : a_(a), b_(b) {}

    template <typename T>
    bool operator()(const T* const curve_strength, T* residual) const {
        // Make sure curve_strength is within the boundaries
        T cs = Clamp(*curve_strength, T(0), T(3));

        // You can use your existing draw_monotile function here to generate the border
        std::vector<T> x_vals, y_vals;
        draw_monotile(T(a_), T(b_), cs, x_vals, y_vals);

        // Calculate the area using the Shoelace formula
        T area = T(0);
        int n = x_vals.size();
        for (int i = 0; i < n - 1; ++i) {
            area += x_vals[i] * y_vals[i + 1];
            area -= y_vals[i] * x_vals[i + 1];
        }
        area += x_vals[n - 1] * y_vals[0];
        area -= y_vals[n - 1] * x_vals[0];
        area = ceres::abs(area) / T(2);

        // Since Ceres performs minimization, we return the negative area to maximize it
        residual[0] = -area;
        return true;
    }

    double a_, b_;
};


int main() {
    double a = 1.0;
    double b = 1.0;
    double curve_strength = 1.0; // Initial guess

    ceres::Problem problem;

    problem.AddResidualBlock(
        new ceres::AutoDiffCostFunction<MonotileAreaCostFunction, 1, 1>(
            new MonotileAreaCostFunction(a, b)),
        nullptr, &curve_strength);

    ceres::Solver::Options options;
    options.minimizer_progress_to_stdout = true;
    options.linear_solver_type = ceres::DENSE_QR;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    std::cout << summary.FullReport() << "\n";
    std::cout << "Final curve_strength: " << curve_strength << "\n";

    return 0;
}


// struct ControlPointsFunctor {
//     template <typename T>
//     bool operator()(const T* const point, T* residuals) const {
//         T x = point[0];
//         T y = point[1];
//         T curve_strength = T(1.0); // Assuming some curve strength for this example

//         T normal_x = y;
//         T normal_y = -x;

//         // Control point 1
//         residuals[0] = -curve_strength * normal_x + x / T(2.0);
//         residuals[1] = -curve_strength * normal_y + y / T(2.0);

//         // Control point 2
//         residuals[2] = curve_strength * normal_x + x / T(2.0);
//         residuals[3] = curve_strength * normal_y + y / T(2.0);

//         return true;
//     }
// };

// int main() {
//     double point[2] = {1.0, 2.0};

//     // Build the problem.
//     ceres::Problem problem;

//     // Set up the cost function (also known as residual). This will be minimized using
//     // the Levenberg-Marquardt method.
//     ceres::CostFunction* cost_function =
//         new ceres::AutoDiffCostFunction<ControlPointsFunctor, 4, 2>(new ControlPointsFunctor);

//     problem.AddResidualBlock(cost_function, nullptr, point);

//     // Run the solver!
//     ceres::Solver::Options options;
//     options.linear_solver_type = ceres::DENSE_QR;
//     options.minimizer_progress_to_stdout = true;

//     ceres::Solver::Summary summary;
//     Solve(options, &problem, &summary);

//     std::cout << summary.BriefReport() << "\n";
//     std::cout << "Original point: (1, 2)\n";
//     std::cout << "Optimized point: (" << point[0] << ", " << point[1] << ")\n";

//     return 0;
// }




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
