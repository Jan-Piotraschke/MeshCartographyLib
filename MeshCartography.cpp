/**
 * @file        main.cpp
 * @brief       Main file of the MeshCartographyLib project
 *
 * @author      Jan-Piotraschke
 * @date        2023-Oct-27
 * @license     Apache License 2.0
 *
 * @bug         -
 * @todo        -
 */

#include <iostream>
#include <filesystem>

#include "SurfaceParametrization/SurfaceParametrization.h"
#include "SurfaceParametrization/TessellationHelper.h"
#include "SurfaceParametrization/AutoDiffMonotileOptimizerHelper.h"
#include "MonotileBorder/SpectreMonotileHelper.h"

const std::filesystem::path PROJECT_PATH = MeshCartographyLib_SOURCE_DIR;

int main()
{
    std::string mesh_path = PROJECT_PATH.string() + "/meshes/ellipsoid_x4.off";
    bool free_boundary = false;

    SurfaceParametrization surface_parametrization;
    auto result = surface_parametrization.create_uv_surface(mesh_path, 0);
    auto mesh_UV_path = std::get<3>(result);

    auto mesh_UV_name = surface_parametrization.get_mesh_name(mesh_UV_path);

    // Create the tessellation mesh
    Tessellation tessellation(surface_parametrization);
    tessellation.create_kachelmuster();

    // Optimize the monotile border by reducing the area
    double a = 1.0;
    double b = 1.0;
    double curve_strength = 1.0; // Initial guess

    MonotileAreaCostFunction cost_function(a, b);
    double initial_area = cost_function.computeArea(curve_strength);
    std::cout << "Initial area: " << initial_area << "\n";

    OptimizationProblem optimization_problem;
    optimization_problem.setBounds(0, 2.0);
    optimization_problem.run(a, b, curve_strength);

    std::vector<double> x_vals, y_vals;
    spectre_border(a, b, curve_strength, x_vals, y_vals, 1000);

    // for (int i = 0; i < x_vals.size(); ++i) {
    //     std::cout << x_vals[i] << " " << y_vals[i] << "\n";
    // }

    return 0;
}
