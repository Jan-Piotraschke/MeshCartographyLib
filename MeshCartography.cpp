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
#include <boost/filesystem.hpp>

#include "SurfaceParametrization/SurfaceParametrization.h"

const boost::filesystem::path PROJECT_PATH = MeshCartographyLib_SOURCE_DIR;

int main()
{
    std::string mesh_path = PROJECT_PATH.string() + "/meshes/ellipsoid_x4.off";
    bool free_boundary = false;

    SurfaceParametrization surface_parametrization(free_boundary);
    auto result = surface_parametrization.create_uv_surface(mesh_path, 0);
    auto mesh_UV_path = std::get<3>(result);

    auto mesh_UV_name = surface_parametrization.get_mesh_name(mesh_UV_path);

    // Create the tessellation mesh
    surface_parametrization.create_kachelmuster();

    auto result_virtual = surface_parametrization.get_virtual_mesh();

    return 0;
}
