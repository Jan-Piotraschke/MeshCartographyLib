---
output_filename: "CachedGeodesicDistanceHelper"

brief: "Zuständing für das caching der berechneten Daten des HeatDistanceHelper"
---

# Cached Geodesic Distance Helper

The purpose of this module is to provide functions for caching the calculated data of the HeatDistanceHelper.

```cpp
#include "CachedGeodesicDistanceHelper.h"

CachedGeodesicDistanceHelper::CachedGeodesicDistanceHelper(fs::path mesh_path)
    : mesh_path(mesh_path), geodesic_distance_helper(mesh_path)
{
}
```

## Calculate the distance using the Heat Method

```cpp
Eigen::MatrixXd CachedGeodesicDistanceHelper::get_mesh_distance_matrix()
{
    fs::path cache_path = mesh_path.parent_path() / (mesh_path.filename().string() + "_distance_matrix_static.csv");
    if (!boost::filesystem::exists(cache_path))
    {
        // Calculate the distance matrix of the static 3D mesh
        Eigen::MatrixXd distance_matrix = geodesic_distance_helper.get_mesh_distance_matrix();

        save_csv(distance_matrix, cache_path);
    }
    Eigen::MatrixXd distance_matrix = load_csv<Eigen::MatrixXd>(cache_path);

    return distance_matrix;
}
```
