// SurfaceParametrization.h
#pragma once

// Standard libraries
#include <cstddef>
#include <fstream>
#include <iostream>
#include <cmath>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

// Eigen
#include <Eigen/Dense>

#include "MeshDefinition.h"
#include "SurfaceParametrizationHelperInterface.h"
#include "MeshCuttingHelperInterface.h"

const fs::path PROJECT_PATH_ = MeshCartographyLib_SOURCE_DIR;
const fs::path MESH_FOLDER = PROJECT_PATH_  / "meshes";
const unsigned int PARAMETERIZATION_ITERATIONS = 9;

class SurfaceParametrization {
public:
    struct MeshMeta{
        std::string mesh_path;
    };

    SurfaceParametrization();

    std::tuple<std::vector<int64_t>, Eigen::MatrixXd, Eigen::MatrixXd, std::string> create_uv_surface(
        std::string mesh_file_path,
        int32_t start_node_int
    );

    std::string get_mesh_name(const std::string mesh_file_path);

    bool check_point_in_polygon(const Point_2_eigen& point);

    std::vector<Eigen::Vector2d> corners;
    Polygon_eigen polygon;
    std::vector<pmp::Vertex> polygon_v;
    MeshMeta meshmeta;

private:
    Eigen::MatrixXd vertice_UV;
    Eigen::MatrixXd vertice_3D;
    std::string mesh_3D_file_path;

    std::vector<int64_t> calculate_uv_surface(pmp::Vertex start_node);
    void save_uv_as_mesh(const pmp::SurfaceMesh& mesh, const fs::path& filename);
    void extract_polygon_border_edges(const std::string& mesh_uv_path);

friend class SurfaceParametrizationTest;
};
