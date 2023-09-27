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

// Boost libraries
#include <boost/optional.hpp>
#include <boost/optional/optional_io.hpp>
#include <boost/filesystem.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/property_map/property_map.hpp>
namespace fs = boost::filesystem;

#include "MeshDefinition.h"
#include "SurfaceParametrizationHelperInterface.h"

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

    bool check_point_in_polygon(
        const Point_2_eigen& point,
        bool is_original_mesh
    );

    Polygon_2 polygon;
    std::vector<_3D::vertex_descriptor> polygon_v;
    MeshMeta meshmeta;

private:
    int combine_key;

    Eigen::MatrixXd vertice_UV;
    Eigen::MatrixXd vertice_3D;
    std::string mesh_3D_file_path;
    bool has_boundary(const pmp::SurfaceMesh& mesh);
    void setup_boundary_constraints(pmp::SurfaceMesh& mesh);
    void harmonic_parameterization(pmp::SurfaceMesh& mesh, bool use_uniform_weights = false);
    std::tuple<Point_3, Point_2, int64_t> getMeshData(
        const UV::vertex_descriptor& vd,
        const UV::Mesh& mesh,
        const _3D::Mesh& sm,
        _3D::UV_pmap& _uvmap
    );

    std::vector<int64_t> calculate_uv_surface(
        _3D::vertex_descriptor start_node
    );
    void save_uv_as_mesh(const pmp::SurfaceMesh& mesh, const std::string& filename);
    void extract_polygon_border_edges(
        const std::string& mesh_uv_path
    );

friend class SurfaceParametrizationTest;
};
