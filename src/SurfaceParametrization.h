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
        std::string mesh_path_virtual;
    };

    explicit SurfaceParametrization(bool& free_boundary);

    std::tuple<std::vector<int64_t>, Eigen::MatrixXd, Eigen::MatrixXd, std::string> create_uv_surface(
        std::string mesh_file_path,
        int32_t start_node_int
    );


    std::string get_mesh_name(
        const std::string mesh_file_path
    );

    std::tuple<std::vector<int64_t>, Eigen::MatrixXd, Eigen::MatrixXd, std::string> get_virtual_mesh();

    bool check_point_in_polygon(
        const Point_2_eigen& point,
        bool is_original_mesh
    );
    void create_kachelmuster();

    std::vector<Point_2_eigen> left, right, up, down;
    std::tuple<std::vector<Point_2_eigen>,
                std::vector<Point_2_eigen>,
                std::vector<Point_2_eigen>,
                std::vector<Point_2_eigen>
            > get_borders(){
                return {left, right, up, down};
            };
private:
    MeshMeta meshmeta;
    int combine_key;

    class Tessellation {
        public:
            Tessellation(SurfaceParametrization& sp) : parent(sp) {}

            void analyseSides();
            void create_kachelmuster();
            std::tuple<
                std::vector<Point_2_eigen>,
                std::vector<Point_2_eigen>,
                std::vector<Point_2_eigen>,
                std::vector<Point_2_eigen>
            > get_sides(){
                return {left_border, right_border, up_border, down_border};
            };
            friend class SurfaceParametrization;

        private:
            SurfaceParametrization& parent;
            std::string docking_side;
            int target_index;

            Point_3 get_point_3d(
                _3D::Mesh& mesh,
                _3D::vertex_descriptor& v,
                std::vector<_3D::vertex_descriptor>& border_list
            );
            Point_2 customRotate(const Point_2& pt, double angle_radians);
            void process_mesh(const std::string& mesh_path, _3D::Mesh& mesh_original, double rotation_angle, int shift_x, int shift_y);
            void find_vertex_index(const Point_2& target);
            void rotate_and_shift_mesh(_3D::Mesh& mesh, double angle_degrees, int shift_x_coordinates, int shift_y_coordinates);
            void add_mesh(_3D::Mesh& mesh, _3D::Mesh& mesh_original);
            bool are_almost_equal(float a, float b);
            _3D::vertex_descriptor find_vertex_by_coordinates(
                const _3D::Mesh& m,
                const Point_3& pt
            );
            std::vector<_3D::vertex_descriptor> left, right, up, down;
            std::vector<Point_2_eigen> left_border, right_border, up_border, down_border;

            static constexpr double EPSILON = 1e-6;

    };

    Polygon_2 polygon;
    std::vector<_3D::vertex_descriptor> polygon_v;
    bool& free_boundary;
    Polygon_2 polygon_virtual;
    Eigen::MatrixXd vertices_UV_virtual;
    Eigen::MatrixXd vertices_3D_virtual;
    Eigen::MatrixXd vertice_UV;
    Eigen::MatrixXd vertice_3D;
    std::vector<int64_t> h_v_mapping_vector_virtual;
    std::string mesh_3D_file_path;

    SMP::Error_code parameterize_UV_mesh(
        UV::Mesh mesh,
        UV::halfedge_descriptor bhd,
        _3D::UV_pmap uvmap
    );

    UV::Mesh create_UV_mesh(
        _3D::Mesh& mesh,
        const std::vector<_3D::edge_descriptor> calc_edges
    );

    void load3DMeshes(
        const std::string& path,
        _3D::Mesh& sm,
        _3D::Mesh& sm_virtual
    );

    std::tuple<Point_3, Point_2, int64_t> getMeshData(
        const UV::vertex_descriptor& vd,
        const UV::Mesh& mesh,
        const _3D::Mesh& sm,
        _3D::UV_pmap& _uvmap
    );

    std::vector<int64_t> calculate_uv_surface(
        _3D::vertex_descriptor start_node,
        int uv_mesh_number
    );

    void save_UV_mesh(
        UV::Mesh _mesh,
        UV::halfedge_descriptor _bhd,
        _3D::UV_pmap _uvmap,
        const std::string mesh_path,
        int uv_mesh_number
    );

    void extract_polygon_border_edges(
        const std::string& mesh_uv_path,
        bool is_original_mesh
    );

friend class SurfaceParametrizationTest;
};
