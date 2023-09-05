// GeodesicDistance.h

#pragma once

#include <string>
#include <vector>
#include <iostream>
#include <fstream>

// Eigen
#include <Eigen/Dense>

// Boost libraries
#include <boost/graph/graph_traits.hpp>
#include <boost/property_map/property_map.hpp>
#include <boost/filesystem.hpp>
namespace fs = boost::filesystem;
const fs::path PROJECT_PATH_GD = PROJECT_SOURCE_DIR;

// Boost libraries
#include <boost/filesystem.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/property_map/property_map.hpp>

// CGAL libraries
#include <CGAL/boost/graph/properties.h>
#include <CGAL/boost/graph/graph_traits_Surface_mesh.h>
#include <CGAL/boost/graph/breadth_first_search.h>
#include <CGAL/IO/read_off_points.h>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/boost/graph/properties.h>
#include <CGAL/boost/graph/graph_traits_Surface_mesh.h>
#include <CGAL/Heat_method_3/Surface_mesh_geodesic_distances_3.h>

// Basic type definitions and constants
using Kernel = CGAL::Simple_cartesian<double>;
using Point_2 = Kernel::Point_2;
using Point_3 = Kernel::Point_3;
using Triangle_mesh = CGAL::Surface_mesh<Point_3>;
using vertex_descriptor = boost::graph_traits<Triangle_mesh>::vertex_descriptor;
using Vertex_distance_map = Triangle_mesh::Property_map<vertex_descriptor, double>;

// 3D definitions
namespace _3D {
    using Mesh = CGAL::Surface_mesh<Point_3>;
    using vertex_descriptor = boost::graph_traits<Mesh>::vertex_descriptor;
    using halfedge_descriptor = boost::graph_traits<Mesh>::halfedge_descriptor;
    using edge_descriptor = boost::graph_traits<Mesh>::edge_descriptor;
    using UV_pmap = Mesh::Property_map<halfedge_descriptor, Point_2>;
}

class GeodesicDistance {
public:
    GeodesicDistance(
        std::string mesh_path_input
    );

    void get_all_distances();
    void calculate_tessellation_distance();
    void calculate_edge_distances(
        _3D::Mesh mesh,
        _3D::vertex_descriptor start_node,
        std::vector<_3D::vertex_descriptor>& predecessor_pmap,
        std::vector<int>& distance
    );

private:
    std::string mesh_path;
    std::string mesh_path_3D;
    std::string mesh_path_UV;
    std::string mesh_name;
    std::string mesh_name_3D;
    std::string mesh_name_UV;

    template<typename MatrixType>
    void save_distance_matrix(MatrixType& distance_matrix_v) {

        const static Eigen::IOFormat CSVFormat(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", "\n");

        std::cout << "Saving distance matrix to file..." << std::endl;
        std::string distance_matrix_path = PROJECT_PATH_GD.string() + "/meshes/data/" + mesh_name + "_distance_matrix_static.csv";
        std::ofstream file(distance_matrix_path);
        file << distance_matrix_v.format(CSVFormat);
        file.close();
        std::cout << "saved" << std::endl;
    };

    void fill_distance_matrix(
        Eigen::MatrixXd& distance_matrix,
        int closest_vertice
    );

    std::vector<double> geo_distance(
        int32_t start_node = 0
    );
};