#pragma once

// Boost libraries
#include <boost/optional.hpp>
#include <boost/optional/optional_io.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/property_map/property_map.hpp>
#include <boost/filesystem.hpp>
namespace fs = boost::filesystem;

// PMP libraries
#include <pmp/surface_mesh.h>
#include <pmp/io/read_off.h>
#include <pmp/io/write_off.h>
#include <pmp/io/io.h>
#include <pmp/algorithms/geodesics.h>
#include "pmp/algorithms/laplace.h"
#include "pmp/algorithms/differential_geometry.h"

using Scalar = double;
using Point_2_pmp = pmp::vec2;
using Point_3_pmp = pmp::vec3;
using Triangle_mesh_pmp = pmp::SurfaceMesh;
using vertex_descriptor_pmp = pmp::Vertex;
using edge_descriptor_pmp = pmp::Edge;
using halfedge_descriptor_pmp = pmp::Halfedge;

namespace _3D_pmp {
    using Mesh = pmp::SurfaceMesh;
    using vertex_descriptor = pmp::Vertex;
    using halfedge_descriptor = pmp::Halfedge;
    using edge_descriptor = pmp::Edge;
}

// Basic type definitions and constants
using Point_2_eigen = Eigen::Vector2d;
using Point_3_eigen = Eigen::Vector3d;
using Polygon_eigen = std::vector<Point_2_eigen>;

class Segment_2_eigen {
private:
    Point_2_eigen source_point, target_point;
public:
    Segment_2_eigen(const Point_2_eigen& s, const Point_2_eigen& t)
        : source_point(s), target_point(t) {}

    Point_2_eigen source() const { return source_point; }
    Point_2_eigen target() const { return target_point; }
};
