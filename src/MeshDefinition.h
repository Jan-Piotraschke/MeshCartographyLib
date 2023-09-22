#pragma once

// Boost libraries
#include <boost/optional.hpp>
#include <boost/optional/optional_io.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/property_map/property_map.hpp>

// CGAL libraries
#include <CGAL/Segment_2.h>
#include <CGAL/intersections.h>
#include <CGAL/Aff_transformation_2.h>
#include <CGAL/IO/read_off_points.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/boost/graph/properties.h>
#include <CGAL/boost/graph/graph_traits_Surface_mesh.h>
#include <CGAL/boost/graph/Seam_mesh.h>
#include <CGAL/Polygon_mesh_processing/border.h>
#include <CGAL/boost/graph/breadth_first_search.h>
#include <CGAL/Polygon_mesh_processing/connected_components.h>
#include <CGAL/Polygon_mesh_processing/measure.h>
#include <CGAL/Surface_mesh_parameterization/IO/File_off.h>
#include <CGAL/Surface_mesh_parameterization/Square_border_parameterizer_3.h>
#include <CGAL/Surface_mesh_parameterization/Discrete_conformal_map_parameterizer_3.h>
#include <CGAL/Surface_mesh_parameterization/parameterize.h>
#include <CGAL/Surface_mesh_parameterization/ARAP_parameterizer_3.h>
namespace SMP = CGAL::Surface_mesh_parameterization;

// Basic type definitions and constants
using Kernel = CGAL::Simple_cartesian<double>;
using Segment_2 = Kernel::Segment_2;
using Point_2 = Kernel::Point_2;
using Point_3 = Kernel::Point_3;
using Polygon_2 = CGAL::Polygon_2<Kernel>;
using Triangle_mesh = CGAL::Surface_mesh<Point_3>;
using vertex_descriptor = boost::graph_traits<Triangle_mesh>::vertex_descriptor;
using Vertex_distance_map = Triangle_mesh::Property_map<vertex_descriptor, double>;

using Point_2_eigen = Eigen::Vector2d;

class Segment_2_eigen {
private:
    Point_2_eigen source_point, target_point;
public:
    Segment_2_eigen(const Point_2_eigen& s, const Point_2_eigen& t)
        : source_point(s), target_point(t) {}

    Point_2_eigen source() const { return source_point; }
    Point_2_eigen target() const { return target_point; }
};

// 3D definitions
namespace _3D {
    using Mesh = CGAL::Surface_mesh<Point_3>;
    using vertex_descriptor = boost::graph_traits<Mesh>::vertex_descriptor;
    using halfedge_descriptor = boost::graph_traits<Mesh>::halfedge_descriptor;
    using edge_descriptor = boost::graph_traits<Mesh>::edge_descriptor;
    using Seam_edge_pmap = Mesh::Property_map<edge_descriptor, bool>;
    using Seam_vertex_pmap = Mesh::Property_map<vertex_descriptor, bool>;
    using UV_pmap = Mesh::Property_map<halfedge_descriptor, Point_2>;
}

// UV definitions
namespace UV {
    using Mesh = CGAL::Seam_mesh<_3D::Mesh, _3D::Seam_edge_pmap, _3D::Seam_vertex_pmap>;
    using vertex_descriptor = boost::graph_traits<Mesh>::vertex_descriptor;
    using halfedge_descriptor = boost::graph_traits<Mesh>::halfedge_descriptor;
}
