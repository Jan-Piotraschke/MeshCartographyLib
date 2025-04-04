#pragma once

#include <filesystem>
#include <optional>
namespace fs = std::filesystem;

// PMP libraries
#include "pmp/algorithms/differential_geometry.h"
#include "pmp/algorithms/laplace.h"
#include <pmp/algorithms/geodesics.h>
#include <pmp/io/io.h>
#include <pmp/io/read_off.h>
#include <pmp/io/write_off.h>
#include <pmp/surface_mesh.h>

using Scalar = double;
using Point_2_pmp = pmp::vec2;
using Point_3_pmp = pmp::vec3;
using Triangle_mesh_pmp = pmp::SurfaceMesh;
using vertex_descriptor_pmp = pmp::Vertex;
using edge_descriptor_pmp = pmp::Edge;
using halfedge_descriptor_pmp = pmp::Halfedge;

namespace _3D_pmp
{
using Mesh = pmp::SurfaceMesh;
using vertex_descriptor = pmp::Vertex;
using halfedge_descriptor = pmp::Halfedge;
using edge_descriptor = pmp::Edge;
} // namespace _3D_pmp

// Basic type definitions and constants
using Point_2_eigen = Eigen::Vector2d;
using Point_3_eigen = Eigen::Vector3d;
using Polygon_eigen = std::vector<Point_2_eigen>;

class Segment_2_eigen
{
  private:
    Point_2_eigen source_point, target_point;

  public:
    Segment_2_eigen(const Point_2_eigen& s, const Point_2_eigen& t) : source_point(s), target_point(t) {}

    Point_2_eigen source() const { return source_point; }
    Point_2_eigen target() const { return target_point; }
};
