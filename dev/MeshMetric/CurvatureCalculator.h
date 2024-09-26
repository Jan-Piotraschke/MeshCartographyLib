#pragma once

#include "MeshDefinition.h"
#include <CGAL/Polygon_mesh_processing/compute_normal.h>
#include <CGAL/Polygon_mesh_processing/measure.h>
#include <CGAL/Surface_mesh.h>
#include <unordered_map>

namespace PMP = CGAL::Polygon_mesh_processing;

class CurvatureCalculator
{
  public:
    using Mesh = _3D::Mesh;
    using vertex_descriptor = boost::graph_traits<Mesh>::vertex_descriptor;
    using FaceRange = typename boost::graph_traits<Mesh>::face_descriptor;

    CurvatureCalculator(Mesh& mesh);
    void compute_vertex_curvatures();

    // Accessor for curvature property map
    const Mesh::Property_map<vertex_descriptor, double>& get_curvature_map() const;

  private:
    Mesh& mesh_;
    Mesh::Property_map<vertex_descriptor, double> curvature_map_;

    void compute_vertex_gaussian_curvature();
};
