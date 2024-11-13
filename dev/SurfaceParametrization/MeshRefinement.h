#pragma once

#include "MeshDefinition.h"
#include "Metric/CurvatureCalculator.h"
#include <CGAL/Polygon_mesh_processing/remesh.h>
#include <CGAL/Polygon_mesh_processing/transform.h>
#include <CGAL/property_map.h>

namespace PMP = CGAL::Polygon_mesh_processing;

class MeshRefinement
{
  public:
    using Mesh = _3D::Mesh;
    using vertex_descriptor = boost::graph_traits<Mesh>::vertex_descriptor;
    using EdgeDescriptor = boost::graph_traits<Mesh>::edge_descriptor;

    MeshRefinement(Mesh& mesh, const Mesh::Property_map<vertex_descriptor, double>& curvature_map);

    void refine_mesh();

  private:
    Mesh& mesh_;
    const Mesh::Property_map<vertex_descriptor, double>& curvature_map_;

    // Property map for edge target lengths
    Mesh::Property_map<EdgeDescriptor, double> edge_length_map_;

    void compute_edge_length_targets();
};
