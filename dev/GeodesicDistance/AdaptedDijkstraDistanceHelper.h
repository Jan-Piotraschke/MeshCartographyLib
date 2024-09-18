#pragma once

#include <CGAL/Surface_mesh_shortest_path.h>
#include <Eigen/Dense>
#include <fstream>
#include <limits>
#include <set>

#include "GeodesicDistanceHelperInterface.h"
#include "MeshDefinition.h"
#include <boost/filesystem.hpp>
namespace fs = boost::filesystem;

class AdaptedDijkstraDistanceHelper : public GeodesicDistanceHelperInterface
{
  public:
    AdaptedDijkstraDistanceHelper(fs::path mesh_path);

    Eigen::MatrixXd get_mesh_distance_matrix() override;

    void fill_distance_matrix(_3D::Mesh& mesh, Eigen::MatrixXd& distance_matrix, _3D::vertex_descriptor vertex);

  private:
    fs::path mesh_path;

    std::vector<double> calculate_geodesic_distance(_3D::Mesh& mesh, _3D::vertex_descriptor start_vertex);
};
