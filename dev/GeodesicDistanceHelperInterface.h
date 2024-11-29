#pragma once

#include <Eigen/Dense>

class GeodesicDistanceHelperInterface
{
  public:
    virtual Eigen::MatrixXd get_mesh_distance_matrix() = 0;
};
