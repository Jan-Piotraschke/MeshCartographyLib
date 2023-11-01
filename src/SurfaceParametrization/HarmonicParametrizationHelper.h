#pragma once

#include "SurfaceParametrizationHelperInterface.h"

class HarmonicParametrizationHelper : public ParametrizationHelperInterface{
public:
    HarmonicParametrizationHelper(
        pmp::SurfaceMesh& mesh,
        pmp::Vertex& start_vertex,
        std::vector<Eigen::Vector2d>& corner_coordinates
    );

    void parameterize_UV_mesh(bool use_uniform_weights = false) override;

private:
    pmp::SurfaceMesh& mesh;
    pmp::Vertex& start_vertex;
    std::vector<Eigen::Vector2d>& corner_coordinates;
    bool use_uniform_weights;

    bool has_boundary();
};
