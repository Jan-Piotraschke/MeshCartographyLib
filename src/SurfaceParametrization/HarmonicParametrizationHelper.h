#pragma once

#include "SurfaceParametrizationHelperInterface.h"

class HarmonicParametrizationHelper : public ParametrizationHelperInterface{
public:
    HarmonicParametrizationHelper(
        pmp::SurfaceMesh& mesh,
        pmp::Vertex& start_vertex
    );

    void parameterize_UV_mesh(bool use_uniform_weights = false) override;

private:
    pmp::SurfaceMesh& mesh;
    pmp::Vertex& start_vertex;
    bool use_uniform_weights;

    bool has_boundary();
};
