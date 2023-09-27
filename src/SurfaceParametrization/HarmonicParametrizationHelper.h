#pragma once

#include "SurfaceParametrizationHelperInterface.h"

class HarmonicParametrizationHelper : public ParametrizationHelperInterface{
public:
    HarmonicParametrizationHelper(
        pmp::SurfaceMesh& mesh
    );

    void parameterize_UV_mesh(bool use_uniform_weights = false) override;

private:
    pmp::SurfaceMesh& mesh;
    bool use_uniform_weights;

    bool has_boundary();
};
