#pragma once

#include "SurfaceParametrizationHelperInterface.h"

class SquareBorderParametrizationHelper : public ParametrizationHelperInterface {
public:
    SquareBorderParametrizationHelper(
        Triangle_mesh& mesh,
        halfedge_descriptor& bhd,
        UV_pmap& uvmap
    );

    SMP::Error_code parameterize_UV_mesh() override;

private:
    Triangle_mesh& mesh;
    halfedge_descriptor& bhd;
    UV_pmap& uvmap;
};
