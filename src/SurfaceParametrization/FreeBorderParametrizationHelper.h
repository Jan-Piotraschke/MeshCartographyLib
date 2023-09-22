#pragma once

#include "SurfaceParametrizationHelperInterface.h"

class FreeBorderParametrizationHelper : public ParametrizationHelperInterface {
public:
    FreeBorderParametrizationHelper(
        UV::Mesh mesh,
        UV::halfedge_descriptor bhd,
        _3D::UV_pmap uvmap
    );

    SMP::Error_code parameterize_UV_mesh() override;

private:
    UV::Mesh mesh;
    UV::halfedge_descriptor bhd;
    _3D::UV_pmap uvmap;
};
