#pragma once

#include "MeshDefinition.h"

class LengthDistortionHelper{
public:
    LengthDistortionHelper(
        pmp::SurfaceMesh& mesh_open,
        pmp::SurfaceMesh& mesh_UV
    );

    double computeLengthDistortion();

private:
    pmp::SurfaceMesh& mesh_open;
    pmp::SurfaceMesh& mesh_UV;

    double edge_length(const pmp::SurfaceMesh& mesh, const pmp::Edge& e);
};
