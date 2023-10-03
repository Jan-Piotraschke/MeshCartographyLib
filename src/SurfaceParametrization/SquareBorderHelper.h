#pragma once

#include "MeshDefinition.h"

class SquareBorderHelper {
public:
    SquareBorderHelper(
        pmp::SurfaceMesh& mesh
    );

    void setup_square_boundary_constraints();

private:
    pmp::SurfaceMesh& mesh;
};
