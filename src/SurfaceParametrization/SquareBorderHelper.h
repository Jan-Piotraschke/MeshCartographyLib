#pragma once

#include "MeshDefinition.h"

class SquareBorderHelper {
public:
    SquareBorderHelper(
        pmp::SurfaceMesh& mesh,
        pmp::Vertex& start_vertex
    );

    void setup_square_boundary_constraints();

private:
    pmp::SurfaceMesh& mesh;
    pmp::Vertex& start_vertex;
};
