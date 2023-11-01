#pragma once

#include "MeshDefinition.h"
#include "SpectreMonotileHelper.h"

class SpectreMonotileBorderHelper {
public:
    SpectreMonotileBorderHelper(
        pmp::SurfaceMesh& mesh,
        pmp::Vertex& start_vertex
    );

    void setup_spectre_monotile_boundary_constraints(double a, double b, double curve_strength);

private:
    pmp::SurfaceMesh& mesh;
    pmp::Vertex& start_vertex;

    void mapToSpectreMonotile(const std::vector<double>& x_vals, const std::vector<double>& y_vals);
};
