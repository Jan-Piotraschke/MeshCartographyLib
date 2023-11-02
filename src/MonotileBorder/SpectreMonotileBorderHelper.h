#pragma once

#include "MeshDefinition.h"
#include "SpectreMonotileHelper.h"
#include <Eigen/Dense>

class SpectreMonotileBorderHelper {
public:
    struct Corner {
        Eigen::Vector2d position;

        // Define a constructor that initializes both the position and sideLength
        Corner(const Eigen::Vector2d& pos) : position(pos) {}
    };

    SpectreMonotileBorderHelper(
        pmp::SurfaceMesh& mesh,
        pmp::Vertex& start_vertex
    );

    void setup_spectre_monotile_boundary_constraints(double a, double b, double curve_strength);

    std::vector<Corner> getCorners() const {
        return corners;
    }

private:
    pmp::SurfaceMesh& mesh;
    pmp::Vertex& start_vertex;
    std::vector<Corner> corners;
    double sideLength;

    void initializeCorners(double a, double b, double curve_strength);
    pmp::TexCoord mapToSpectreMonotile(double length);
    static constexpr double TOLERANCE = 1e-5;
};
