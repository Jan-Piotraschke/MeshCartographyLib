#pragma once

#include "MeshDefinition.h"

class HexagonBorderHelper {
public:
    HexagonBorderHelper(
        pmp::SurfaceMesh& mesh,
        pmp::Vertex& start_vertex
    );

    void setup_hexagon_boundary_constraints();

    struct Corner {
        Eigen::Vector2d position;
        double sideLength;

        // Define a constructor that initializes both the position and sideLength
        Corner(const Eigen::Vector2d& pos, double length) : position(pos), sideLength(length) {}
    };

    std::vector<Corner> getCorners() {
        return corners;
    }

private:
    pmp::SurfaceMesh& mesh;
    pmp::Vertex& start_vertex;

    std::vector<Corner> corners;

    void initializeCorners(double sideLength);
    pmp::TexCoord mapToHexagon(double l);
};
