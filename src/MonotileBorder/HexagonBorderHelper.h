#pragma once

#include "MeshDefinition.h"
#include <vector>
#include <Eigen/Core>  // For vector math, if necessary for corners

class HexagonBorderHelper {
public:
    // Constructor to initialize the mesh, starting vertex, and UV map
    HexagonBorderHelper(UV::Mesh& mesh, UV::halfedge_descriptor bhd, _3D::UV_pmap& uvmap);

    // Function to set up the hexagonal boundary constraints
    void setup_hexagon_boundary_constraints();

    // Function to draw the hexagon and save it as an image
    void drawHexagon(const std::string& filename);

private:
    // Helper functions for hexagon boundary setup
    void initializeCorners(double sideLength);
    Point_2 mapToHexagon(double l);

    // Mesh, boundary halfedge descriptor, and UV map
    UV::Mesh& mesh;
    UV::halfedge_descriptor bhd;
    _3D::UV_pmap& uvmap;

    // Stores the corners of the hexagon
    struct Corner {
        Eigen::Vector2d position;
        double sideLength;
        Corner(Eigen::Vector2d pos, double sideLen) : position(pos), sideLength(sideLen) {}
    };
    std::vector<Corner> corners;
};
