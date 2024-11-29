#pragma once

#include "MeshDefinition/pmp.h"

class SquareBorderHelper
{
  public:
    SquareBorderHelper(pmp::SurfaceMesh& mesh, pmp::Vertex& start_vertex);

    void setup_square_boundary_constraints();

    struct Corner
    {
        Eigen::Vector2d position;
        double sideLength;

        // Define a constructor that initializes both the position and sideLength
        Corner(const Eigen::Vector2d& pos, double length) : position(pos), sideLength(length) {}
    };

    std::vector<Corner> getCorners() { return corners; }

  private:
    pmp::SurfaceMesh& mesh;
    pmp::Vertex& start_vertex;

    std::vector<Corner> corners;

    void initializeCorners(double sideLength);
    pmp::TexCoord mapToSquare(double l);
};
