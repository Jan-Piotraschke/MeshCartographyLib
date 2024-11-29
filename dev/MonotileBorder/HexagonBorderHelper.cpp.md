---
output_filename: "HexagonBorderHelper"

brief: "Create a hexagonal border"
---

# Hexagon Border Helper

The purpose of this module is to provide functions for creating a hexagonal border for an UV mesh model.

```cpp
#include "HexagonBorderHelper.h"

// ========================================
// Constructor
// ========================================

HexagonBorderHelper::HexagonBorderHelper(pmp::SurfaceMesh& mesh, pmp::Vertex& start_vertex)
    : mesh(mesh), start_vertex(start_vertex) {};

// ========================================
// Public Functions
// ========================================

void HexagonBorderHelper::setup_hexagon_boundary_constraints()
{
    // get properties
    auto points = mesh.vertex_property<pmp::Point>("v:point");
    auto tex = mesh.vertex_property<pmp::TexCoord>("v:tex");

    pmp::SurfaceMesh::VertexIterator vit, vend = mesh.vertices_end();
    pmp::Halfedge hh;
    std::vector<pmp::Vertex> loop;

    // Initialize all texture coordinates to the origin.
    for (auto v : mesh.vertices())
    {
        tex[v] = pmp::TexCoord(0.0, 0.0); // Initialize to the bottom-left corner
    }

    // collect boundary loop
    hh = mesh.halfedge(start_vertex);
    do
    {
        loop.push_back(mesh.from_vertex(hh));
        hh = mesh.next_halfedge(hh);
    } while (hh != mesh.halfedge(start_vertex));

    unsigned int vertice_id, N = loop.size();
    double l, length;
    pmp::TexCoord t;

    // compute length of boundary loop
    for (vertice_id = 0, length = 0.0; vertice_id < N; ++vertice_id)
    {
        length += pmp::distance(points[loop[vertice_id]], points[loop[(vertice_id + 1) % N]]);
    }

    // Initialize the hexagon corners
    double sideLength = length / 6.0;
    initializeCorners(sideLength);

    double step_size = length / N;
    auto tolerance = 1e-4;

    // map length intervals to hexagon intervals
    for (auto [vertice_id, l] = std::pair<unsigned int, double>{0, 0.0}; vertice_id < N; ++vertice_id, l += step_size)
    {
        pmp::TexCoord t = mapToHexagon(l);

        // Apply tolerance
        if (std::abs(t[0]) < tolerance)
            t[0] = 0.0;
        if (std::abs(t[1]) < tolerance)
            t[1] = 0.0;

        tex[loop[vertice_id]] = t;
    }
}
```

## Initialize Hexagon Corners and Map to Hexagon

```cpp
void HexagonBorderHelper::initializeCorners(double sideLength)
{
    corners.clear();
    double angle = M_PI / 3; // 60 degrees

    // Initialize the first corner to be at (0, 0)
    corners.push_back(Corner(Eigen::Vector2d(0.0, 0.0), sideLength));

    // Calculate positions for the other corners
    for (int i = 1; i < 6; ++i)
    {
        double x = corners[i - 1].position(0) + sideLength * std::cos(i * angle);
        double y = corners[i - 1].position(1) + sideLength * std::sin(i * angle);
        corners.push_back(Corner(Eigen::Vector2d(x, y), sideLength));
    }
}

pmp::TexCoord HexagonBorderHelper::mapToHexagon(double l)
{
    double sum = 0.0;
    for (size_t i = 0; i < corners.size(); ++i)
    {
        const auto& corner = corners[i];
        const auto& nextCorner = corners[(i + 1) % corners.size()];
        if (l <= sum + corner.sideLength)
        {
            double frac = (l - sum) / corner.sideLength;
            return pmp::TexCoord((corner.position * (1.0 - frac) + nextCorner.position * frac).cast<float>());
        }
        sum += corner.sideLength;
    }
    return pmp::TexCoord(0.0, 0.0);
}
```
