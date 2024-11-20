/**
 * @file        SquareBorderHelper.cpp
 * @brief       Create a square border
 *
 * @author      Jan-Piotraschke
 * @date        2023-Sep-27
 * @license     Apache License 2.0
 *
 * @bug         -
 * @todo        -
 */

#include "SquareBorderHelper.h"

// ========================================
// Constructor
// ========================================

SquareBorderHelper::SquareBorderHelper(pmp::SurfaceMesh& mesh, pmp::Vertex& start_vertex)
    : mesh(mesh), start_vertex(start_vertex) {};

// ========================================
// Public Functions
// ========================================

void SquareBorderHelper::setup_square_boundary_constraints()
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

    int corner_count = 4;
    double sideLength = length / corner_count;
    initializeCorners(sideLength);

    double step_size = length / N;
    auto tolerance = 1e-4;

    // map length intervals to square intervals
    for (auto [vertice_id, l] = std::pair<unsigned int, double>{0, 0.0}; vertice_id < N; ++vertice_id, l += step_size)
    {
        pmp::TexCoord t = mapToSquare(l);

        // Apply tolerance
        if (t[0] < tolerance)
            t[0] = 0.0;
        if (t[1] < tolerance)
            t[1] = 0.0;

        tex[loop[vertice_id]] = t;
    }
}

// ========================================
// Private Functions
// ========================================

void SquareBorderHelper::initializeCorners(double sideLength)
{
    corners.clear();
    corners.push_back(Corner({0.0, 0.0}, sideLength));
    corners.push_back(Corner({1.0, 0.0}, sideLength));
    corners.push_back(Corner({1.0, 1.0}, sideLength));
    corners.push_back(Corner({0.0, 1.0}, sideLength));
}

pmp::TexCoord SquareBorderHelper::mapToSquare(double l)
{
    double sum = 0.0;
    for (size_t i = 0; i < corners.size(); ++i)
    {
        const auto& corner = corners[i];
        const auto& nextCorner = corners[(i + 1) % corners.size()];
        if (l <= sum + corner.sideLength)
        {
            double frac = (l - sum) / corner.sideLength;
            return corner.position * (1.0 - frac) + nextCorner.position * frac;
        }
        sum += corner.sideLength;
    }
    return pmp::TexCoord(0.0, 0.0);
}
