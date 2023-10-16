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

SquareBorderHelper::SquareBorderHelper(
    pmp::SurfaceMesh& mesh,
    pmp::Vertex& start_vertex
)
    : mesh(mesh),
    start_vertex(start_vertex)
{};

void SquareBorderHelper::setup_square_boundary_constraints()
{
    // get properties
    auto points = mesh.vertex_property<pmp::Point>("v:point");
    auto tex = mesh.vertex_property<pmp::TexCoord>("v:tex");

    pmp::SurfaceMesh::VertexIterator vit, vend = mesh.vertices_end();
    pmp::Halfedge hh;
    std::vector<pmp::Vertex> loop;

    // Initialize all texture coordinates to the origin.
    for (auto v : mesh.vertices()) {
        tex[v] = pmp::TexCoord(0.0, 0.0); // Initialize to the bottom-left corner
    }

    std::cout << "start_vertex:: " << start_vertex << std::endl;
    // collect boundary loop
    hh = mesh.halfedge(start_vertex);
    do {
        loop.push_back(mesh.from_vertex(hh));
        hh = mesh.next_halfedge(hh);
    } while (hh != mesh.halfedge(start_vertex));

    unsigned int vertice_id, N = loop.size();
    double l, length;
    pmp::TexCoord t;

    // compute length of boundary loop
    for (vertice_id = 0, length = 0.0; vertice_id < N; ++vertice_id) {
        length += pmp::distance(points[loop[vertice_id]], points[loop[(vertice_id + 1) % N]]);
    }
    int corner_count = 4;

    // Define lengths of the square sides
    double sideLength = length / corner_count;
    double step_size = length / N;

    auto tolerance = 1e-4;

    // map length intervals to square intervals
    for (vertice_id = 0, l = 0.0; vertice_id < N;) {
        if (l <= sideLength) { // bottom side
            t[0] = l / sideLength;
            t[1] = 0.0;
        } else if (l <= 2 * sideLength) { // right side
            t[0] = 1.0;
            t[1] = (l - sideLength) / sideLength;
        } else if (l <= 3 * sideLength) { // top side
            t[0] = 1.0 - (l - 2 * sideLength) / sideLength;
            t[1] = 1.0;
        } else { // left side
            t[0] = 0.0;
            t[1] = 1.0 - (l - 3 * sideLength) / sideLength;
        }

        if (t[0] < tolerance) {
            t[0] = 0.0;
        }
        if (t[1] < tolerance) {
            t[1] = 0.0;
        }

        tex[loop[vertice_id]] = t;

        ++vertice_id;
        if (vertice_id < N) {
            l += step_size;
        }
    }
}
