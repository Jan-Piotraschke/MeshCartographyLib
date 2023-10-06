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
    pmp::SurfaceMesh& mesh
)
    : mesh(mesh)
{};

void SquareBorderHelper::setup_square_boundary_constraints()
{
    // get properties
    auto points = mesh.vertex_property<pmp::Point>("v:point");
    auto tex = mesh.vertex_property<pmp::TexCoord>("v:tex");

    pmp::SurfaceMesh::VertexIterator vit, vend = mesh.vertices_end();
    pmp::Vertex vh;
    pmp::Halfedge hh;
    std::vector<pmp::Vertex> loop;

    // Initialize all texture coordinates to the origin.
    for (auto v : mesh.vertices()) {
        tex[v] = pmp::TexCoord(0.0, 0.0); // Initialize to the bottom-left corner
    }

    // find 1st boundary vertex
    for (vit = mesh.vertices_begin(); vit != vend; ++vit) {
        if (mesh.is_boundary(*vit)) {
            break;
        }
    }

    // collect boundary loop
    vh = *vit;
    hh = mesh.halfedge(vh);
    do {
        loop.push_back(mesh.to_vertex(hh));
        hh = mesh.next_halfedge(hh);
    } while (hh != mesh.halfedge(vh));

    unsigned int vertice_id, N = loop.size();
    pmp::Scalar l, length;
    pmp::TexCoord t;

    // compute length of boundary loop
    for (vertice_id = 0, length = 0.0; vertice_id < N; ++vertice_id) {
        length += distance(points[loop[vertice_id]], points[loop[(vertice_id + 1) % N]]);
    }
    int corner_count = 4;

    // Define lengths of the square sides
    pmp::Scalar sideLength = length / corner_count;
    pmp::Scalar step_size = length / N;

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
