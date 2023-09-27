/**
 * @file        SquareBorderHelper.cpp
 * @brief       Create a square border
 *
 * @author      Jan-Piotraschke
 * @date        2023-Sep-27
 * @license     Apache License 2.0
 *
 * @bug         -
 * @todo        improve the accuracy of the square border
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

    unsigned int i, n = loop.size();
    pmp::Scalar l, length;
    pmp::TexCoord t;

    // compute length of boundary loop
    for (i = 0, length = 0.0; i < n; ++i) {
        length += distance(points[loop[i]], points[loop[(i + 1) % n]]);
    }

    // Define lengths of the square sides
    pmp::Scalar sideLength = length / 4.0;

    // map length intervals to square intervals
    for (i = 0, l = 0.0; i < n;) {
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

        tex[loop[i]] = t;

        ++i;
        if (i < n) {
            l += distance(points[loop[i]], points[loop[(i + 1) % n]]);
        }
    }

    // // print out the texture coordinates
    // for (auto v : mesh.vertices()) {
    //     std::cout << tex[v] << std::endl;
    // }
}
