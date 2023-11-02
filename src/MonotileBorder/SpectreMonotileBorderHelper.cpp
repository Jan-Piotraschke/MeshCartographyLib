/**
 * @file        SpectreMonotileHelper.cpp
 * @brief       Create the border of the spectre monotile
 *
 * @author      Jan-Piotraschke
 * @date        2023-Oct-26
 * @license     Apache License 2.0
 *
 * @bug         -
 * @todo        -
 */

#include "SpectreMonotileBorderHelper.h"

SpectreMonotileBorderHelper::SpectreMonotileBorderHelper(
    pmp::SurfaceMesh& mesh,
    pmp::Vertex& start_vertex
)
    : mesh(mesh),
    start_vertex(start_vertex)
{};

void SpectreMonotileBorderHelper::setup_spectre_monotile_boundary_constraints(double a, double b, double curve_strength)
{
    // get properties
    auto points = mesh.vertex_property<pmp::Point>("v:point");
    auto tex = mesh.vertex_property<pmp::TexCoord>("v:tex");

    // Initialize all texture coordinates to the origin.
    for (auto v : mesh.vertices()) {
        tex[v] = pmp::TexCoord(0.0, 0.0);
    }

    // map to spectre monotile
    pmp::SurfaceMesh::VertexIterator vit, vend = mesh.vertices_end();
    pmp::Halfedge hh;
    std::vector<pmp::Vertex> loop;

    // collect boundary loop
    hh = mesh.halfedge(start_vertex);
    do {
        loop.push_back(mesh.from_vertex(hh));
        hh = mesh.next_halfedge(hh);
    } while (hh != mesh.halfedge(start_vertex));

    // Calculate spectre monotile border
    std::vector<double> x_vals, y_vals;

    // multiply by 14 because spectre has 14 edges
    size_t desired_num_points = loop.size() * 14;

    // reserve space for the vectors
    spectre_border(a, b, curve_strength, x_vals, y_vals, desired_num_points);

    x_vals.pop_back();
    y_vals.pop_back();

    for (int i = 0; i < x_vals.size(); ++i) {
        if (i % loop.size() == 0) {
            corners.push_back(Eigen::Vector2d(x_vals[i], y_vals[i]));
        }
    }

    // Assign each 14th point to the texture coordinate
    std::vector<Eigen::Vector2d> corners_placeholder;
    for (size_t i = 0; i < x_vals.size(); i += 14) {
        corners_placeholder.push_back(Eigen::Vector2d(x_vals[i], y_vals[i]));
    }

    // Assign each 14th point to the texture coordinate
    pmp::TexCoord t;
    for (size_t i = 0; i < loop.size(); ++i) {
        t = pmp::TexCoord(corners_placeholder[i][0], corners_placeholder[i][1]);
        tex[loop[i]] = t;
    }
}
