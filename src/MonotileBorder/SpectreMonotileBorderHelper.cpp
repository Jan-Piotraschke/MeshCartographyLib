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

    // Calculate spectre monotile border
    std::vector<double> x_vals, y_vals;
    spectre_border(a, b, curve_strength, x_vals, y_vals);

    // Map the calculated border to texture coordinates
    mapToSpectreMonotile(x_vals, y_vals);
}

void SpectreMonotileBorderHelper::mapToSpectreMonotile(const std::vector<double>& x_vals, const std::vector<double>& y_vals)
{
    auto tex = mesh.vertex_property<pmp::TexCoord>("v:tex");

    pmp::Halfedge hh = mesh.halfedge(start_vertex);
    size_t index = 0;

    do {
        if (index < x_vals.size()) {
            pmp::Vertex v = mesh.from_vertex(hh);
            tex[v] = pmp::TexCoord(x_vals[index], y_vals[index]);
            index++;
        }
        hh = mesh.next_halfedge(hh);
    } while (hh != mesh.halfedge(start_vertex) && index < x_vals.size());
}
