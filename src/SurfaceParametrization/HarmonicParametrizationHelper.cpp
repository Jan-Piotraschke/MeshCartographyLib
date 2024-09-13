/**
 * @file        HarmonicParametrizationHelper.cpp
 * @brief       Create a parametrization of the mesh with a square border
 *
 * @author      Jan-Piotraschke
 * @date        2023-Sep-22
 * @license     Apache License 2.0
 *
 * @bug         -
 * @todo        -
 */

#include "HarmonicParametrizationHelper.h"
#include "MonotileBorder/SquareBorderHelper.h"
// #include "MonotileBorder/HexagonBorderHelper.h"

HarmonicParametrizationHelper::HarmonicParametrizationHelper(
    UV::Mesh mesh, UV::halfedge_descriptor bhd, _3D::UV_pmap uvmap)
    : mesh(mesh), bhd(bhd), uvmap(uvmap) {};

// ========================================
// Public Functions
// ========================================

/**
 * @brief Perform UV parameterization
 *
 * Computes a one-to-one mapping from a 3D triangle surface mesh to a simple 2D domain.
 * The mapping is piecewise linear on the triangle mesh. The result is a pair (U,V) of parameter coordinates for each
 * vertex of the input mesh.
 */
SMP::Error_code HarmonicParametrizationHelper::parameterize_UV_mesh()
{
    // Use the SquareBorderHelper to set up the square boundary constraints
    SquareBorderHelper square_border(mesh, bhd, uvmap);

    // Get the border parameterizer from the helper class
    auto border_parameterizer = square_border.get_square_border_parameterizer();

    // Minimize Angle Distortion: Discrete Conformal Map Parameterization
    // from https://doi.org/10.1145/218380.218440
    using Parameterizer = SMP::Discrete_conformal_map_parameterizer_3<UV::Mesh, decltype(border_parameterizer)>;

    // Perform the parameterization using CGAL's built-in function
    return SMP::parameterize(mesh, Parameterizer(), bhd, uvmap);
}
