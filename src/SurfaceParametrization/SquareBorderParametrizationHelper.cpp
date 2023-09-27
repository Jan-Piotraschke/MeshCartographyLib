/**
 * @file        SquareBorderParametrizationHelper.cpp
 * @brief       Create a parametrization of the mesh with a square border
 *
 * @author      Jan-Piotraschke
 * @date        2023-Sep-22
 * @license     Apache License 2.0
 *
 * @bug         -
 * @todo        -
 */

#include "SquareBorderParametrizationHelper.h"

SquareBorderParametrizationHelper::SquareBorderParametrizationHelper(
        Triangle_mesh& mesh,
        halfedge_descriptor& bhd,
        UV_pmap& uvmap
) :
        mesh(mesh),
        bhd(bhd),
        uvmap(uvmap)
{};

// ========================================
// Public Functions
// ========================================

/**
 * @brief Perform UV parameterization
*
* Computes a one-to-one mapping from a 3D triangle surface mesh to a simple 2D domain.
* The mapping is piecewise linear on the triangle mesh. The result is a pair (U,V) of parameter coordinates for each vertex of the input mesh.
*/
SMP::Error_code SquareBorderParametrizationHelper::parameterize_UV_mesh(){
    // Choose the border type of the uv parametrisation
    using Border_parameterizer = SMP::Square_border_uniform_parameterizer_3<Triangle_mesh>;
    Border_parameterizer border_parameterizer;

    // Minimize Angle Distortion: Discrete Conformal Map Parameterization
    // from https://doi.org/10.1145/218380.218440
    using Parameterizer = SMP::Discrete_conformal_map_parameterizer_3<Triangle_mesh, Border_parameterizer>;

    return SMP::parameterize(mesh, Parameterizer(), bhd, uvmap);
}
