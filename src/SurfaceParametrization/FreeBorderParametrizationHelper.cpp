/**
 * @file        FreeBorderParametrizationHelper.cpp
 * @brief       Create a parametrization of the mesh with a free border
 *
 * @author      Jan-Piotraschke
 * @date        2023-Sep-22
 * @license     Apache License 2.0
 *
 * @bug         -
 * @todo        -
 */

#include "FreeBorderParametrizationHelper.h"

FreeBorderParametrizationHelper::FreeBorderParametrizationHelper(
        UV::Mesh mesh,
        UV::halfedge_descriptor bhd,
        _3D::UV_pmap uvmap
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
SMP::Error_code FreeBorderParametrizationHelper::parameterize_UV_mesh(){
    // Choose the border type of the uv parametrisation
    using Border_parameterizer = SMP::Square_border_uniform_parameterizer_3<UV::Mesh>;
    Border_parameterizer border_parameterizer;

    // ARAP parameterization
    using Parameterizer = SMP::ARAP_parameterizer_3<UV::Mesh, Border_parameterizer>;

    // Specify lambda value and other optional parameters
    int lambda = 1000;
    unsigned int iterations = 50;
    double tolerance = 1e-6;
    Parameterizer parameterizer(border_parameterizer, Parameterizer::Solver_traits(), lambda, iterations, tolerance);

    return SMP::parameterize(mesh, parameterizer, bhd, uvmap);
}
