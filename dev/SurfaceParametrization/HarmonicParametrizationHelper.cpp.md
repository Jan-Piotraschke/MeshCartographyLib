---
output_filename: "HarmonicParametrizationHelper"

brief: "Create a parametrization of the mesh with a square border"
---

# Harmonic Parametrization Helper

The purpose of this module is to provide functions for creating a parametrization of a closed 3D mesh.

Computes a one-to-one mapping from a 3D triangle surface mesh to a simple 2D domain.
The mapping is piecewise linear on the triangle mesh. The result is a pair (U,V) of parameter coordinates for each
vertex of the input mesh. We use discrete conformal mapping to **minimize angle distortion** based on the following paper:
https://doi.org/10.1145/218380.218440

```cpp
#include "HarmonicParametrizationHelper.h"
#include "MonotileBorder/HexagonBorderHelper.h"
#include "MonotileBorder/SquareBorderHelper.h"

HarmonicParametrizationHelper::HarmonicParametrizationHelper(
    UV::Mesh mesh, UV::halfedge_descriptor bhd, _3D::UV_pmap uvmap)
    : mesh(mesh), bhd(bhd), uvmap(uvmap) {};


SMP::Error_code HarmonicParametrizationHelper::parameterize_UV_mesh()
{
    HexagonBorderHelper hexagon_border(mesh, bhd, uvmap);
    hexagon_border.setup_hexagon_boundary_constraints();
    hexagon_border.drawHexagon("hexagon_border.png");

    SquareBorderHelper square_border(mesh, bhd, uvmap);
    auto border_parameterizer = square_border.get_square_border_parameterizer();

    // Minimize Angle Distortion: Discrete Conformal Map Parameterization
    using Parameterizer = SMP::Discrete_conformal_map_parameterizer_3<UV::Mesh, decltype(border_parameterizer)>;

    // Perform the parameterization using CGAL's built-in function
    return SMP::parameterize(mesh, Parameterizer(), bhd, uvmap);
}
