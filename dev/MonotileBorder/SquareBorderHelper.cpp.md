---
output_filename: "SquareBorderHelper"

brief: "Create a square border"
---

# Square Border Helper

The purpose of this module is to provide functions for creating a square border for an UV mesh model.

```cpp
#include "SquareBorderHelper.h"

SquareBorderHelper::SquareBorderHelper(UV::Mesh& mesh, UV::halfedge_descriptor bhd, _3D::UV_pmap& uvmap)
    : mesh(mesh), bhd(bhd), uvmap(uvmap)
{
}

CGAL::Surface_mesh_parameterization::Square_border_uniform_parameterizer_3<UV::Mesh>
SquareBorderHelper::get_square_border_parameterizer()
{
    using Border_parameterizer = CGAL::Surface_mesh_parameterization::Square_border_uniform_parameterizer_3<UV::Mesh>;

    // Create and return an instance of the CGAL square border parameterizer
    Border_parameterizer border_parameterizer;
    return border_parameterizer;
}
```
