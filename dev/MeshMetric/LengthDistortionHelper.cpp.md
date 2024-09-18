---
output_filename: "LengthDistortionHelper"

brief: "Calculate the length distortion of the mesh"
details: "A parameterization is length-preserving if it is both angle- and area-preserving.
            In this case the first fundamental form is the identity, i.e., σ1 = σ2 = 1.
            Only developable surfaces, where these surfaces have zero Gaussian curvature everywhere, admit a perfect length-preserving parameterization."
---

```cpp
#include "LengthDistortionHelper.h"

LengthDistortionHelper::LengthDistortionHelper(_3D::Mesh& mesh_open, UV::Mesh& mesh_UV)
    : mesh_open(mesh_open), mesh_UV(mesh_UV)
{
}
```

## Compute Length Distortion

```cpp
double LengthDistortionHelper::computeLengthDistortion()
{
    double totalDistortion = 0.0;

    for (const auto& e : mesh_open.edges())
    {
        // Calculate lengths for both the 3D mesh and UV mesh
        double lengthOpen = edge_length(mesh_open, e);

        // Extract the underlying mesh from the UV seam mesh and calculate the edge length
        const auto& underlying_mesh = mesh_UV.mesh();
        double lengthUV = edge_length(underlying_mesh, e);

        totalDistortion += std::fabs(lengthOpen - lengthUV);
    }

    return totalDistortion / mesh_open.number_of_edges(); // Average length distortion
}
```

### Helper Function: Compute Mesh Edge Length

```cpp
template <typename MeshType>
double LengthDistortionHelper::edge_length(const MeshType& mesh, const typename MeshType::Edge_index& e)
{
    // Get the vertices of the edge
    auto halfedge = mesh.halfedge(e);
    auto v1 = mesh.point(mesh.source(halfedge));
    auto v2 = mesh.point(mesh.target(halfedge));

    // Compute the vector representing the edge
    Kernel::Vector_3 edgeVector = v2 - v1;

    // Return the length of the edge
    return std::sqrt(edgeVector.squared_length());
}
```
