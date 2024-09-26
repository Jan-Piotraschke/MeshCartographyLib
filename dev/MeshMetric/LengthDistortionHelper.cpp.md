---
output_filename: "LengthDistortionHelper"

brief: "Calculate the length distortion of the mesh"
details: "A parameterization is length-preserving if it is both angle- and area-preserving.
            In this case the first fundamental form is the identity, i.e., σ1 = σ2 = 1.
            Only developable surfaces, where these surfaces have zero Gaussian curvature everywhere, admit a perfect length-preserving parameterization."
---

# Introduction

In mesh processing, **length distortion** measures how much the edge lengths of a mesh change between different representations, such as from 3D space to a 2D UV mapping.

This document explains the implementation of the `LengthDistortionHelper` class, which computes the average length distortion between a 3D mesh and its UV mapping.

```cpp
#include "LengthDistortionHelper.h"

LengthDistortionHelper::LengthDistortionHelper(_3D::Mesh& mesh_open, UV::Mesh& mesh_UV)
    : mesh_open(mesh_open), mesh_UV(mesh_UV)
{
}
```

## Mathematical Background

Understanding the mathematical basis is crucial for grasping how length distortion is calculated.

Given an edge with vertices $ A $ and $ B $, the length of the edge is calculated using:

$$
L = \| \mathbf{A} - \mathbf{B} \|
$$

**Where:**

- $ \mathbf{A}, \mathbf{B} $ are the position vectors of the edge's vertices.
- $ \| \cdot \| $ denotes the Euclidean norm (magnitude) of a vector.

### Computing Total Length Distortion

The total length distortion between the 3D mesh and its UV mapping is calculated by:

$$
\text{TotalDistortion} = \sum_{e \in E} \left| L^{\text{3D}}(e) - L^{\text{UV}}(e) \right|
$$

**Where:**

- $ E $ is the set of all edges in the mesh.
- $ L^{\text{3D}}(e) $ is the length of edge $ e $ in the original 3D mesh.
- $ L^{\text{UV}}(e) $ is the length of edge $ e $ in the UV-mapped mesh.
- $ | \cdot | $ denotes the absolute value.

### Computing Average Length Distortion

The average length distortion is then calculated by:

$$
\text{AverageDistortion} = \frac{\text{TotalDistortion}}{|E|}
$$

**Where:**

- $ |E| $ is the total number of edges in the mesh.

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
