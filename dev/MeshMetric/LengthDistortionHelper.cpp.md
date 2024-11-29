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

LengthDistortionHelper::LengthDistortionHelper(pmp::SurfaceMesh& mesh_open, pmp::SurfaceMesh& mesh_UV)
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

    for (auto e : mesh_open.edges())
    {
        double lengthOpen = edge_length(mesh_open, e);
        double lengthUV = edge_length(mesh_UV, e);

        totalDistortion += std::fabs(lengthOpen - lengthUV);
    }

    return totalDistortion / mesh_open.n_edges(); // Average length distortion
}
```

### Helper Function: Compute Mesh Edge Length

```cpp
double LengthDistortionHelper::edge_length(const pmp::SurfaceMesh& mesh, const pmp::Edge& e)
{
    pmp::Vertex v1 = mesh.vertex(e, 0);
    pmp::Vertex v2 = mesh.vertex(e, 1);

    pmp::Point p1 = mesh.position(v1);
    pmp::Point p2 = mesh.position(v2);

    return norm(p1 - p2); // norm computes the magnitude or length
}
```
