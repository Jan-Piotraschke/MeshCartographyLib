---
output_filename: "FaceDistortionHelper"

brief: "Calculate the face distortion of the mesh"
details: "Since the area of a mapped patch x(U), U ⊂ parameter space Ω, is computed as ∫ U √det(I)dA, the
            parameterization is area-preserving if det I = 1, or equivalently σ1σ2 = 1, for all points u ∈ Ω"
---

# Introduction

In mesh processing, **face distortion** measures how much the areas of mesh faces (typically triangles) change between different representations, such as from 3D space to a 2D UV mapping.

This document explains the implementation of the `FaceDistortionHelper` class, which computes the average face area distortion between a 3D mesh and its UV mapping.

```cpp
#include "FaceDistortionHelper.h"

FaceDistortionHelper::FaceDistortionHelper(pmp::SurfaceMesh& mesh_open, pmp::SurfaceMesh& mesh_UV)
    : mesh_open(mesh_open), mesh_UV(mesh_UV)
{
}
```

## Mathematical Background

Understanding the mathematical basis is key for comprehending how face distortion is calculated.

Given a triangle with vertices $ A $, $ B $, and $ C $, the area of the triangle is calculated using the cross product of two of its sides:

$$
\text{Area} = \frac{1}{2} \| (\mathbf{B} - \mathbf{A}) \times (\mathbf{C} - \mathbf{A}) \|
$$

**Where:**

- $ \mathbf{A}, \mathbf{B}, \mathbf{C} $ are the position vectors of the triangle's vertices.
- $ \times $ denotes the cross product of two vectors.
- $ \| \cdot \| $ denotes the Euclidean norm (magnitude) of a vector.

### Computing Total Face Distortion

The total face distortion between the 3D mesh and its UV mapping is calculated by:

$$
\text{TotalDistortion} = \sum_{f \in F} \left| \text{Area}^{\text{3D}}(f) - \text{Area}^{\text{UV}}(f) \right|
$$

**Where:**

- $ F $ is the set of all faces in the mesh.
- $ \text{Area}^{\text{3D}}(f) $ is the area of face $ f $ in the original 3D mesh.
- $ \text{Area}^{\text{UV}}(f) $ is the area of face $ f $ in the UV-mapped mesh.
- $ | \cdot | $ denotes the absolute value.

### Computing Average Face Distortion

The average face distortion is then calculated by:

$$
\text{AverageDistortion} = \frac{\text{TotalDistortion}}{|F|}
$$

**Where:**

- $ |F| $ is the total number of faces in the mesh.

## Compute Face Distortion

```cpp
double FaceDistortionHelper::computeFaceDistortion()
{
    double totalDistortion = 0.0;

    for (auto f : mesh_open.faces())
    {
        double areaOpen = triangle_area(mesh_open, f);
        double areaUV = triangle_area(mesh_UV, f);
        totalDistortion += std::fabs(areaOpen - areaUV);
    }

    return totalDistortion / mesh_open.n_faces(); // Average face distortion
}
```

### Helper Function: Compute Mesh Triangle Area

```cpp
double FaceDistortionHelper::triangle_area(const pmp::SurfaceMesh& mesh, const pmp::Face& f)
{
    std::vector<pmp::Point> pts;
    for (auto v : mesh.vertices(f))
    {
        pts.push_back(mesh.position(v));
    }

    // Compute the vectors representing two sides of the triangle
    pmp::Point v1 = pts[1] - pts[0];
    pmp::Point v2 = pts[2] - pts[0];

    // Compute the cross product of the two vectors
    pmp::Point crossProduct;
    crossProduct[0] = v1[1] * v2[2] - v1[2] * v2[1];
    crossProduct[1] = v1[2] * v2[0] - v1[0] * v2[2];
    crossProduct[2] = v1[0] * v2[1] - v1[1] * v2[0];

    return 0.5 * norm(crossProduct); // The magnitude of this cross product is twice the area of the triangle
}
```
