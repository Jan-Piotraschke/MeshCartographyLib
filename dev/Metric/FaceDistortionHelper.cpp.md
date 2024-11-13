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

FaceDistortionHelper::FaceDistortionHelper(_3D::Mesh& mesh_open, UV::Mesh& mesh_UV)
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

    for (const auto& f : mesh_open.faces())
    {
        // Calculate areas for both the 3D mesh and UV mesh
        double areaOpen = triangle_area(mesh_open, f);

        // Extract the underlying mesh from the UV seam mesh and calculate the area
        const auto& underlying_mesh = mesh_UV.mesh();
        double areaUV = triangle_area(underlying_mesh, f);

        totalDistortion += std::fabs(areaOpen - areaUV);
    }

    return totalDistortion / mesh_open.number_of_faces(); // Average face distortion
}
```

### Helper Function: Compute Mesh Triangle Area

```cpp
template <typename MeshType>
double FaceDistortionHelper::triangle_area(const MeshType& mesh, const typename MeshType::Face_index& f)
{
    std::vector<Point_3> pts;

    // Collect the vertices of the face
    for (const auto& halfedge : CGAL::halfedges_around_face(mesh.halfedge(f), mesh))
    {
        pts.push_back(mesh.point(CGAL::target(halfedge, mesh)));
    }

    // Compute vectors representing two sides of the triangle
    Kernel::Vector_3 v1 = pts[1] - pts[0];
    Kernel::Vector_3 v2 = pts[2] - pts[0];

    // Compute the cross product of the two vectors
    Kernel::Vector_3 crossProduct = CGAL::cross_product(v1, v2);

    // The magnitude of the cross product is twice the area of the triangle
    return 0.5 * std::sqrt(crossProduct.squared_length());
}
```
