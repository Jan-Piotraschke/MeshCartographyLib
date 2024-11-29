---
output_filename: "AngleDistortionHelper"

brief: "Calculate the angle distortion of the mesh"
details: "The angle is preserved if the first fundamental form is a multiple of the identity, i.e., I(u) = η(u)Id"
---

# Introduction

In mesh processing, **angle distortion** measures how much the angles of mesh faces (typically triangles) change between different representations of the mesh, such as from 3D space to a 2D UV mapping. Preserving angles is crucial for creating maps which should get used in navigation.

This document explains the implementation of the `AngleDistortionHelper` class, which computes the average angle distortion between a 3D mesh and its UV mapping.

```cpp
#include "AngleDistortionHelper.h"

AngleDistortionHelper::AngleDistortionHelper(pmp::SurfaceMesh& mesh_open, pmp::SurfaceMesh& mesh_UV)
    : mesh_open(mesh_open), mesh_UV(mesh_UV)
{
}
```

&NewLine;
## Mathematical Background

Understanding the mathematical basis is essential for grasping how angle distortion is calculated.

Given a triangle with vertices $ A $, $ B $, and $ C $, the angle at vertex $ C $ is calculated using:

$$
\theta_C = \arccos \left( \frac{ (\mathbf{A} - \mathbf{C}) \cdot (\mathbf{B} - \mathbf{C}) }{ \| \mathbf{A} - \mathbf{C} \| \, \| \mathbf{B} - \mathbf{C} \| } \right)
$$

**Where:**

- $ \mathbf{A}, \mathbf{B}, \mathbf{C} $ are the position vectors of the triangle's vertices.
- $ \cdot $ denotes the dot product of two vectors.
- $ \| \cdot \| $ denotes the Euclidean norm (magnitude) of a vector.
- $ \arccos $ is the inverse cosine function, returning the angle in radians.


### Computing Total Angle Distortion

The total angle distortion between the 3D mesh and its UV mapping is calculated by:

$$
\text{TotalDistortion} = \sum_{f \in F} \sum_{i=1}^{3} \left| \theta_i^{\text{3D}}(f) - \theta_i^{\text{UV}}(f) \right|
$$

**Where:**

- $ F $ is the set of all faces in the mesh.
- $ \theta_i^{\text{3D}}(f) $ is the $ i $-th angle of face $ f $ in the original 3D mesh.
- $ \theta_i^{\text{UV}}(f) $ is the $ i $-th angle of face $ f $ in the UV-mapped mesh.
- $ | \cdot | $ denotes the absolute value.

### Computing Average Angle Distortion

The average angle distortion is calculated by:

$$
\text{AverageDistortion} = \frac{\text{TotalDistortion}}{3 \times |F|}
$$

**Where:**

- $ |F| $ is the total number of faces in the mesh.


## Compute Angle Distortion

```cpp
double AngleDistortionHelper::computeAngleDistortion()
{
    double totalDistortion = 0.0;

    for (auto f : mesh_open.faces())
    {
        std::vector<double> anglesOpen = triangle_angles(mesh_open, f);
        std::vector<double> anglesUV = triangle_angles(mesh_UV, f);

        for (int i = 0; i < 3; i++)
        {
            totalDistortion += std::fabs(anglesOpen[i] - anglesUV[i]);
        }
    }

    return totalDistortion / (3 * mesh_open.n_faces()); // Average angle distortion
}
```

### Helper Function: Compute Mesh Triangle Angles

```cpp
std::vector<double> AngleDistortionHelper::triangle_angles(const pmp::SurfaceMesh& mesh, const pmp::Face& f)
{
    std::vector<pmp::Point> pts;
    for (auto v : mesh.vertices(f))
    {
        pts.push_back(mesh.position(v));
    }

    std::vector<double> angles(3);
    angles[0] = computeAngle(pts[0], pts[1], pts[2]);
    angles[1] = computeAngle(pts[1], pts[2], pts[0]);
    angles[2] = computeAngle(pts[2], pts[0], pts[1]);

    return angles;
}

// Compute angle given three points A, B, and C
double AngleDistortionHelper::computeAngle(const pmp::Point& A, const pmp::Point& B, const pmp::Point& C)
{
    pmp::Point CA = A - C;
    pmp::Point CB = B - C;

    double dotProduct = CA[0] * CB[0] + CA[1] * CB[1] + CA[2] * CB[2];
    double magnitudeProduct = norm(CA) * norm(CB); // norm computes the magnitude

    return std::acos(dotProduct / magnitudeProduct);
}
```
