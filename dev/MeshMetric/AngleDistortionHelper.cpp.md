---
output_filename: "AngleDistortionHelper"

brief: "Calculate the angle distortion of the mesh"
details: "The angle is preserved if the first fundamental form is a multiple of the identity, i.e., I(u) = η(u)Id"
---

```cpp
#include "AngleDistortionHelper.h"

AngleDistortionHelper::AngleDistortionHelper(_3D::Mesh& mesh_open, UV::Mesh& mesh_UV)
    : mesh_open(mesh_open), mesh_UV(mesh_UV)
{
}
```

## Compute Angle Distortion

```cpp
double AngleDistortionHelper::computeAngleDistortion()
{
    double totalDistortion = 0.0;

    for (const auto& f : mesh_open.faces())
    {
        // Get angles for both 3D mesh and UV mesh
        std::vector<double> anglesOpen = triangle_angles(mesh_open, f);

        // Extract the underlying mesh from the UV seam mesh and calculate the angles
        const auto& underlying_mesh = mesh_UV.mesh();
        std::vector<double> anglesUV = triangle_angles(underlying_mesh, f);

        for (int i = 0; i < 3; ++i)
        {
            totalDistortion += std::fabs(anglesOpen[i] - anglesUV[i]);
        }
    }

    return totalDistortion / (3 * mesh_open.number_of_faces()); // Average angle distortion
}
```

### Helper Function: Compute Mesh Triangle Angles

```cpp
template <typename MeshType>
std::vector<double> AngleDistortionHelper::triangle_angles(const MeshType& mesh, const typename MeshType::Face_index& f)
{
    std::vector<Point_3> pts;

    // Collect the vertices of the face
    for (const auto& halfedge : CGAL::halfedges_around_face(mesh.halfedge(f), mesh))
    {
        pts.push_back(mesh.point(CGAL::target(halfedge, mesh)));
    }

    // Compute the angles of the triangle
    std::vector<double> angles(3);
    angles[0] = computeAngle(pts[0], pts[1], pts[2]);
    angles[1] = computeAngle(pts[1], pts[2], pts[0]);
    angles[2] = computeAngle(pts[2], pts[0], pts[1]);

    return angles;
}

// Compute angle given three points A, B, and C
double AngleDistortionHelper::computeAngle(const Point_3& A, const Point_3& B, const Point_3& C)
{
    // Vector from C to A and C to B
    Kernel::Vector_3 CA = A - C;
    Kernel::Vector_3 CB = B - C;

    // Dot product between CA and CB
    double dotProduct = CA[0] * CB[0] + CA[1] * CB[1] + CA[2] * CB[2];

    // Magnitudes of CA and CB
    double magnitudeProduct = std::sqrt(CA.squared_length()) * std::sqrt(CB.squared_length());

    // Return the angle in radians
    return std::acos(dotProduct / magnitudeProduct);
}
```

&NewLine;
## *ScienceLens*: Angle Distortion

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
