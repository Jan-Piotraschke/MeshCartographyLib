---
output_filename: "IO"

brief: "Loading the mesh model vertices and faces; saving and loading data to/from csv files"
license: "Apache License 2.0"
---

# IO

The purpose of this module is to provide functions for loading the vertices and faces of a surface mesh model from an OFF file into Eigen matrices.

```cpp
#include "IO.h"

static pmp::SurfaceMesh loadMesh(const std::string& filepath)
{
    pmp::SurfaceMesh mesh;
    pmp::read_off(mesh, filepath);
    return mesh;
}
```

## Load Mesh Vertices

This function reads the vertices coordinates of a surface mesh model from a file and stores them in an Eigen matrix.
Up to dimension 3 is supported.

```cpp
void loadMeshVertices(const std::string& filepath, Eigen::MatrixXd& vertices)
{
    pmp::SurfaceMesh mesh = loadMesh(filepath);

    if (!mesh.n_vertices())
    {
        return;
    }

    vertices.resize(mesh.n_vertices(), 3);

    size_t i = 0;
    for (const auto& v : mesh.vertices())
    {
        const pmp::Point& p = mesh.position(v);
        vertices(i, 0) = p[0];
        vertices(i, 1) = p[1];
        vertices(i, 2) = p[2];
        i++;
    }
}
```

## Load Mesh Faces

This function reads the faces of a surface mesh model from a file and stores them in an Eigen matrix.
The faces are stored as indices of the vertices.

```cpp
void loadMeshFaces(const std::string& filepath, Eigen::MatrixXi& faces)
{
    pmp::SurfaceMesh mesh = loadMesh(filepath);

    if (!mesh.n_faces())
    {
        return;
    }

    faces.resize(mesh.n_faces(), 3);

    size_t i = 0;
    for (const auto& f : mesh.faces())
    {
        size_t j = 0;
        for (const auto& v : mesh.vertices(f))
        {
            faces(i, j) = v.idx();
            j++;
        }
        i++;
    }
}
```
