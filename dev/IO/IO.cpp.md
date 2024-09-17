---
output_filename: "IO"

brief: "Loading the mesh model vertices and faces"
license: "Apache License 2.0"
---

# IO

The purpose of this module is to provide functions for loading the vertices and faces of a surface mesh model from an OFF file into Eigen matrices.

```cpp
#include "IO.h"

static _3D::Mesh loadMesh(const std::string& filepath)
{
    _3D::Mesh mesh;
    CGAL::IO::read_OFF(filepath, mesh);
    return mesh;
}
```

## Load Mesh Vertices

This function reads the vertices coordinates of a surface mesh model from a file and stores them in an Eigen matrix.
Up to dimension 3 is supported.

```cpp
void loadMeshVertices(const std::string& filepath, Eigen::MatrixXd& vertices)
{
    _3D::Mesh mesh = loadMesh(filepath);
    if (!mesh.number_of_vertices())
    {
        return;
    }

    vertices.resize(mesh.number_of_vertices(), 3);

    size_t i = 0;
    for (const auto& v : mesh.vertices())
    {
        const auto& p = mesh.point(v);
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
    _3D::Mesh mesh = loadMesh(filepath);

    if (!mesh.number_of_faces())
    {
        return;
    }

    faces.resize(mesh.number_of_faces(), 3);

    size_t i = 0;
    for (const auto& f : mesh.faces())
    {
        size_t j = 0;
        // get the vertices of the face f
        for (auto v : CGAL::vertices_around_face(mesh.halfedge(f), mesh))
        {
            faces(i, j) = v.idx();
            j++;
        };
        i++;
    }
}
```
