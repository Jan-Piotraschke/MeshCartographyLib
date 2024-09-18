---
output_filename: "MeshCutHelper"

brief: "Calculate the cut line of the mesh"
---

# Mesh Cut Helper

The purpose of this module is to provide functions for calculating the cut line of a mesh model.

## 切り方工芸 - Perfekte Schnitttechnik des 3D Meshes

Nicht nur die Form der Grenzen des Monotiles sind wichtig, sondern vorab auch wie man das 3D Mesh aufschneidet! Manchmal kann es sinnvoll sein, eine dünne Struktur wie ein Kamelbein spiralenförmig aufzuschneiden, damit dieses am flächenerhaltestens in 2D parametrisiert werden kann.

## Cut Mesh Open

```cpp
#include "MeshCutHelper.h"
#include "GaussianCutLineHelper.h"

MeshCutHelper::MeshCutHelper(_3D::Mesh& mesh, _3D::vertex_descriptor& start_vertex)
    : mesh(mesh), start_vertex(start_vertex)
{
};

// void MeshCutHelper::cut_mesh_open()
// {
//     // Get the cutline
//     // CutLineHelper helper = CutLineHelper(mesh_3D_file_path, start_vertex);
//     // CutLineHelperInterface& cutline_helper = helper;
//     // auto edge_path = cutline_helper.get_gaussian_cutline();

//     // open_mesh_along_seam(edge_path);
// }
```

We "cut" the 3D mesh by introducing a seam edge along the calculated edge path. This seam edge is used to create the UV mesh.

```cpp
UV::Mesh MeshCutHelper::cut_mesh_open(const std::vector<_3D::edge_descriptor> calc_edges)
{
    // Create property maps to store seam edges and vertices
    _3D::Seam_edge_pmap seam_edge_pm
        = mesh.add_property_map<_3D::edge_descriptor, bool>("e:on_seam", false).first; // if not false -> we can't add
                                                                                       // seam edges
    _3D::Seam_vertex_pmap seam_vertex_pm
        = mesh.add_property_map<_3D::vertex_descriptor, bool>("v:on_seam", false).first; // if not false -> we can't run
                                                                                         // the parameterization part

    UV::Mesh UV_mesh(mesh, seam_edge_pm, seam_vertex_pm);

    for (_3D::edge_descriptor e : calc_edges)
    {
        UV_mesh.add_seam(source(e, mesh), target(e, mesh));
    }
    return UV_mesh;
}
```
