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

MeshCutHelper::MeshCutHelper(pmp::SurfaceMesh& mesh, pmp::Vertex& start_vertex)
    : mesh(mesh), start_vertex(start_vertex) {};

// ========================================
// Public Functions
// ========================================

/**
 * @brief Calculate the border of the mesh
 */
void MeshCutHelper::cut_mesh_open()
{
    // Get the cutline
    GaussianCutLineHelper cutline_helper = GaussianCutLineHelper(mesh, start_vertex);
    auto edge_path = cutline_helper.get_gaussian_cutline();

    open_mesh_along_seam(edge_path);
}

// ========================================
// Private Functions
// ========================================

std::map<pmp::Vertex, int> MeshCutHelper::get_vertex_neighbors_count() const
{
    std::map<pmp::Vertex, int> vertex_neighbors_count;
    for (const auto& v : mesh.vertices())
    {
        if (std::find(cut_line_vertices.begin(), cut_line_vertices.end(), v) == cut_line_vertices.end())
        {
            vertex_neighbors_count[v] = mesh.valence(v);
        }
    }
    return vertex_neighbors_count;
}
```

We "cut" the 3D mesh by introducing a seam edge along the calculated edge path. This seam edge is used to create the UV mesh.

```cpp
void MeshCutHelper::open_mesh_along_seam(const std::vector<pmp::Edge>& seamEdges)
{
    pmp::SurfaceMesh mesh_uv;

    std::vector<pmp::Vertex> edge_direction;
    edge_direction.push_back(mesh.vertex(seamEdges[0], 0));
    edge_direction.push_back(mesh.vertex(seamEdges[0], 1));
    edge_direction.push_back(mesh.vertex(seamEdges[1], 0));
    edge_direction.push_back(mesh.vertex(seamEdges[1], 1));

    pmp::Vertex heading_towards, heading_from;
    bool found = false;

    for (size_t i = 0; i < edge_direction.size() && !found; ++i)
    {
        for (size_t j = i + 1; j < edge_direction.size(); ++j)
        {
            if (edge_direction[i] == edge_direction[j])
            {
                heading_towards = edge_direction[i];
                found = true;
                break;
            }
        }
    }

    auto option_a = mesh.vertex(seamEdges[0], 0);
    auto option_b = mesh.vertex(seamEdges[0], 1);
    if (option_a == heading_towards)
    {
        heading_from = option_b;
    }
    else
    {
        heading_from = option_a;
    }

    // Collect the special vertices
    cut_line_vertices.push_back(heading_from);

    // 0.1 Get the mapping of seamHalfedge for the seamEdges
    std::vector<pmp::Halfedge> halfedgesPointingToSeam;

    for (size_t i = 0; i < seamEdges.size(); ++i)
    {
        // Get the vertices of the seamEdge
        pmp::Vertex v0 = mesh.vertex(seamEdges[i], 0);
        pmp::Vertex v1 = mesh.vertex(seamEdges[i], 1);
        pmp::Halfedge h;

        if (v0 == heading_from)
        {
            h = mesh.find_halfedge(v0, v1);
            if (!mesh.is_valid(h))
            {
                std::cerr << "Invalid halfedge for vertices: " << v0 << ", " << v1 << std::endl;
                continue;
            }
            halfedgesPointingToSeam.push_back(h);
            // std::cout << v0 << " -> " << v1 << std::endl;
            heading_from = v1;
        }
        else if (v1 == heading_from)
        {
            h = mesh.find_halfedge(v1, v0);
            if (!mesh.is_valid(h))
            {
                std::cerr << "Invalid halfedge for vertices: " << v1 << ", " << v0 << std::endl;
                continue;
            }
            // std::cout << v1 << " -> " << v0 << std::endl;
            halfedgesPointingToSeam.push_back(h);
            heading_from = v0;
        }
        else
        {
            std::cerr << "Neither vertex matches heading_from!" << std::endl;
            continue;
        }
    }

    for (auto h : halfedgesPointingToSeam)
    {
        cut_line_vertices.push_back(mesh.to_vertex(h));
    }

    auto original_vertex_neighbors_count = get_vertex_neighbors_count();
    auto original_edge_count = mesh.n_edges();

    // 0.2 Initialize the mesh: Add all the vertices of the mesh to the mesh_uv
    for (auto v : mesh.vertices())
    {
        mesh_uv.add_vertex(mesh.position(v));
    }

    std::vector<pmp::Vertex> old_vertex;
    std::vector<pmp::Vertex> new_vertex;

    // 0.3 Add the vertices of the seam to the mesh_uv
    for (size_t i = 0; i < halfedgesPointingToSeam.size() - 1; ++i)
    {
        auto v = mesh.to_vertex(halfedgesPointingToSeam[i]);
        old_vertex.push_back(v);
        v = mesh_uv.add_vertex(mesh.position(v));
        new_vertex.push_back(v);
    }

    // add the two other halfedges that heads towards the seam
    auto halfedgesCopy = halfedgesPointingToSeam;
    for (size_t i = 0; i < halfedgesCopy.size() - 1; ++i)
    {
        auto h = halfedgesCopy[i];
        auto vertice_after = cut_line_vertices[i + 2];

        while (true)
        {
            auto next_h = mesh.next_halfedge(h);
            auto h2 = mesh.opposite_halfedge(next_h);
            if (mesh.to_vertex(next_h) == vertice_after)
            {
                break;
            }
            halfedgesPointingToSeam.push_back(h2);
            h = h2;
        }
    }

    for (auto f : mesh.faces())
    {
        // 2. Get the three halfedges of the face
        auto h0 = mesh.halfedge(f);
        auto h1 = mesh.next_halfedge(h0);
        auto h2 = mesh.prev_halfedge(h0);

        // 2.1 Get the vertices of the halfedges
        pmp::Vertex v0 = mesh.to_vertex(h0);
        pmp::Vertex v1 = mesh.to_vertex(h1);
        pmp::Vertex v2 = mesh.to_vertex(h2);

        // find if h0, h1 or h2 is in halfedgesPointingToSeam
        bool h0Exists = std::find(halfedgesPointingToSeam.begin(), halfedgesPointingToSeam.end(), h0)
                        != halfedgesPointingToSeam.end();
        bool h1Exists = std::find(halfedgesPointingToSeam.begin(), halfedgesPointingToSeam.end(), h1)
                        != halfedgesPointingToSeam.end();
        bool h2Exists = std::find(halfedgesPointingToSeam.begin(), halfedgesPointingToSeam.end(), h2)
                        != halfedgesPointingToSeam.end();

        // if any of the halfedges is in halfedgesPointingToSeam
        if (h0Exists || h1Exists || h2Exists)
        {
            // find if v1 inside old_vertex
            auto it = std::find(old_vertex.begin(), old_vertex.end(), v0);
            if (it != old_vertex.end())
            {
                size_t index = std::distance(old_vertex.begin(), it);
                v0 = new_vertex[index];
            }

            it = std::find(old_vertex.begin(), old_vertex.end(), v1);
            if (it != old_vertex.end())
            {
                size_t index = std::distance(old_vertex.begin(), it);
                v1 = new_vertex[index];
            }

            it = std::find(old_vertex.begin(), old_vertex.end(), v2);
            if (it != old_vertex.end())
            {
                size_t index = std::distance(old_vertex.begin(), it);
                v2 = new_vertex[index];
            }
        }

        // 4. Add a new triangle based on the new vertices
        auto new_face = mesh_uv.add_triangle(v0, v1, v2);

        // 5. Check if the new face is valid
        if (!mesh_uv.is_valid(new_face))
        {
            std::cerr << "invalid face found" << std::endl;
        }
    }

    mesh.clear();
    mesh = mesh_uv;

    // Test if the number of neighbors of each vertex is the same as before except for the vertices on the cut line
    std::map<pmp::Vertex, int> new_vertex_neighbors_count = get_vertex_neighbors_count();
    for (auto v : mesh.vertices())
    {
        if (original_vertex_neighbors_count.find(v) != original_vertex_neighbors_count.end())
        {
            if (new_vertex_neighbors_count[v] != original_vertex_neighbors_count[v])
            {
                std::cerr << "vertex " << v << " has " << new_vertex_neighbors_count[v] << " neighbors instead of "
                          << original_vertex_neighbors_count[v] << std::endl;
            }
        }
    }

    // Test if the number of edges is the old number of edges plus the number of edges of the seam -1 through an error
    // message
    auto new_edge_count = mesh.n_edges();
    if (new_edge_count != original_edge_count + cut_line_vertices.size() - 1)
    {
        std::cerr << "The number of edges is not the old number of edges plus the number of edges of the seam -1"
                  << std::endl;
    }
}
```
