---
output_filename: "HarmonicParametrizationHelper"

brief: "Create a parametrization of the mesh with a square border"
---

# Harmonic Parametrization Helper

The purpose of this module is to provide functions for creating a parametrization of a closed 3D mesh.

Computes a one-to-one mapping from a 3D triangle surface mesh to a simple 2D domain.
The mapping is piecewise linear on the triangle mesh. The result is a pair (U,V) of parameter coordinates for each
vertex of the input mesh. We use discrete conformal mapping to **minimize angle distortion** based on the following paper:
https://doi.org/10.1145/218380.218440

```cpp
#include "HarmonicParametrizationHelper.h"
#include "MonotileBorder/HexagonBorderHelper.h"
#include "MonotileBorder/SquareBorderHelper.h"

HarmonicParametrizationHelper::HarmonicParametrizationHelper(
    pmp::SurfaceMesh& mesh, pmp::Vertex& start_vertex, std::vector<Eigen::Vector2d>& corner_coordinates)
    : mesh(mesh), start_vertex(start_vertex), corner_coordinates(corner_coordinates) {};

// ========================================
// Public Functions
// ========================================

/**
 * @brief Perform UV parameterization
 *
 * Computes a one-to-one mapping from a 3D triangle surface mesh to a simple 2D domain.
 * The mapping is piecewise linear on the triangle mesh. The result is a pair (U,V) of parameter coordinates for each
 * vertex of the input mesh.
 */

void HarmonicParametrizationHelper::parameterize_UV_mesh(bool use_uniform_weights)
{
    std::cout << "Harmonic parameterization... " << std::endl;
    // check precondition
    if (!has_boundary())
    {
        auto what = std::string{__func__} + ": Mesh has no boundary.";
        throw pmp::InvalidInputException(what);
    }

    // create boundary
    // HexagonBorderHelper border_helper = HexagonBorderHelper(mesh, start_vertex);
    // border_helper.setup_hexagon_boundary_constraints();

    SquareBorderHelper border_helper = SquareBorderHelper(mesh, start_vertex);
    border_helper.setup_square_boundary_constraints();

    // get the border corners
    const auto& corners = border_helper.getCorners();
    for (auto v : corners)
    {
        corner_coordinates.push_back(v.position);
    }

    // get properties
    auto UV_coord = mesh.vertex_property<pmp::TexCoord>("v:tex");

    // build system matrix (clamp negative cotan weights to zero)
    pmp::SparseMatrix L;
    if (use_uniform_weights)
    {
        pmp::uniform_laplace_matrix(mesh, L);
    }
    else
    {
        pmp::laplace_matrix(mesh, L, true);
    }

    // build right-hand side B and inject boundary constraints
    pmp::DenseMatrix B(mesh.n_vertices(), 2);
    B.setZero();
    for (auto v : mesh.vertices())
    {
        if (mesh.is_boundary(v))
        {
            B.row(v.idx()) = static_cast<Eigen::Vector2d>(UV_coord[v]);
        }
    }

    // solve system
    auto is_constrained = [&](unsigned int i) { return mesh.is_boundary(pmp::Vertex(i)); };
    pmp::DenseMatrix X = pmp::cholesky_solve(L, B, is_constrained, B);

    // copy solution
    for (auto v : mesh.vertices())
    {
        UV_coord[v] = X.row(v.idx());
    }
}

// ========================================
// Private Functions
// ========================================

bool HarmonicParametrizationHelper::has_boundary()
{
    for (auto v : mesh.vertices())
    {
        if (mesh.is_boundary(v))
        {
            return true;
        }
    }
    return false;
}
```
