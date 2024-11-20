/**
 * @file        IO.cpp
 * @brief       Loading the mesh model vertices and faces; saving and loading data to/from csv files
 *
 * @author      Jan-Piotraschke
 * @date        2023-Jul-18
 * @license     Apache License 2.0
 *
 * @bug         -
 * @todo        -
 */

#include "IO.h"

static pmp::SurfaceMesh loadMesh(const std::string& filepath)
{
    pmp::SurfaceMesh mesh;
    pmp::read_off(mesh, filepath);
    return mesh;
}

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
