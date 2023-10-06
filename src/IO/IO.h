#pragma once

#include <string>
#include <Eigen/Dense>
#include <pmp/surface_mesh.h>
#include <pmp/io/read_off.h>

void loadMeshVertices(
    const std::string& filepath,
    Eigen::MatrixXd& vertices
);

void loadMeshFaces(
    const std::string& filepath,
    Eigen::MatrixXi& faces
);
