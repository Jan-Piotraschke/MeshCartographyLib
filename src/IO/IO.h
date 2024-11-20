#pragma once

#include <Eigen/Dense>
#include <pmp/io/read_off.h>
#include <pmp/surface_mesh.h>
#include <string>

void loadMeshVertices(const std::string& filepath, Eigen::MatrixXd& vertices);

void loadMeshFaces(const std::string& filepath, Eigen::MatrixXi& faces);
