#pragma once

#include "MeshDefinition.h"
#include <CGAL/Surface_mesh/IO/OFF.h>
#include <Eigen/Dense>
#include <string>

void loadMeshVertices(const std::string& filepath, Eigen::MatrixXd& vertices);

void loadMeshFaces(const std::string& filepath, Eigen::MatrixXi& faces);
