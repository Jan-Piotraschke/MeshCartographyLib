#pragma once

#include <Eigen/Dense>

class MeshCuttingHelperInterface {
public:
    virtual void cut_mesh_open() = 0;
};
