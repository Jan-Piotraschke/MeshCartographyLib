#pragma once

#include "MeshDefinition.h"

class AngleDistortionHelper{
public:
    AngleDistortionHelper(
        pmp::SurfaceMesh& mesh_open,
        pmp::SurfaceMesh& mesh_UV
    );

    double computeAngleDistortion();

private:
    pmp::SurfaceMesh& mesh_open;
    pmp::SurfaceMesh& mesh_UV;

    std::vector<double> triangle_angles(const pmp::SurfaceMesh& mesh, const pmp::Face& f);
    double computeAngle(const pmp::Point& A, const pmp::Point& B, const pmp::Point& C);
};
