#pragma once

#include <Eigen/Dense>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "SurfaceParametrizationHelperInterface.h"

class CutLineHelper : public CutLineHelperInterface
{
  public:
    CutLineHelper(const std::string mesh_3D_file_path, _3D::vertex_descriptor start_node);

    std::vector<_3D::edge_descriptor> set_UV_border_edges() override;

  private:
    const std::string mesh_3D_file_path;
    _3D::vertex_descriptor start_node;

    std::vector<_3D::edge_descriptor> get_cut_line(
        const _3D::Mesh mesh,
        const _3D::vertex_descriptor start_node,
        _3D::vertex_descriptor current,
        const std::vector<_3D::vertex_descriptor> predecessor_pmap);

    void calculate_distances(
        _3D::Mesh mesh,
        _3D::vertex_descriptor start_node,
        std::vector<_3D::vertex_descriptor>& predecessor_pmap,
        std::vector<int>& distance);

    _3D::vertex_descriptor find_farthest_vertex(
        const _3D::Mesh mesh, _3D::vertex_descriptor start_node, const std::vector<int> distance);
};
