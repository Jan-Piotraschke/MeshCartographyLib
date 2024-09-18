---
output_filename: "SurfaceParametrization"

brief: "Create the 2D maps based on the 3D mesh"
---

# Surface Parametrization

The purpose of this module is to provide functions for creating 2D maps based on the 3D mesh model.

## Init

```cpp
#include "SurfaceParametrization.h"

#include "HarmonicParametrizationHelper.h"
#include "MeshCutting/GaussianCutLineHelper.h"
#include "MeshCutting/MeshCutHelper.h"

#include "MeshMetric/AngleDistortionHelper.h"

SurfaceParametrization::SurfaceParametrization()
{
}
```

## Public Helper Functions

Extract the mesh name (without extension) from its file path

```cpp
std::string SurfaceParametrization::get_mesh_name(const std::string mesh_3D_path)
{
    fs::path path(mesh_3D_path);
    return path.stem().string();
}
````

Check if a given point is inside our polygon border, represented by a monotile shape.

```cpp
bool SurfaceParametrization::check_point_in_polygon(const Point_2_eigen& point, bool is_original_mesh)
{
    Point_2 cgal_point(point[0], point[1]);

    auto result = CGAL::bounded_side_2(polygon.vertices_begin(), polygon.vertices_end(), cgal_point, Kernel());

    return result == CGAL::ON_BOUNDED_SIDE || result == CGAL::ON_BOUNDARY;
}
```

## Create the UV Map

```cpp
std::tuple<std::vector<int64_t>, Eigen::MatrixXd, Eigen::MatrixXd, std::string>
SurfaceParametrization::create_uv_surface(std::string mesh_path, int32_t start_node_int)
{
    _3D::vertex_descriptor start_node(start_node_int);
    mesh_3D_file_path = mesh_path;
    auto h_v_mapping_vector = calculate_uv_surface(start_node);

    std::string mesh_uv_file_path = meshmeta.mesh_path;

    extract_polygon_border_edges(mesh_uv_file_path);

    return {h_v_mapping_vector, vertice_UV, vertice_3D, mesh_uv_file_path};
}
```

## Calculate the UV Surface

Calculate the UV coordinates of the 3D mesh and also their mapping to the 3D coordinates

```cpp
std::vector<int64_t> SurfaceParametrization::calculate_uv_surface(_3D::vertex_descriptor start_node)
{
    _3D::Mesh sm;
    std::ifstream in(CGAL::data_file_path(mesh_3D_file_path));
    in >> sm;

    // Canonical Halfedges Representing a Vertex
    _3D::UV_pmap uvmap = sm.add_property_map<_3D::halfedge_descriptor, Point_2>("h:uv").first;

    // Create the seam mesh
    MeshCutHelper mesh_cut_helper = MeshCutHelper(sm, mesh_3D_file_path, start_node);
    MeshCuttingHelperInterface& mesh_cut_helper_interface = mesh_cut_helper;
    UV::Mesh mesh = mesh_cut_helper_interface.cut_mesh_open();

    // Save the open mesh for testing
    fs::path mesh_uv_path_test = MESH_FOLDER / (get_mesh_name(mesh_3D_file_path) + "_open.off");
    CGAL::IO::write_OFF(mesh_uv_path_test.string(), mesh);

    _3D::Mesh mesh_open;
    std::ifstream in_open_mesh(CGAL::data_file_path(mesh_uv_path_test.string()));
    in_open_mesh >> mesh_open;

    // Choose a halfedge on the border
    UV::halfedge_descriptor bhd = CGAL::Polygon_mesh_processing::longest_border(mesh).first;

    // Perform parameterization
    HarmonicParametrizationHelper square_helper = HarmonicParametrizationHelper(mesh, bhd, uvmap);
    ParametrizationHelperInterface& square_border_parametrization_helper = square_helper;
    square_border_parametrization_helper.parameterize_UV_mesh();

    // Save the uv mesh
    save_UV_mesh(mesh, bhd, uvmap, mesh_3D_file_path);

    // Calculate the angle distortion
    // AngleDistortionHelper angle_distortion_helper = AngleDistortionHelper(mesh_open, mesh);
    // double angle_distortion = angle_distortion_helper.computeAngleDistortion();

    std::vector<int64_t> h_v_mapping_vector;
    int number_of_vertices = size(vertices(mesh));
    vertice_3D.resize(number_of_vertices, 3);
    vertice_UV.resize(number_of_vertices, 3);

    int i = 0;
    for (UV::vertex_descriptor vd : vertices(mesh))
    {
        auto [point_3D, uv, target_vertice] = getMeshData(vd, mesh, sm, uvmap);

        h_v_mapping_vector.push_back(target_vertice);

        // Get the points
        vertice_3D(i, 0) = point_3D.x();
        vertice_3D(i, 1) = point_3D.y();
        vertice_3D(i, 2) = point_3D.z();

        // Get the uv points
        vertice_UV(i, 0) = uv.x();
        vertice_UV(i, 1) = uv.y();
        vertice_UV(i, 2) = 0;
        i++;
    }

    return h_v_mapping_vector;
}
```

## Private Helper Functions

```cpp
std::tuple<Point_3, Point_2, int64_t> SurfaceParametrization::getMeshData(
    const UV::vertex_descriptor& vd, const UV::Mesh& mesh, const _3D::Mesh& sm, _3D::UV_pmap& _uvmap)
{
    int64_t target_vertice = target(vd, sm);
    Point_3 point_3D = sm.point(target(vd, sm));
    Point_2 uv = get(_uvmap, halfedge(vd, mesh));
    return {point_3D, uv, target_vertice};
}
```

### Save the generated UV mesh

```cpp
void SurfaceParametrization::save_UV_mesh(
    UV::Mesh _mesh, UV::halfedge_descriptor _bhd, _3D::UV_pmap _uvmap, const std::string mesh_path)
{
    // Get the mesh name without the extension
    auto mesh_3D_name = get_mesh_name(mesh_path);

    // Create the output file path based on uv_mesh_number
    fs::path output_file_path;
    std::string output_file_path_str;

    output_file_path = MESH_FOLDER / (mesh_3D_name + "_uv.off");
    output_file_path_str = output_file_path.string();
    meshmeta.mesh_path = output_file_path_str;

    std::ofstream out(output_file_path_str);
    SMP::IO::output_uvmap_to_off(_mesh, _bhd, _uvmap, out);
}
```

### Extract the polygon border edges

```cpp
void SurfaceParametrization::extract_polygon_border_edges(const std::string& mesh_uv_path)
{
    std::ifstream input(CGAL::data_file_path(mesh_uv_path));
    _3D::Mesh mesh;
    input >> mesh;

    // Find the border edges of the mesh
    std::vector<_3D::halfedge_descriptor> border_edges;
    CGAL::Polygon_mesh_processing::border_halfedges(mesh, std::back_inserter(border_edges));

    // Create a map from source vertex to border halfedge
    std::unordered_map<_3D::vertex_descriptor, _3D::halfedge_descriptor> source_to_halfedge;
    for (const _3D::halfedge_descriptor& h : border_edges)
    {
        source_to_halfedge[mesh.source(h)] = h;
    }

    // Extract the coordinates of the vertices in the correct order
    std::unordered_set<_3D::vertex_descriptor> visited;
    _3D::vertex_descriptor v = mesh.source(border_edges[0]);
    for (std::size_t i = 0; i < border_edges.size(); i++)
    {
        polygon_v.push_back(v);
        polygon.push_back(Point_2(mesh.point(v).x(), mesh.point(v).y()));

        visited.insert(v);

        _3D::halfedge_descriptor next_h = source_to_halfedge[mesh.target(source_to_halfedge[v])];
        v = mesh.source(next_h);

        // Ensure that we don't visit the same vertex again
        if (visited.count(v))
        {
            break;
        }
    }
}
```
