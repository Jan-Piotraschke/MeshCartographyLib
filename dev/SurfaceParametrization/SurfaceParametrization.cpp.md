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
#include "MeshCutting/MeshCutHelper.h"

#include "MeshMetric/AngleDistortionHelper.h"
#include "MeshMetric/FaceDistortionHelper.h"
#include "MeshMetric/LengthDistortionHelper.h"

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
```

Check if a given point is inside our polygon border, represented by a monotile shape.

```cpp
bool SurfaceParametrization::check_point_in_polygon(const Point_2_eigen& point)
{
    bool inside = false;
    int n = polygon.size();
    for (int i = 0; i < n; ++i)
    {
        const Point_2_eigen& p1 = polygon[i];
        const Point_2_eigen& p2 = polygon[(i + 1) % n];

        // Check if point is on a vertex
        if (point == p1 || point == p2)
        {
            return true;
        }

        // Check if point is on a horizontal boundary
        if ((p1[1] == point[1] && p2[1] == point[1])
            && (point[0] > std::min(p1[0], p2[0]) && point[0] < std::max(p1[0], p2[0])))
        {
            return true;
        }

        // Check if point is on a vertical boundary
        if ((p1[0] == point[0] && p2[0] == point[0])
            && (point[1] > std::min(p1[1], p2[1]) && point[1] < std::max(p1[1], p2[1])))
        {
            return true;
        }

        if ((p1[1] > point[1]) != (p2[1] > point[1])
            && (point[0] < (p2[0] - p1[0]) * (point[1] - p1[1]) / (p2[1] - p1[1]) + p1[0]))
        {
            inside = !inside;
        }
    }
    return inside;
}
```

## Create the UV Map

```cpp
std::tuple<std::vector<int64_t>, Eigen::MatrixXd, Eigen::MatrixXd, std::string>
SurfaceParametrization::create_uv_surface(std::string mesh_path, int32_t start_node_int)
{
    pmp::Vertex start_node(start_node_int);
    mesh_3D_file_path = mesh_path;
    auto h_v_mapping_vector = calculate_uv_surface(start_node);

    fs::path mesh_uv_path = MESH_FOLDER / (get_mesh_name(mesh_3D_file_path) + "_uv.off");
    meshmeta.mesh_path = mesh_uv_path.string();
    std::string mesh_uv_file_path = meshmeta.mesh_path;
    extract_polygon_border_edges(mesh_uv_file_path);

    return {h_v_mapping_vector, vertice_UV, vertice_3D, mesh_uv_file_path};
}
```

## Calculate the UV Surface

Calculate the UV coordinates of the 3D mesh and also their mapping to the 3D coordinates

```cpp
std::vector<int64_t> SurfaceParametrization::calculate_uv_surface(pmp::Vertex start_vertex)
{
    pmp::SurfaceMesh mesh;
    pmp::read_off(mesh, mesh_3D_file_path);

    // Set the border edges of the UV mesh
    MeshCutHelper helper = MeshCutHelper(mesh, start_vertex);
    MeshCuttingHelperInterface& cutline_helper = helper;
    cutline_helper.cut_mesh_open();

    // Save the open mesh for testing
    fs::path mesh_uv_path_test = MESH_FOLDER / (get_mesh_name(mesh_3D_file_path) + "_open.off");
    pmp::write(mesh, mesh_uv_path_test.string());

    // Perform the parameterization
    HarmonicParametrizationHelper helper_p = HarmonicParametrizationHelper(mesh, start_vertex, corners);
    ParametrizationHelperInterface& parametrization_helper = helper_p;
    parametrization_helper.parameterize_UV_mesh(false);

    // Save the uv mesh
    fs::path mesh_uv_path = MESH_FOLDER / (get_mesh_name(mesh_3D_file_path) + "_uv.off");
    save_uv_as_mesh(mesh, mesh_uv_path);

    pmp::SurfaceMesh mesh_open, mesh_uv;
    pmp::read_off(mesh_open, mesh_uv_path_test.string());
    pmp::read_off(mesh_uv, mesh_uv_path.string());

    // Calculate the angle distortion
    AngleDistortionHelper angle_distortion_helper = AngleDistortionHelper(mesh_open, mesh_uv);
    double angle_distortion = angle_distortion_helper.computeAngleDistortion();

    // Calculate the face distortion
    FaceDistortionHelper face_distortion_helper = FaceDistortionHelper(mesh_open, mesh_uv);
    double face_distortion = face_distortion_helper.computeFaceDistortion();

    // Calculate the length distortion
    LengthDistortionHelper length_distortion_helper = LengthDistortionHelper(mesh_open, mesh_uv);
    double length_distortion = length_distortion_helper.computeLengthDistortion();

    std::vector<int64_t> h_v_mapping_vector;
    int number_of_vertices = mesh.n_vertices();
    vertice_3D.resize(number_of_vertices, 3);
    vertice_UV.resize(number_of_vertices, 3);

    auto UV_coord = mesh.get_vertex_property<pmp::TexCoord>("v:tex");
    auto points = mesh.get_vertex_property<pmp::Point>("v:point");

    int i = 0;
    for (pmp::Vertex vd : mesh.vertices())
    {
        int64_t target_id = vd.idx();
        h_v_mapping_vector.push_back(target_id);

        Point_3_pmp point_3D = points[vd];

        // Get the points
        vertice_3D(i, 0) = point_3D(0, 0);
        vertice_3D(i, 1) = point_3D(1, 0);
        vertice_3D(i, 2) = point_3D(2, 0);

        // Get the uv points
        vertice_UV(i, 0) = UV_coord[vd][0];
        vertice_UV(i, 1) = UV_coord[vd][1];
        vertice_UV(i, 2) = 0;
        i++;
    }

    return h_v_mapping_vector;
}

void SurfaceParametrization::save_uv_as_mesh(const pmp::SurfaceMesh& mesh, const fs::path& filename)
{
    pmp::SurfaceMesh uvMesh;

    // Get UV coordinates property
    auto UV_coord = mesh.get_vertex_property<pmp::TexCoord>("v:tex");
    if (!UV_coord)
        throw std::runtime_error("UV coordinates not found!");

    // Create a mapping from the old vertices to the new vertices
    std::map<pmp::Vertex, pmp::Vertex> vertexMapping;

    // Add vertices to uvMesh using UV coordinates as positions
    for (auto v : mesh.vertices())
    {
        pmp::Point uvPoint(UV_coord[v][0], UV_coord[v][1], 0.0); // We set the Z coordinate to 0
        vertexMapping[v] = uvMesh.add_vertex(uvPoint);
    }

    // Add faces to uvMesh
    for (auto f : mesh.faces())
    {
        std::vector<pmp::Vertex> uvFaceVertices;
        for (auto v : mesh.vertices(f))
        {
            uvFaceVertices.push_back(vertexMapping[v]);
        }
        uvMesh.add_face(uvFaceVertices);
    }

    // Write uvMesh to an .off file
    pmp::write(uvMesh, filename.string());
}

void SurfaceParametrization::extract_polygon_border_edges(const std::string& mesh_uv_path)
{
    pmp::SurfaceMesh mesh;
    pmp::read_off(mesh, mesh_uv_path);

    // Find the border edges of the mesh
    std::vector<pmp::Halfedge> border_edges;

    // get properties
    auto points = mesh.vertex_property<pmp::Point>("v:point");
    auto tex = mesh.vertex_property<pmp::TexCoord>("v:tex");

    // find 1st boundary vertex here, as all UV meshes have this as the origin of their parameterization
    pmp::Vertex vh;
    for (auto v : mesh.vertices())
    {
        if (points[v] == pmp::Point(0, 0, 0))
        {
            vh = v;
            break;
        }
    }

    // collect boundary edges
    pmp::Halfedge hh = mesh.halfedge(vh);
    do
    {
        border_edges.push_back(hh);
        hh = mesh.next_halfedge(hh);
    } while (hh != mesh.halfedge(vh));

    // Extract the coordinates of the vertices in the correct order
    int current_border = 0;
    auto first_v = mesh.from_vertex(border_edges[0]);
    auto first_position = mesh.position(first_v);
    Eigen::Vector2d first_point(first_position[0], first_position[1]);
    border_v_map[current_border].push_back(first_v);
    border_map[current_border].push_back(first_point);

    for (const auto& h : border_edges)
    {
        auto v = mesh.to_vertex(h);
        polygon_v.push_back(v);

        auto position = mesh.position(v);
        Eigen::Vector2d point = Eigen::Vector2d(position[0], position[1]);
        polygon.push_back(point);

        border_v_map[current_border].push_back(v);
        border_map[current_border].push_back(point);

        // Check if we crossed a corner and need to start a new border
        for (size_t i = 0; i < corners.size(); ++i)
        {
            if (point.isApprox(corners[i], 1e-4) && v != first_v)
            {
                ++current_border;
                border_v_map[current_border].push_back(v);
                border_map[current_border].push_back(point);
                break;
            }
        }
    }

    // create the twin border map
    int corner_count = corners.size();
    for (int i = 0; i < corner_count - 1; ++i)
    {
        twin_border_map[i] = current_border - i;
        twin_border_map[current_border - i] = i;
    }
}
```
