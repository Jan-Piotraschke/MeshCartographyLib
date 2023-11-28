// wasm-pack uses wasm-bindgen to provide a bridge between the types of JavaScript and Rust
use wasm_bindgen::prelude::*;
use std::env;
use std::path::PathBuf;

extern crate tri_mesh;
use tri_mesh::Mesh;

mod mesh_definition;
use crate::mesh_definition::TexCoord;

mod io;

#[allow(non_snake_case)]
mod SurfaceParametrization {
    pub mod laplacian_matrix;
    pub mod harmonic_parameterization_helper;
}

// fn print_type_of<T>(_: &T) {
//     println!("{}", std::any::type_name::<T>())
// }

#[wasm_bindgen]
pub fn create_uv_surface() {
    log::info!("Reading mesh from file...");

    let mesh_cartography_lib_dir_str = env::var("Meshes_Dir").expect("MeshCartographyLib_DIR not set");
    let mesh_cartography_lib_dir = PathBuf::from(mesh_cartography_lib_dir_str);
    let new_path = mesh_cartography_lib_dir.join("ellipsoid_x4_open.obj");

    // Load the mesh
    let surface_mesh = io::load_obj_mesh(new_path);

    // Save the mesh to a file
    let save_path = mesh_cartography_lib_dir.join("ellipsoid_x4_edited.obj");
    io::save_mesh_as_obj(&surface_mesh, save_path.clone()).expect("Failed to save mesh to file");

    // Find the boundary vertices
    find_boundary_vertices(&surface_mesh);
}


pub fn find_boundary_vertices(surface_mesh: &Mesh) -> (Vec<tri_mesh::VertexID>, mesh_definition::MeshTexCoords) {
    let mut length = 0.0;
    let boundary_edges = get_boundary_edges(&surface_mesh, &mut length);

    println!("Length of boundary loop: {}", length);
    // NOTE: in c++ this is the result: "Length of boundary loop: 42.3117"

    // Collect edges in a Vec to maintain order
    let edge_list = boundary_edges.iter().cloned().collect::<Vec<_>>();

    // Collect the boundary vertices
    let boundary_vertices = get_boundary_vertices(&edge_list);

    let corner_count = 4;
    let side_length = length / corner_count as f64;
    let tolerance = 1e-4;

    let mut mesh_tex_coords = mesh_definition::MeshTexCoords::new(&surface_mesh);

    for vertex_id in surface_mesh.vertex_iter() {
        mesh_tex_coords.set_tex_coord(vertex_id, TexCoord(0.0, 0.0)); // Initialize to the origin
    }

    let tex_coords = distribute_vertices_around_square(&boundary_vertices, side_length, tolerance, length);
    for (&vertex_id, tex_coord) in boundary_vertices.iter().zip(tex_coords.iter()) {
        mesh_tex_coords.set_tex_coord(vertex_id, TexCoord(tex_coord.0, tex_coord.1));
    }

    SurfaceParametrization::harmonic_parameterization_helper::harmonic_parameterization(&surface_mesh, &mut mesh_tex_coords, true);

    let mesh_cartography_lib_dir_str = env::var("Meshes_Dir").expect("MeshCartographyLib_DIR not set");
    let mesh_cartography_lib_dir = PathBuf::from(mesh_cartography_lib_dir_str);
    let save_path2 = mesh_cartography_lib_dir.join("ellipsoid_x4_uv.obj");
    io::save_uv_mesh_as_obj(&surface_mesh, &mut mesh_tex_coords, save_path2.clone()).expect("Failed to save mesh to file");

    (boundary_vertices, mesh_tex_coords)
}


fn get_boundary_edges(surface_mesh: &Mesh, length: &mut f64) -> Vec<(tri_mesh::VertexID, tri_mesh::VertexID)> {
    let mut boundary_edges = Vec::new();

    for edge in surface_mesh.edge_iter() {
        // Returns the vertex id of the two adjacent vertices to the given edge.
        let v0 = surface_mesh.edge_vertices(edge).0;
        let v1 = surface_mesh.edge_vertices(edge).1;

        if surface_mesh.is_vertex_on_boundary(v0) && surface_mesh.is_vertex_on_boundary(v1) {
            boundary_edges.push((v0, v1));
            *length += surface_mesh.edge_length(edge);
        }
    }

    boundary_edges
}


fn get_boundary_vertices(edge_list: &[(tri_mesh::VertexID, tri_mesh::VertexID)]) -> Vec<tri_mesh::VertexID> {
    if edge_list.is_empty() {
        return Vec::new();
    }

    let mut boundary_vertices = Vec::new();
    let mut current_vertex = edge_list[0].0; // Start with the first vertex
    boundary_vertices.push(current_vertex);

    while boundary_vertices.len() <= edge_list.len() {
        let mut found = false;
        for &(v0, v1) in edge_list {
            if v0 == current_vertex && !boundary_vertices.contains(&v1) {
                current_vertex = v1;
                boundary_vertices.push(current_vertex);
                found = true;
                break;
            } else if v1 == current_vertex && !boundary_vertices.contains(&v0) {
                current_vertex = v0;
                boundary_vertices.push(current_vertex);
                found = true;
                break;
            }
        }
        if !found {
            break; // Break if no next vertex is found
        }
    }

    assert_eq!(boundary_vertices.len(), 112); // Compared with result from C++

    boundary_vertices
}

fn distribute_vertices_around_square(boundary_vertices: &[tri_mesh::VertexID], side_length: f64, tolerance: f64, total_length: f64) -> Vec<TexCoord> {
    let n = boundary_vertices.len();
    let step_size = total_length / n as f64;
    let mut vertices = Vec::new();

    for i in 0..n {
        let mut l = i as f64 * step_size;
        let mut tex_coord;

        // Determine the side and calculate the position
        if l < side_length { // First side (bottom)
            tex_coord = TexCoord(l / side_length, 0.0);
        } else if l < 2.0 * side_length { // Second side (right)
            l -= side_length;
            tex_coord = TexCoord(1.0, l / side_length);
        } else if l < 3.0 * side_length { // Third side (top)
            l -= 2.0 * side_length;
            tex_coord = TexCoord((side_length - l) / side_length, 1.0);
        } else { // Fourth side (left)
            l -= 3.0 * side_length;
            tex_coord = TexCoord(0.0, (side_length - l) / side_length);
        }

        // Apply tolerance
        if tex_coord.0 < tolerance {
            tex_coord.0 = 0.0;
        }
        if tex_coord.1 < tolerance {
            tex_coord.1 = 0.0;
        }

        vertices.push(tex_coord);
    }

    vertices
}

#[wasm_bindgen]
extern {
    pub fn alert(s: &str);
}

#[wasm_bindgen]
pub fn greet() {
    alert("Hello, py-torch!");
}



#[cfg(test)]
mod tests {
    use super::*;

    fn load_test_mesh() -> Mesh {
        let mesh_cartography_lib_dir_str = env::var("Meshes_Dir").expect("MeshCartographyLib_DIR not set");
        let mesh_cartography_lib_dir = PathBuf::from(mesh_cartography_lib_dir_str);
        let new_path = mesh_cartography_lib_dir.join("ellipsoid_x4_open.obj");
        io::load_obj_mesh(new_path)
    }

    #[test]
    fn test_find_boundary_edges() {
        let surface_mesh = load_test_mesh();

        let mut length = 0.0;
        let boundary_edges = get_boundary_edges(&surface_mesh, &mut length);

        assert!(length > 0.0);
        assert_eq!(boundary_edges.len(), 112);

        for &(v0, v1) in &boundary_edges {
            // println!("v0: {:?}, v1: {:?}", v0, v1);
            assert!(surface_mesh.is_vertex_on_boundary(v0));
            assert!(surface_mesh.is_vertex_on_boundary(v1));
        }
    }

    #[test]
    fn test_find_bourdary_vertices() {
        let surface_mesh = load_test_mesh();

        let mut length = 0.0;
        let boundary_edges = get_boundary_edges(&surface_mesh, &mut length);

        // Collect edges in a Vec to maintain order
        let edge_list = boundary_edges.iter().cloned().collect::<Vec<_>>();

        // Collect the boundary vertices
        let boundary_vertices = get_boundary_vertices(&edge_list);

        assert_eq!(boundary_vertices.len(), 112);

        for vertex_id in boundary_vertices {
            // println!("vertex_id: {:?}", vertex_id);
            assert!(surface_mesh.is_vertex_on_boundary(vertex_id));
        }
    }

    #[test]
    fn test_assign_vertices_to_boundary() {
        let surface_mesh = load_test_mesh();

        let mut length = 0.0;
        let boundary_edges = get_boundary_edges(&surface_mesh, &mut length);
        let edge_list = boundary_edges.iter().cloned().collect::<Vec<_>>();
        let boundary_vertices = get_boundary_vertices(&edge_list);

        let corner_count = 4;
        let side_length = length / corner_count as f64;
        let tolerance = 1e-4;

        let mut mesh_tex_coords = mesh_definition::MeshTexCoords::new(&surface_mesh);

        for vertex_id in surface_mesh.vertex_iter() {
            mesh_tex_coords.set_tex_coord(vertex_id, TexCoord(0.0, 0.0)); // Initialize to the origin
        }

        let tex_coords = distribute_vertices_around_square(&boundary_vertices, side_length, tolerance, length);
        for (&vertex_id, tex_coord) in boundary_vertices.iter().zip(tex_coords.iter()) {
            mesh_tex_coords.set_tex_coord(vertex_id, TexCoord(tex_coord.0, tex_coord.1));
        }

        let vertex_id = surface_mesh.vertex_iter().next().unwrap();
        let tex_coord = mesh_tex_coords.get_tex_coord(vertex_id).unwrap();
        assert_eq!(tex_coord.0, 0.0);
    }
}
