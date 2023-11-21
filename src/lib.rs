// wasm-pack uses wasm-bindgen to provide a bridge between the types of JavaScript and Rust
use wasm_bindgen::prelude::*;
use std::env;
use std::fs::File;
use std::io::{Write, Result};
use std::path::PathBuf;
// use ffi::ToCppString;
extern crate tobj;
extern crate tri_mesh;

use tri_mesh::Mesh;

// fn print_type_of<T>(_: &T) {
//     println!("{}", std::any::type_name::<T>())
// }

#[wasm_bindgen]
pub fn read_mesh_from_file() {
    println!("Reading mesh from file...");
    let mesh_cartography_lib_dir_str = env::var("Meshes_Dir").expect("MeshCartographyLib_DIR not set");
    let mesh_cartography_lib_dir = PathBuf::from(mesh_cartography_lib_dir_str);
    let new_path = mesh_cartography_lib_dir.join("ellipsoid_x4_open.obj");

    // Load the mesh from a file
    let model: three_d_asset::Model = three_d_asset::io::load_and_deserialize(new_path.clone()).expect("Failed loading asset");
    let surface_mesh = Mesh::new(&model.geometries[0]);

    // Test if the mesh is valid
    // println!("{:?}", surface_mesh.is_valid());
    println!("Mesh is closed: {:?}", surface_mesh.is_closed());

    // Test if the mesh was created correctly
    // assert_eq!(surface_mesh.no_vertices(), 4725);
    // assert_eq!(surface_mesh.no_faces(), 9336);

    // Save the mesh to a file
    let save_path = mesh_cartography_lib_dir.join("ellipsoid_x4_edited.obj");
    save_mesh_as_obj(&surface_mesh, save_path.clone()).expect("Failed to save mesh to file");

    // Find the boundary vertices
    find_boundary_vertices(&surface_mesh);
}


fn save_mesh_as_obj(mesh: &tri_mesh::Mesh, file_path: PathBuf) -> Result<()> {
    let mut file = File::create(file_path)?;

    // Add meta data
    writeln!(file, "# Generated by MeshCartographyLib")?;

    // Write vertices
    for vertex_id in mesh.vertex_iter() {
        let vertex = mesh.vertex_position(vertex_id);
        writeln!(file, "v {} {} {}", vertex.x, vertex.y, vertex.z)?;
    }

    // Write faces
    for face_id in mesh.face_iter() {
        let face = mesh.face_vertices(face_id);

        // OBJ indices start at 1, so we need to add 1 to each index
        let f0 = face.0.to_string().parse::<i32>().unwrap() + 1;
        let f1 = face.1.to_string().parse::<i32>().unwrap() + 1;
        let f2 = face.2.to_string().parse::<i32>().unwrap() + 1;

        writeln!(file, "f {} {} {}", f0, f1, f2)?;
    }

    Ok(())
}


pub fn find_boundary_vertices(surface_mesh: &Mesh) {

    for halfedge in surface_mesh.halfedge_iter() {
        if surface_mesh.is_edge_on_boundary(halfedge) {
            let boundary_vertices = get_boundary_vertices(surface_mesh, halfedge);

            if boundary_vertices.len() == 3 {
                let mut test_vertices = Vec::new();

                let walker = surface_mesh.walker_from_halfedge(halfedge);
                let start_vertex = walker.clone().into_previous().vertex_id();

                let mut current_vertex = start_vertex;
                let mut i = 0;
                loop {
                    test_vertices.push(current_vertex);

                    let mut vertex_walker = surface_mesh.walker_from_vertex(current_vertex.unwrap());
                    current_vertex = vertex_walker.as_next().vertex_id();
                    println!("Current vertex: {:?}", current_vertex);
                    if current_vertex == start_vertex {
                        break;
                    }
                    i += 1;
                    if i > 10 {
                        break;
                    }
                }

                // let mut vertex_walker = surface_mesh.walker_from_vertex(start_vertex.unwrap());
                // let mut next_vertex = vertex_walker.as_next().vertex_id();
                // println!("Next vertex: {:?}", next_vertex);

                // let mut test_walker = surface_mesh.walker_from_vertex(next_vertex.unwrap());
                // let mut test_next_vertex = test_walker.as_next().vertex_id();
                // println!("Test next vertex: {:?}", test_next_vertex);

                // let mut test_walker2 = surface_mesh.walker_from_vertex(test_next_vertex.unwrap());
                // let mut test_next_vertex2 = test_walker2.as_next().vertex_id();
                // println!("Test next vertex2: {:?}", test_next_vertex2);
            }

            println!("Boundary vertices: {:?}", boundary_vertices);
            println!("Number of boundary vertices: {}", boundary_vertices.len());

            // Stop after finding one boundary loop
            break;
        }
    }
}


fn get_boundary_vertices(surface_mesh: &Mesh, halfedge: tri_mesh::HalfEdgeID) -> Vec<tri_mesh::VertexID> {
    let mut boundary_vertices = Vec::new();
    let start = halfedge;
    let mut current = start;

    loop {
        let walker = surface_mesh.walker_from_halfedge(current);

        // Add the vertex at the start of the halfedge to the list
        boundary_vertices.push(walker.vertex_id().unwrap());

        // Move to the next halfedge along the boundary
        current = walker.into_next().halfedge_id().unwrap();

        // Break the loop if we have completed the loop
        if current == start {
            break;
        }
    }

    boundary_vertices
}


#[wasm_bindgen]
extern {
    pub fn alert(s: &str);
}

#[wasm_bindgen]
pub fn greet() {
    alert("Hello, py-torch!");
}
