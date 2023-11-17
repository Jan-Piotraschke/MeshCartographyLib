// wasm-pack uses wasm-bindgen to provide a bridge between the types of JavaScript and Rust
use wasm_bindgen::prelude::*;
use std::env;
use std::fs::File;
use std::io::{Write, Result};
use std::path::PathBuf;
// use ffi::ToCppString;
extern crate tobj;
extern crate tri_mesh;

use three_d_asset::Positions::F64;
use three_d_asset::io::*;
use tri_mesh::Mesh;

fn print_type_of<T>(_: &T) {
    println!("{}", std::any::type_name::<T>())
}

#[wasm_bindgen]
pub fn read_mesh_from_file() {
    println!("Reading mesh from file...");
    let mesh_cartography_lib_dir_str = env::var("Meshes_Dir").expect("MeshCartographyLib_DIR not set");
    let mesh_cartography_lib_dir = PathBuf::from(mesh_cartography_lib_dir_str);
    let new_path = mesh_cartography_lib_dir.join("ellipsoid_x4_open.obj");

    let mesh_object = tobj::load_obj(new_path.clone(), &tobj::LoadOptions::default());
    assert!(mesh_object.is_ok());

    // Unwrap the mesh object
    let (models, _materials) = mesh_object.unwrap();
    let mesh = &models[0].mesh;

    // ! BUG: wir erschaffen das surface_mesh noch nicht richtig. Es hat mit 1575 viel zu wenig faces

    // Convert positions from tobj to the format required by three_d_asset::TriMesh
    let vertices: Vec<tri_mesh::math::Vector3<f64>> = mesh.positions.chunks(3)
        .map(|chunk| tri_mesh::vec3(chunk[0] as f64, chunk[1] as f64, chunk[2] as f64))
        .collect();

    // Create a new TriMesh using the vertices
    let surface_mesh = Mesh::new(&three_d_asset::TriMesh {
        positions: F64(vertices),
        ..Default::default()
    });

    // Test if the mesh was created correctly
    assert_eq!(surface_mesh.no_vertices(), 4725);
    assert_eq!(surface_mesh.no_faces(), 9336);

    // ! Ende bug

    // Save the mesh to a file
    let save_path = mesh_cartography_lib_dir.join("ellipsoid_x4_edited.obj");
    print_type_of(&surface_mesh);
    save_mesh_as_obj(&surface_mesh, save_path.clone()).expect("Failed to save mesh to file");

    // Find the boundary vertices
    // find_boundary_vertices(&surface_mesh);
}


fn save_mesh_as_obj(mesh: &tri_mesh::Mesh, file_path: PathBuf) -> Result<()> {
    let mut file = File::create(file_path)?;

    // Write vertices
    for vertex_id in mesh.vertex_iter() {
        let vertex = mesh.vertex_position(vertex_id);
        writeln!(file, "v {} {} {}", vertex.x, vertex.y, vertex.z)?;
    }

    // Write faces
    for face_id in mesh.face_iter() {
        let face = mesh.face_vertices(face_id);
        // Adjust for OBJ file indexing (starts at 1)
        writeln!(file, "f {} {} {}", face.0, face.1, face.2)?;
    }

    Ok(())
}


pub fn find_boundary_vertices(surface_mesh: &Mesh) {
    // Vector to store boundary vertices
    // let mut boundary_vertices: Vec<T> = Vec::new();

    // Iterate over all halfedges
    let mut i = 0;
    // let mut one_ring_average_position = Vec3::zero();
    // let mut i = 0;
    // for halfedge_id in mesh.vertex_halfedge_iter(vertex_id) {
    //     let walker = mesh.walker_from_halfedge(halfedge_id);
    //     one_ring_average_position += mesh.vertex_position(walker.vertex_id().unwrap());
    //     i = i+1;
    // }
    // one_ring_average_position /= i as f64;
    for halfedge in surface_mesh.halfedge_iter() {
        // Check if the halfedge is a boundary
        if surface_mesh.is_edge_on_boundary(halfedge) {

            // Start at the current halfedge and traverse the boundary loop
            let start = halfedge;
            let mut current = start;

            let walker = surface_mesh.walker_from_halfedge(current);
            // println!("Walker: {:?}"s, walker.vertex_id());
            i += 1;
            // loop {
            //     // Get the vertex at the start of the halfedge
            //     let vertex = surface_mesh.walker_from_vertex(current);
            //     boundary_vertices.push(vertex);

            //     // Move to the next halfedge along the boundary
            //     current = surface_mesh.as(current);

            //     // Break the loop if we have completed the loop
            //     if current == start {
            //         break;
            //     }
            // }

            // // Stop after finding one boundary loop
            // break;
        }
    }
    println!("Number of boundary loops: {}", i);

    // // Process the boundary vertices as needed
    // println!("Boundary vertices: {:?}", boundary_vertices);
    // println!("Number of boundary vertices: {}", boundary_vertices.len());
}

#[wasm_bindgen]
extern {
    pub fn alert(s: &str);
}

#[wasm_bindgen]
pub fn greet() {
    alert("Hello, py-torch!");
}
