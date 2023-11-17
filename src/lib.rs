// wasm-pack uses wasm-bindgen to provide a bridge between the types of JavaScript and Rust
use wasm_bindgen::prelude::*;
use std::env;
use std::path::PathBuf;
// use ffi::ToCppString;
extern crate tobj;
extern crate tri_mesh;

use three_d_asset::Positions::F64;
use tri_mesh::Mesh;

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

    // Convert positions from tobj to the format required by three_d_asset::TriMesh
    let vertices: Vec<tri_mesh::math::Vector3<f64>> = mesh.positions.chunks(3)
        .map(|chunk| tri_mesh::vec3(chunk[0] as f64, chunk[1] as f64, chunk[2] as f64))
        .collect();

    // Create a new TriMesh using the vertices
    let surface_mesh = Mesh::new(&three_d_asset::TriMesh {
        positions: F64(vertices),
        ..Default::default()
    });

    // print the number of vertices
    println!("Number of vertices: {}", surface_mesh.no_vertices());
    println!("Number of halfedges: {}", surface_mesh.no_halfedges());
}

#[wasm_bindgen]
extern {
    pub fn alert(s: &str);
}

#[wasm_bindgen]
pub fn greet() {
    alert("Hello, py-torch!");
}
