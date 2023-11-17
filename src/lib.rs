// wasm-pack uses wasm-bindgen to provide a bridge between the types of JavaScript and Rust
use wasm_bindgen::prelude::*;
use std::env;
use std::path::PathBuf;
// use ffi::ToCppString;
extern crate tobj;

#[wasm_bindgen]
pub fn read_mesh_from_file() {
    println!("Reading mesh from file...");
    let mesh_cartography_lib_dir_str = env::var("Meshes_Dir").expect("MeshCartographyLib_DIR not set");
    let mesh_cartography_lib_dir = PathBuf::from(mesh_cartography_lib_dir_str);
    let new_path = mesh_cartography_lib_dir.join("ellipsoid_x4_open.obj");

    let mesh_object = tobj::load_obj(new_path, &tobj::LoadOptions::default());
    assert!(mesh_object.is_ok());

    // Unwrap the mesh object
    let (models, _materials) = mesh_object.unwrap();

    let mesh = &models[0].mesh;
    println!("Number of vertices: {}", mesh.positions.len() / 3);
}

#[wasm_bindgen]
extern {
    pub fn alert(s: &str);
}

#[wasm_bindgen]
pub fn greet() {
    alert("Hello, py-torch!");
}
