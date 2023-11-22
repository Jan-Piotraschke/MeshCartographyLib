// wasm-pack uses wasm-bindgen to provide a bridge between the types of JavaScript and Rust
use wasm_bindgen::prelude::*;
use std::env;
use std::fs::File;
use std::io::{Write, Result};
use std::path::PathBuf;
use std::collections::HashMap;
use nalgebra::DMatrix;
use nalgebra_sparse::{CooMatrix, CsrMatrix};

extern crate tri_mesh;
use tri_mesh::Mesh;

mod mesh_definition;
use crate::mesh_definition::TexCoord;

mod square_border_helper;
mod io;

// fn print_type_of<T>(_: &T) {
//     println!("{}", std::any::type_name::<T>())
// }

#[wasm_bindgen]
pub fn read_mesh_from_file() {
    log::info!("Reading mesh from file...");

    let mesh_cartography_lib_dir_str = env::var("Meshes_Dir").expect("MeshCartographyLib_DIR not set");
    let mesh_cartography_lib_dir = PathBuf::from(mesh_cartography_lib_dir_str);
    let new_path = mesh_cartography_lib_dir.join("ellipsoid_x4_open.obj");

    // Load the mesh
    let surface_mesh = io::load_obj_mesh(new_path);

    // // Save the mesh to a file
    // let save_path = mesh_cartography_lib_dir.join("ellipsoid_x4_edited.obj");
    // io::save_mesh_as_obj(&surface_mesh, save_path.clone()).expect("Failed to save mesh to file");

    // Find the boundary vertices
    find_boundary_vertices(&surface_mesh);
}


pub fn find_boundary_vertices(surface_mesh: &Mesh) -> (Vec<tri_mesh::VertexID>, mesh_definition::MeshTexCoords) {
    let mut boundary_edges = Vec::new(); // A vector to store boundary edges
    let mut length = 0.0;
    let mut _l = 0.0;

    for edge in surface_mesh.edge_iter() {
        // Returns the vertex id of the two adjacent vertices to the given edge.
        let v0 = surface_mesh.edge_vertices(edge).0;
        let v1 = surface_mesh.edge_vertices(edge).1;

        if surface_mesh.is_vertex_on_boundary(v0) && surface_mesh.is_vertex_on_boundary(v1) {
            boundary_edges.push((v0, v1));
            length += surface_mesh.edge_length(edge);
        }
    }

    let n = boundary_edges.len();
    println!("Length of boundary loop: {}", length);
    // NOTE: in c++ this is the result: "Length of boundary loop: 42.3117"

    // Create a map for easy look-up
    let mut edge_map = HashMap::new();
    for &(v0, v1) in &boundary_edges {
        edge_map.insert(v0, v1);
    }

    // Get the first key from the HashMap
    let start_key = *edge_map.keys().next().expect("HashMap is empty");

    let mut current_key = start_key;
    let mut boundary_vertices = Vec::new();

    // Iterate through the HashMap
    while let Some(&next_value) = edge_map.get(&current_key) {
        boundary_vertices.push(next_value);
        current_key = next_value;

        // Break condition if the sequence becomes too long or cyclic
        if boundary_vertices.len() > edge_map.len() {
            break;
        }
    }

    boundary_vertices.pop();
    assert_eq!(boundary_vertices.len(), 112);  // Compared with result from C++

    let corner_count = 4;
    let side_length = length / corner_count as f64;
    let step_size = length / n as f64;
    let tolerance = 1e-4;

    // let corners = initialize_corners(side_length);

    let mut mesh_tex_coords = mesh_definition::MeshTexCoords::new(&surface_mesh);

    for vertex_id in surface_mesh.vertex_iter() {
        mesh_tex_coords.set_tex_coord(vertex_id, TexCoord(0.0, 0.0)); // Initialize to the origin
    }

    let tex_coords = distribute_vertices_around_square(&boundary_vertices, side_length, tolerance, length);
    for (&vertex_id, tex_coord) in boundary_vertices.iter().zip(tex_coords.iter()) {
        mesh_tex_coords.set_tex_coord(vertex_id, TexCoord(tex_coord.0, tex_coord.1));
    }

    harmonic_parameterization(&surface_mesh, &mut mesh_tex_coords, true);

    (boundary_vertices, mesh_tex_coords)
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

fn cholesky_solve(L: &CsrMatrix<f64>, B: &DMatrix<f64>) -> DMatrix<f64> {
    DMatrix::zeros(B.nrows(), B.ncols())
}

fn build_laplace_matrix(mesh: &Mesh, clamp: bool) -> CsrMatrix<f64> {
    let num_vertices = mesh.no_vertices();
    let mut coo = CooMatrix::new(num_vertices, num_vertices);

    for face in mesh.face_iter() {
        // Assuming face_vertices returns a tuple of VertexIDs
        let (vertex1, vertex2, vertex3) = mesh.face_vertices(face);

        // Use each vertex individually instead of iterating over a tuple
        let vertices = [vertex1, vertex2, vertex3];
        let n = vertices.len();

        // Collect positions of vertices in this face
        let mut polygon = DMatrix::zeros(n, 3);
        for (i, &vertex) in vertices.iter().enumerate() {
            let position = mesh.vertex_position(vertex);
            polygon.set_row(i, &nalgebra::RowVector3::new(position.x, position.y, position.z));
        }

        // Compute local Laplace matrix for these vertices
        let mut Lpoly = polygon_laplace_matrix(&polygon);

        // Assemble local matrices into global matrix
        for (j, &vertex_j) in vertices.iter().enumerate() {
            for (k, &vertex_k) in vertices.iter().enumerate() {
                let index_as_u32: u32 = *vertex_j;
                let idx_j: usize = index_as_u32 as usize;

                let index_as_u32: u32 = *vertex_k;
                let idx_k: usize = index_as_u32 as usize;

                let value = Lpoly[(k, j)];
                coo.push(idx_j, idx_k, value);
            }
        }
    }

    // Convert COO to CSR format
    let csr = CsrMatrix::from(&coo);

    // TODO: implement clamping
    // // Clamping negative off-diagonal entries to zero
    // if clamp {
    //     for i in 0..num_vertices {
    //         let mut diag_offset = 0.0;
    //         for j in 0..num_vertices {
    //             if i != j && csr.get_mut(i, j).map_or(false, |x| *x < 0.0) {
    //                 diag_offset -= csr[(i, j)];
    //                 csr[(i, j)] = 0.0;
    //             }
    //         }
    //         if csr.get_mut(i, i).map_or(false, |x| *x < 0.0) {
    //             *csr.get_mut(i, i).unwrap() -= diag_offset;
    //         }
    //     }
    // }

    csr
}


fn polygon_laplace_matrix(polygon: &DMatrix<f64>) -> DMatrix<f64> {
    // Implementation depends on how you calculate the local Laplace matrix
    // for a polygon. This needs the positions of the vertices of the polygon.
    DMatrix::zeros(polygon.nrows(), polygon.ncols()) // Placeholder
}

// Main implementation
fn harmonic_parameterization(mesh: &Mesh, mesh_tex_coords: &mut mesh_definition::MeshTexCoords, use_uniform_weights: bool) {
    let L = build_laplace_matrix(mesh, use_uniform_weights);
    let mut B = DMatrix::zeros(mesh.no_vertices(), 2);

    // Inject boundary constraints
    for vertex_id in mesh.vertex_iter() {
        if mesh.is_vertex_on_boundary(vertex_id) {
            if let Some(tex_coord) = mesh_tex_coords.get_tex_coord(vertex_id) {
                let index_as_u32: u32 = *vertex_id; // Dereference to get u32
                let index_as_usize: usize = index_as_u32 as usize; // Cast u32 to usize
                B.set_row(index_as_usize, &nalgebra::RowVector2::new(tex_coord.0, tex_coord.1));
            }
        }
    }

    // Solve the system
    let X = cholesky_solve(&L, &B);

    // Update mesh texture coordinates
    for (vertex_id, row) in mesh.vertex_iter().zip(X.row_iter()) {
        let tex_coord = TexCoord(row[0], row[1]);
        // println!("tex_coord: {:?} {:?}", row[0], row[1]);
        mesh_tex_coords.set_tex_coord(vertex_id, tex_coord);
    }
}


#[wasm_bindgen]
extern {
    pub fn alert(s: &str);
}

#[wasm_bindgen]
pub fn greet() {
    alert("Hello, py-torch!");
}
