use nalgebra::{DMatrix, Cholesky};
use nalgebra_sparse::CsrMatrix;

extern crate tri_mesh;
use tri_mesh::Mesh;

use crate::mesh_definition;
use crate::mesh_definition::TexCoord;
use crate::SurfaceParametrization::laplacian_matrix;


// Harmonic parameterization
pub fn harmonic_parameterization(mesh: &Mesh, mesh_tex_coords: &mut mesh_definition::MeshTexCoords, use_uniform_weights: bool) {
    let L = laplacian_matrix::build_laplace_matrix(mesh, use_uniform_weights);
    let mut B = DMatrix::zeros(mesh.no_vertices(), 2);

    // Inject boundary constraints
    for vertex_id in mesh.vertex_iter() {
        if mesh.is_vertex_on_boundary(vertex_id) {
            if let Some(tex_coord) = mesh_tex_coords.get_tex_coord(vertex_id) {
                let index_as_u32: u32 = *vertex_id; // Dereference to get u32
                let index_as_usize: usize = index_as_u32 as usize; // Cast u32 to usize
                // println!("{:?}, {:?}", tex_coord.0, tex_coord.1);
                B.set_row(index_as_usize, &nalgebra::RowVector2::new(tex_coord.0, tex_coord.1));
            }
        }
    }

    // Solve the system
    let X = cholesky_solve(&L, &B).unwrap(); // Or handle the error properly

    // Update mesh texture coordinates
    for (vertex_id, row) in mesh.vertex_iter().zip(X.row_iter()) {
        let tex_coord = TexCoord(row[0], row[1]);
        // println!("tex_coord: {:?} {:?}", row[0], row[1]);
        mesh_tex_coords.set_tex_coord(vertex_id, tex_coord);
    }
}


fn cholesky_solve(L: &CsrMatrix<f64>, B: &DMatrix<f64>) -> Result<DMatrix<f64>, String> {
    let nrows = L.nrows();
    let ncols = L.ncols();
    let mut dense_L = DMatrix::zeros(nrows, ncols);

    // println!("L: {:?}", L);
    // Iterating over all rows
    for i in 0..nrows {
        // Iterating over each column
        for j in 0..ncols {
            // println!("i: {:?}, j: {:?}", i, j);
            // if let Some(value) = L.get(i, j) {
            //     dense_L[(i, j)] = value;
            // }
        }
    }


    // Perform Cholesky decomposition
    let cholesky = Cholesky::new(dense_L).ok_or("Failed to factorize linear system.")?;

    // Solve the system
    let X = cholesky.solve(B);

    // Check if the solution is valid
    if X.nrows() != B.nrows() || X.ncols() != B.ncols() {
        return Err("Failed to solve linear system.".to_string());
    }

    Ok(X)
}


