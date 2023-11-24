use nalgebra::{DMatrix, QR};
use nalgebra_sparse::CsrMatrix;
use num_traits::Zero;
use std::collections::HashMap;
use std::ops::AddAssign;

extern crate tri_mesh;
use tri_mesh::Mesh;

use crate::mesh_definition;
use crate::SurfaceParametrization::laplacian_matrix;

use crate::mesh_definition::TexCoord;

struct Triplet<T> {
    row: usize,
    col: usize,
    value: T,
}

// Harmonic parameterization
#[allow(non_snake_case)]
pub fn harmonic_parameterization(mesh: &Mesh, mesh_tex_coords: &mut mesh_definition::MeshTexCoords, use_uniform_weights: bool) {
    // build system matrix (clamp negative cotan weights to zero)
    let L = laplacian_matrix::build_laplace_matrix(mesh, use_uniform_weights);

    // build right-hand side B and inject boundary constraints
    let mut B = DMatrix::zeros(mesh.no_vertices(), 2);
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

    let mut is_constrained = Vec::new();
    for vertex_id in mesh.vertex_iter() {
        is_constrained.push(mesh.is_vertex_on_boundary(vertex_id));
    }

    // Solve the system
    let result = cholesky_solve(&L, &B, |i: usize| is_constrained[i]);

    match result {
        Ok(X) => {
            for (vertex_id, row) in mesh.vertex_iter().zip(X.row_iter()) {
                let tex_coord = TexCoord(row[0], row[1]);
                // println!("tex_coord: {:?} {:?}", row[0], row[1]);
                mesh_tex_coords.set_tex_coord(vertex_id, tex_coord);
            }
        }
        Err(e) => {
            println!("An error occurred: {}", e);
        }
    }

}


#[allow(non_snake_case)]
fn cholesky_solve(L: &CsrMatrix<f64>, B: &DMatrix<f64>, is_constrained: impl Fn(usize) -> bool) -> Result<DMatrix<f64>, String> {
    let nrows = L.nrows();
    let ncols = L.ncols();

    // Build index map
    let mut idx = vec![usize::MAX; nrows];
    let mut n_dofs = 0;
    for i in 0..nrows {
        if !is_constrained(i) {
            idx[i] = n_dofs;
            n_dofs += 1;
        }
    }

    // Copy columns for RHS
    let mut BB = DMatrix::zeros(n_dofs, B.ncols());
    for i in 0..nrows {
        if let Some(j) = idx.get(i).cloned().filter(|&x| x != usize::MAX) {
            for k in 0..B.ncols() {
                BB[(j, k)] = B[(i, k)];
            }
        }
    }

    // collect entries for reduced matrix
    // update rhs with constraints
    let mut triplets: Vec<Triplet<f64>> = Vec::new();

    // Using triplet_iter to iterate over non-zero elements
    for triplet in L.triplet_iter() {
        let i = triplet.0;
        let j = triplet.1;
        let v = triplet.2;

        if idx[i] != usize::MAX { // row is dof
            if idx[j] != usize::MAX { // col is dof
                triplets.push(Triplet { row: idx[i], col: idx[j], value: *v });
            } else { // col is constraint
                // Update B
                for col in 0..B.ncols() {
                    BB[(idx[i], col)] -= v * B[(j, col)];
                }
            }
        }
    }

    let csr_matrix = build_csr_matrix(n_dofs, n_dofs, &triplets);

    // Convert CSR matrix to dense matrix
    let mut dense_matrix = DMatrix::zeros(csr_matrix.nrows(), csr_matrix.ncols());
    for triplet in csr_matrix.triplet_iter() {
        let i = triplet.0;
        let j = triplet.1;
        let v = *triplet.2;

        dense_matrix[(i, j)] = v;
    }

    // Perform QR decomposition
    let qr = QR::new(dense_matrix.clone());

    // Solve the system Lxx = BB using QR decomposition
    let xx = qr.solve(&BB)
        .ok_or("Failed to solve the system using QR decomposition")?;

    let mut X = DMatrix::zeros(B.nrows(), B.ncols());
    for i in 0..ncols {
        for j in 0..B.ncols() {
            X[(i, j)] = if idx[i] == usize::MAX { B[(i, j)] } else { xx[(idx[i], j)] };
        }
    }

    Ok(X)
}


// Function to convert custom triplets to a CSR matrix
fn build_csr_matrix<T: Copy + nalgebra::Scalar + Zero + AddAssign>(nrows: usize, ncols: usize, triplets: &[Triplet<T>]) -> CsrMatrix<T> {
    let mut entries: HashMap<(usize, usize), T> = HashMap::new();
    for triplet in triplets {
        let key = (triplet.row, triplet.col);
        *entries.entry(key).or_insert_with(Zero::zero) += triplet.value;
    }

    // Sort entries: first by row, then by column
    let mut sorted_entries: Vec<_> = entries.into_iter().collect();
    sorted_entries.sort_by_key(|&((row, col), _)| (row, col));

    // Convert the sorted entries to vectors for CSR matrix construction
    let mut values = Vec::new();
    let mut row_indices = Vec::new();
    let mut col_ptrs = vec![0; nrows + 1];

    for ((row, col), value) in sorted_entries {
        values.push(value);
        row_indices.push(col);  // Note: col indices for each row
        col_ptrs[row + 1] += 1;
    }

    // Compute the starting index of each row
    for i in 1..=nrows {
        col_ptrs[i] += col_ptrs[i - 1];
    }

    // Create the CSR matrix
    CsrMatrix::try_from_csr_data(nrows, ncols, col_ptrs, row_indices, values)
        .expect("Failed to create CSR matrix")
}
