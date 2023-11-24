use nalgebra::{Cholesky, DMatrix, Dyn, MatrixViewMut, U1};
use nalgebra_sparse::CsrMatrix;

extern crate tri_mesh;
use tri_mesh::Mesh;

use crate::mesh_definition;
use crate::SurfaceParametrization::laplacian_matrix;


// Harmonic parameterization
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
    // for i in 0..B.nrows() {
    //     if B[(i, 0)] != 0.0 || B[(i, 1)] != 0.0 {
    //         // If at least one of the values in the row is non-zero, print the row
    //         println!("{:.2}, {:.2}", B[(i, 0)], B[(i, 1)]);
    //     }
    // }

    let mut is_constrained = Vec::new();
    for vertex_id in mesh.vertex_iter() {
        is_constrained.push(mesh.is_vertex_on_boundary(vertex_id));
    }

    // Solve the system
    let X = cholesky_solve(&L, &B, |i: usize| is_constrained[i]);
    println!("X: {:?}", X);

    // // Update mesh texture coordinates
    // for (vertex_id, row) in mesh.vertex_iter().zip(X.row_iter()) {
    //     let tex_coord = TexCoord(row[0], row[1]);
    //     // println!("tex_coord: {:?} {:?}", row[0], row[1]);
    //     mesh_tex_coords.set_tex_coord(vertex_id, tex_coord);
    // }
}

struct Triplet<T> {
    row: usize,
    col: usize,
    value: T,
}
use nalgebra::Vector3;
use nalgebra::{Dynamic, SVD, QR, VectorN};


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

    // Convert L to a dense matrix
    let mut dense_L = DMatrix::zeros(ncols, ncols);
    for triplet in L.triplet_iter() {
        let i = triplet.0;
        let j = triplet.1;
        let v = *triplet.2; // Dereference the value

        dense_L[(i, j)] = v;
    }
    // n_dofs: 4613
    println!("n_dofs: {:?}", n_dofs);
    println!("size of triplets: {:?}", triplets.len());

    // print size of BB and dense_L
    println!("BB: {:?} {:?}", BB.nrows(), BB.ncols());
    println!("dense_L: {:?} {:?}", dense_L.nrows(), dense_L.ncols());
//     std::cout << BB.rows() << " " << BB.cols() << std::endl;
//     std::cout << AA.rows() << " " << AA.cols() << std::endl;
//     4613 2
// 4613 4613
    // Perform QR decomposition
    let qr = QR::new(dense_L.clone());

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

// // Define your matrix 'A'
// let a: DMatrix<f64> = DMatrix::identity(4, 4); // Replace with your matrix

// // Define a known vector 'b' using `from_vec`
// let b = VectorN::<f64, Dynamic>::from_vec(vec![1.0, 2.0, 3.0, 4.0]); // Replace with your vector

// // Perform SVD
// let svd = SVD::new(a, true, true);

// // Solve Ax = b using SVD
// let x = svd.solve(&b, 1e-12).expect("Cannot solve system");

// println!("Solution x:\n{}", x);


// DenseMatrix cholesky_solve(const SparseMatrix& A, const DenseMatrix& B,
//                            std::function<bool(unsigned int)> is_constrained,
//                            const DenseMatrix& C)
// {




//     SparseMatrix AA(n, n);
//     AA.setFromTriplets(triplets.begin(), triplets.end());

//     // factorize system
//     Eigen::SimplicialLDLT<SparseMatrix> solver;
//     solver.compute(AA);
//     if (solver.info() != Eigen::Success)
//     {
//         auto what =
//             std::string{__func__} + ": Failed to factorize linear system.";
//         throw SolverException(what);
//     }

//     // solve system
//     const DenseMatrix XX = solver.solve(BB);
//     if (solver.info() != Eigen::Success)
//     {
//         auto what = std::string{__func__} + ": Failed to solve linear system.";
//         throw SolverException(what);
//     }

//     // build full-size result vector from solver result (X) and constraints (C)
//     DenseMatrix X(B.rows(), B.cols());
//     for (int i = 0; i < A.cols(); ++i)
//         X.row(i) = idx[i] == -1 ? C.row(i) : XX.row(idx[i]);

//     return X;
// }
