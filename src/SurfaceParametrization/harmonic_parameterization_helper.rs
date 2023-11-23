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


fn cholesky_solve(L: &CsrMatrix<f64>, B: &DMatrix<f64>, is_constrained: impl Fn(usize) -> bool) -> Result<DMatrix<f64>, String> {
    let nrows = L.nrows();
    let ncols = L.ncols();
    let mut dense_L = DMatrix::zeros(nrows, ncols);

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

    // println!("BB: {:?}", BB);




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





// DenseMatrix cholesky_solve(const SparseMatrix& A, const DenseMatrix& B,
//                            std::function<bool(unsigned int)> is_constrained,
//                            const DenseMatrix& C)
// {
//     // build index map; n is #dofs
//     int n = 0;
//     std::vector<int> idx(A.cols(), -1);
//     for (int i = 0; i < A.cols(); ++i)
//         if (!is_constrained(i))
//             idx[i] = n++;

//     // copy columns for rhs
//     DenseMatrix BB(n, B.cols());
//     for (int i = 0; i < A.cols(); ++i)
//         if (idx[i] != -1)
//             BB.row(idx[i]) = B.row(i);

//     // collect entries for reduced matrix
//     // update rhs with constraints
//     std::vector<Triplet> triplets;
//     triplets.reserve(A.nonZeros());
//     for (unsigned int k = 0; k < A.outerSize(); k++)
//     {
//         for (SparseMatrix::InnerIterator iter(A, k); iter; ++iter)
//         {
//             const int i = iter.row();
//             const int j = iter.col();

//             if (idx[i] != -1) // row is dof
//             {
//                 if (idx[j] != -1) // col is dof
//                 {
//                     triplets.emplace_back(idx[i], idx[j], iter.value());
//                 }
//                 else // col is constraint
//                 {
//                     BB.row(idx[i]) -= iter.value() * C.row(j);
//                 }
//             }
//         }
//     }
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
