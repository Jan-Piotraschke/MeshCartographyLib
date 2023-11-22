use nalgebra::{DMatrix, Cholesky};
use nalgebra::{Point3, Vector3};
use nalgebra_sparse::{CooMatrix, CsrMatrix};

extern crate tri_mesh;
use tri_mesh::Mesh;

use crate::mesh_definition;
use crate::mesh_definition::TexCoord;


// Harmonic parameterization
pub fn harmonic_parameterization(mesh: &Mesh, mesh_tex_coords: &mut mesh_definition::MeshTexCoords, use_uniform_weights: bool) {
    let L = build_laplace_matrix(mesh, use_uniform_weights);
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


fn build_laplace_matrix(mesh: &Mesh, clamp: bool) -> CsrMatrix<f64> {
    let num_vertices = mesh.no_vertices();
    let mut coo = CooMatrix::new(num_vertices, num_vertices);

    for face in mesh.face_iter() {
        let (vertex1, vertex2, vertex3) = mesh.face_vertices(face);

        // Use each vertex individually instead of iterating over a tuple
        let vertices = [vertex1, vertex2, vertex3];

        // Create an array of Point3<f64> for the triangle
        let triangle = [
            Point3::new(mesh.vertex_position(vertex1).x, mesh.vertex_position(vertex1).y, mesh.vertex_position(vertex1).z),
            Point3::new(mesh.vertex_position(vertex2).x, mesh.vertex_position(vertex2).y, mesh.vertex_position(vertex2).z),
            Point3::new(mesh.vertex_position(vertex3).x, mesh.vertex_position(vertex3).y, mesh.vertex_position(vertex3).z),
        ];

        // Compute local Laplace matrix for the triangle
        let Lpoly = calculate_laplacian_matrix(&triangle);

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

    csr
}


fn calculate_laplacian_matrix(polygon: &[Point3<f64>]) -> DMatrix<f64> {
    let a = polygon[0];
    let b = polygon[1];
    let c = polygon[2];

    let ab = b - a;
    let bc = c - b;
    let ca = a - c;

    let cot_cab = cotangent_angle(&ca, &-bc);
    let cot_abc = cotangent_angle(&ab, &-ca);
    let cot_bca = cotangent_angle(&bc, &-ab);

    DMatrix::from_row_slice(3, 3, &[
        -cot_cab, cot_cab, 0.0,
        cot_cab, -(cot_cab + cot_bca), cot_bca,
        0.0, cot_bca, -cot_bca
    ])
}

fn cotangent_angle(v0: &Vector3<f64>, v1: &Vector3<f64>) -> f64 {
    let dot = v0.dot(v1);
    let cross = v0.cross(v1).norm();

    dot / cross
}


#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::{Point3, Vector3};

    #[test]
    fn test_cotangent_angle() {
        let v0 = Vector3::new(1.0, 0.0, 0.0);
        let v1 = Vector3::new(0.0, 1.0, 0.0);

        let cotangent = cotangent_angle(&v0, &v1);
        assert_eq!(cotangent, 0.0); // Cotangent of 90 degrees is 0
    }

    #[test]
    fn test_calculate_laplacian_matrix() {
        // Define a simple equilateral triangle
        let triangle = [
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(0.5, 0.86602540378, 0.0), // sin(60 degrees) = ~0.866
        ];

        let laplace_matrix = calculate_laplacian_matrix(&triangle);
        // Check if the matrix has the expected properties, e.g., symmetry, non-zero values at certain positions, etc.
        // As an example, checking symmetry:
        assert_eq!(laplace_matrix[(0, 1)], laplace_matrix[(1, 0)]);
        assert_eq!(laplace_matrix[(1, 2)], laplace_matrix[(2, 1)]);
        assert_eq!(laplace_matrix[(2, 0)], laplace_matrix[(0, 2)]);
    }

    #[test]
    fn test_cotangent_angle_acute() {
        let v0 = Vector3::new(1.0, 0.0, 0.0);
        let v1 = Vector3::new(1.0, 1.0, 0.0);

        let cotangent = cotangent_angle(&v0, &v1);
        assert!(cotangent > 0.0); // Cotangent of an acute angle is positive
    }

    #[test]
    fn test_cotangent_angle_obtuse() {
        let v0 = Vector3::new(1.0, 0.0, 0.0);
        let v1 = Vector3::new(-1.0, 1.0, 0.0);

        let cotangent = cotangent_angle(&v0, &v1);
        assert!(cotangent < 0.0); // Cotangent of an obtuse angle is negative
    }

    #[test]
    fn test_calculate_laplacian_matrix_values() {
        // Define a simple right-angle triangle
        let triangle = [
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
        ];

        let laplace_matrix = calculate_laplacian_matrix(&triangle);

        assert_eq!(laplace_matrix[(0, 1)], 1.0);
        assert_eq!(laplace_matrix[(1, 0)], 1.0);
        assert_eq!(laplace_matrix[(1, 2)], 1.0);
        assert_eq!(laplace_matrix[(2, 1)], 1.0);
        assert_eq!(laplace_matrix[(0, 0)], -1.0);
        assert_eq!(laplace_matrix[(1, 1)], -2.0);
        assert_eq!(laplace_matrix[(2, 2)], -1.0);
    }

    #[test]
    fn test_calculate_laplacian_matrix_isosceles_triangle() {
        // Define an isosceles triangle
        let triangle = [
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(2.0, 0.0, 0.0),
            Point3::new(1.0, 1.0, 0.0),
        ];

        let laplace_matrix = calculate_laplacian_matrix(&triangle);

        assert_eq!(laplace_matrix[(0, 0)], 0.0);
        assert_eq!(laplace_matrix[(0, 1)], 0.0);
        assert_eq!(laplace_matrix[(0, 2)], 0.0);
        assert_eq!(laplace_matrix[(1, 0)], 0.0);
        assert_eq!(laplace_matrix[(1, 1)], -1.0);
        assert_eq!(laplace_matrix[(1, 2)], 1.0);
        assert_eq!(laplace_matrix[(2, 0)], 0.0);
        assert_eq!(laplace_matrix[(2, 1)], 1.0);
        assert_eq!(laplace_matrix[(2, 2)], -1.0);
    }
}
