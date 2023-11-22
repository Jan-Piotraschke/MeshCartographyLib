use nalgebra::DMatrix;
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

    // print csr matrix
    csr
}

fn polygon_laplace_matrix(polygon: &DMatrix<f64>) -> DMatrix<f64> {
    // Implementation depends on how you calculate the local Laplace matrix
    // for a polygon. This needs the positions of the vertices of the polygon.
    DMatrix::zeros(polygon.nrows(), polygon.ncols()) // Placeholder
}
