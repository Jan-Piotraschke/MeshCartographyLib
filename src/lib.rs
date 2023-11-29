// wasm-pack uses wasm-bindgen to provide a bridge between the types of JavaScript and Rust
use wasm_bindgen::prelude::*;
use std::env;
use std::path::PathBuf;

extern crate tri_mesh;
use tri_mesh::Mesh;

mod mesh_definition;
use crate::mesh_definition::TexCoord;

mod io;

#[allow(non_snake_case)]
mod SurfaceParametrization {
    pub mod laplacian_matrix;
    pub mod harmonic_parameterization_helper;
}

// fn print_type_of<T>(_: &T) {
//     println!("{}", std::any::type_name::<T>())
// }

#[wasm_bindgen]
pub fn create_uv_surface() {
    log::info!("Reading mesh from file...");

    let mesh_cartography_lib_dir_str = env::var("Meshes_Dir").expect("MeshCartographyLib_DIR not set");
    let mesh_cartography_lib_dir = PathBuf::from(mesh_cartography_lib_dir_str);
    let new_path = mesh_cartography_lib_dir.join("ellipsoid_x4_open.obj");

    // Load the mesh
    let surface_mesh = io::load_obj_mesh(new_path);

    // Save the mesh to a file
    let save_path = mesh_cartography_lib_dir.join("ellipsoid_x4_edited.obj");
    io::save_mesh_as_obj(&surface_mesh, save_path.clone()).expect("Failed to save mesh to file");

    // Find the boundary vertices
    find_boundary_vertices(&surface_mesh);
}


pub fn find_boundary_vertices(surface_mesh: &Mesh) -> (Vec<tri_mesh::VertexID>, mesh_definition::MeshTexCoords) {
    let mut length = 0.0;
    let boundary_edges = get_boundary_edges(&surface_mesh, &mut length);

    println!("Length of boundary loop: {}", length);
    // NOTE: in c++ this is the result: "Length of boundary loop: 42.3117"

    // Collect edges in a Vec to maintain order
    let edge_list = boundary_edges.iter().cloned().collect::<Vec<_>>();

    // Collect the boundary vertices
    let boundary_vertices = get_boundary_vertices(&edge_list);

    let corner_count = 4;
    let side_length = length / corner_count as f64;
    let tolerance = 1e-4;

    let mut mesh_tex_coords = mesh_definition::MeshTexCoords::new(&surface_mesh);

    for vertex_id in surface_mesh.vertex_iter() {
        mesh_tex_coords.set_tex_coord(vertex_id, TexCoord(0.0, 0.0)); // Initialize to the origin
    }

    let tex_coords = distribute_vertices_around_square(&boundary_vertices, side_length, tolerance, length);
    for (&vertex_id, tex_coord) in boundary_vertices.iter().zip(tex_coords.iter()) {
        mesh_tex_coords.set_tex_coord(vertex_id, TexCoord(tex_coord.0, tex_coord.1));
    }

    SurfaceParametrization::harmonic_parameterization_helper::harmonic_parameterization(&surface_mesh, &mut mesh_tex_coords, true);

    let mesh_cartography_lib_dir_str = env::var("Meshes_Dir").expect("MeshCartographyLib_DIR not set");
    let mesh_cartography_lib_dir = PathBuf::from(mesh_cartography_lib_dir_str);
    let save_path2 = mesh_cartography_lib_dir.join("ellipsoid_x4_uv.obj");
    io::save_uv_mesh_as_obj(&surface_mesh, &mut mesh_tex_coords, save_path2.clone()).expect("Failed to save mesh to file");

    (boundary_vertices, mesh_tex_coords)
}


fn get_boundary_edges(surface_mesh: &Mesh, length: &mut f64) -> Vec<(tri_mesh::VertexID, tri_mesh::VertexID)> {
    let mut boundary_edges = Vec::new();

    for edge in surface_mesh.edge_iter() {
        // Returns the vertex id of the two adjacent vertices to the given edge.
        let v0 = surface_mesh.edge_vertices(edge).0;
        let v1 = surface_mesh.edge_vertices(edge).1;

        if surface_mesh.is_vertex_on_boundary(v0) && surface_mesh.is_vertex_on_boundary(v1) {
            boundary_edges.push((v0, v1));
            *length += surface_mesh.edge_length(edge);
        }
    }

    boundary_edges
}


fn get_boundary_vertices(edge_list: &[(tri_mesh::VertexID, tri_mesh::VertexID)]) -> Vec<tri_mesh::VertexID> {
    if edge_list.is_empty() {
        return Vec::new();
    }

    let mut boundary_vertices = Vec::new();
    let mut current_vertex = edge_list[0].0; // Start with the first vertex
    boundary_vertices.push(current_vertex);

    while boundary_vertices.len() <= edge_list.len() {
        let mut found = false;
        for &(v0, v1) in edge_list {
            if v0 == current_vertex && !boundary_vertices.contains(&v1) {
                current_vertex = v1;
                boundary_vertices.push(current_vertex);
                found = true;
                break;
            } else if v1 == current_vertex && !boundary_vertices.contains(&v0) {
                current_vertex = v0;
                boundary_vertices.push(current_vertex);
                found = true;
                break;
            }
        }
        if !found {
            break; // Break if no next vertex is found
        }
    }

    assert_eq!(boundary_vertices.len(), 112); // Compared with result from C++

    boundary_vertices
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

#[wasm_bindgen]
extern {
    pub fn alert(s: &str);
}

#[wasm_bindgen]
pub fn greet() {
    alert("Hello, py-torch!");
}



#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::HashMap;
    use std::iter::zip;

    fn load_test_mesh() -> Mesh {
        let mesh_cartography_lib_dir_str = env::var("Meshes_Dir").expect("MeshCartographyLib_DIR not set");
        let mesh_cartography_lib_dir = PathBuf::from(mesh_cartography_lib_dir_str);
        let new_path = mesh_cartography_lib_dir.join("ellipsoid_x4_open.obj");
        io::load_obj_mesh(new_path)
    }

    fn count_mesh_degree(surface_mesh: &Mesh) -> HashMap<tri_mesh::VertexID, usize> {
        // Iterate over the connected faces
        let connected_faces = Mesh::connected_components(&surface_mesh); // Vec<HashSet<FaceID>>
        let mut vertex_degree = HashMap::new();

        for face_id in connected_faces[0].iter() {
            let face = surface_mesh.face_vertices(*face_id);

            // Destructure the tuple and increment count for each VertexID
            let (v1, v2, v3) = face;
            *vertex_degree.entry(v1).or_insert(0) += 1;
            *vertex_degree.entry(v2).or_insert(0) += 1;
            *vertex_degree.entry(v3).or_insert(0) += 1;
        }

        vertex_degree
    }

    fn count_open_mesh_degree(surface_mesh: &Mesh, boundary_vertices: &Vec<tri_mesh::VertexID>) -> HashMap<tri_mesh::VertexID, usize> {
        let mut vertex_degree = count_mesh_degree(&surface_mesh);

        // Add +1 for each boundary vertex
        for vertex_id in boundary_vertices.iter() {
            *vertex_degree.entry(*vertex_id).or_insert(0) += 1;
        }

        vertex_degree
    }

    fn rotate_boundary_vertices(boundary_vertices: &mut Vec<tri_mesh::VertexID>, surface_mesh: &Mesh, vertex_degree: &HashMap<tri_mesh::VertexID, usize>) {
        // Rotate the boundary vertices so that the start vertex is at the beginning as in the C++17 code
        let mut start_vertex = surface_mesh.vertex_iter().next().unwrap();
        for vertex_id in boundary_vertices.iter() {
            if vertex_degree.get(&vertex_id) == Some(&7) {
                start_vertex = *vertex_id;
            }
        }

        if let Some(position) = boundary_vertices.iter().position(|&v| v == start_vertex) {
            boundary_vertices.rotate_left(position);
        }
    }

    #[test]
    fn test_mesh_connectivity() {
        let surface_mesh = load_test_mesh();
        let mut length = 0.0;
        let boundary_edges = get_boundary_edges(&surface_mesh, &mut length);
        let edge_list = boundary_edges.iter().cloned().collect::<Vec<_>>();
        let mut boundary_vertices = get_boundary_vertices(&edge_list);

        // Test if the mesh is valid
        assert!(!surface_mesh.is_closed(), "Mesh is not open");

        // Count the degree of each vertex
        let vertex_degree = count_open_mesh_degree(&surface_mesh, &boundary_vertices);

        // Rotate to match the C++17 code
        rotate_boundary_vertices(&mut boundary_vertices, &surface_mesh, &vertex_degree);

        // Neighbors from C++17 code
        let exspected_neighbours = [7, 4, 4, 4, 4, 4, 4, 4, 4, 5, 5, 4, 4, 4, 4, 4, 4, 4, 5, 3, 5, 4, 4, 4, 4, 4, 4, 4,
                        4, 4, 4, 4, 4, 4, 4, 5, 4, 4, 4, 5, 4, 4, 4, 3, 4, 4, 4, 4, 4, 4, 5, 4, 4, 4, 4, 4, 5, 5, 4, 4,
                        4, 5, 3, 4, 4, 4, 4, 4, 5, 4, 5, 4, 4, 3, 5, 4, 5, 3, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4,
                        4, 4, 4, 4, 4, 5, 4, 4, 4, 3, 4, 4, 4, 4, 4, 4, 4, 4, 4];

        // Print the counts for the boundary vertices
        for (vertex_id, expected) in zip(boundary_vertices, exspected_neighbours.iter()) {
            assert_eq!(vertex_degree.get(&vertex_id), Some(expected));
            // println!("{:?}, {:?}", vertex_degree.get(&vertex_id), expected);
        }
    }

    #[test]
    fn test_neighbors_based_on_L_matrix() {
        let surface_mesh = load_test_mesh();
        let mut length = 0.0;
        let boundary_edges = get_boundary_edges(&surface_mesh, &mut length);
        let edge_list = boundary_edges.iter().cloned().collect::<Vec<_>>();
        let mut boundary_vertices = get_boundary_vertices(&edge_list);

        // Count the degree of each vertex
        let vertex_degree = count_open_mesh_degree(&surface_mesh, &boundary_vertices);

        // Get the Laplace matrix L
        let L = SurfaceParametrization::laplacian_matrix::build_laplace_matrix(&surface_mesh, true);

        for vertex_id in surface_mesh.vertex_iter() {
            let index_as_u32: u32 = *vertex_id;
            let index_as_usize: usize = index_as_u32 as usize;
            let expected = vertex_degree.get(&vertex_id).unwrap();

            // -1 because the diagonal entry is not counted as it is the vertex itself
            assert_eq!(L.row(index_as_usize).values().len() - 1, *expected);
        }
    }

    #[test]
    fn test_find_boundary_edges() {
        let surface_mesh = load_test_mesh();

        let mut length = 0.0;
        let boundary_edges = get_boundary_edges(&surface_mesh, &mut length);

        assert!(length > 0.0);
        assert_eq!(boundary_edges.len(), 112);

        for &(v0, v1) in &boundary_edges {
            // println!("v0: {:?}, v1: {:?}", v0, v1);
            assert!(surface_mesh.is_vertex_on_boundary(v0));
            assert!(surface_mesh.is_vertex_on_boundary(v1));
        }
    }

    #[test]
    fn test_find_bourdary_vertices() {
        let surface_mesh = load_test_mesh();

        let mut length = 0.0;
        let boundary_edges = get_boundary_edges(&surface_mesh, &mut length);

        // Collect edges in a Vec to maintain order
        let edge_list = boundary_edges.iter().cloned().collect::<Vec<_>>();

        // Collect the boundary vertices
        let boundary_vertices = get_boundary_vertices(&edge_list);

        assert_eq!(boundary_vertices.len(), 112);

        for vertex_id in boundary_vertices {
            // println!("vertex_id: {:?}", vertex_id);
            assert!(surface_mesh.is_vertex_on_boundary(vertex_id));
        }
    }

    #[test]
    fn test_assign_vertices_to_boundary() {
        let surface_mesh = load_test_mesh();

        let mut length = 0.0;
        let boundary_edges = get_boundary_edges(&surface_mesh, &mut length);
        let edge_list = boundary_edges.iter().cloned().collect::<Vec<_>>();
        let boundary_vertices = get_boundary_vertices(&edge_list);

        let corner_count = 4;
        let side_length = length / corner_count as f64;
        let tolerance = 1e-4;

        let mut mesh_tex_coords = mesh_definition::MeshTexCoords::new(&surface_mesh);

        for vertex_id in surface_mesh.vertex_iter() {
            mesh_tex_coords.set_tex_coord(vertex_id, TexCoord(0.0, 0.0)); // Initialize to the origin
        }

        let tex_coords = distribute_vertices_around_square(&boundary_vertices, side_length, tolerance, length);
        for (&vertex_id, tex_coord) in boundary_vertices.iter().zip(tex_coords.iter()) {
            mesh_tex_coords.set_tex_coord(vertex_id, TexCoord(tex_coord.0, tex_coord.1));
        }

        let vertex_id = surface_mesh.vertex_iter().next().unwrap();
        let tex_coord = mesh_tex_coords.get_tex_coord(vertex_id).unwrap();
        assert_eq!(tex_coord.0, 0.0);
        assert_eq!(tex_coord.1, 0.0);

        let vertex_id = surface_mesh.vertex_iter().nth(3619).unwrap();
        let tex_coord = mesh_tex_coords.get_tex_coord(vertex_id).unwrap();
        assert_eq!(tex_coord.0, 1.0);
        assert_eq!(tex_coord.1, 0.0);

        let vertex_id = surface_mesh.vertex_iter().nth(3129).unwrap();
        let tex_coord = mesh_tex_coords.get_tex_coord(vertex_id).unwrap();
        assert_eq!(tex_coord.0, 0.0);
        assert_eq!(tex_coord.1, 0.75);

        let vertex_id = surface_mesh.vertex_iter().nth(2095).unwrap();
        let tex_coord = mesh_tex_coords.get_tex_coord(vertex_id).unwrap();
        assert_eq!(tex_coord.0, 0.0);
        assert_eq!(tex_coord.1, 1.0);
    }

    #[test]
    #[allow(non_snake_case)]
    fn test_boundary_matrix_B_creation() {
        let surface_mesh = load_test_mesh();
        let mut mesh_tex_coords = create_mocked_mesh_tex_coords();
        let B = SurfaceParametrization::harmonic_parameterization_helper::set_boundary_constraints(&surface_mesh, &mut mesh_tex_coords);

        let mut num_boundary_vertices = 0;
        for i in 0..B.nrows() {
            let row_data: Vec<f64> = B.row(i).iter().cloned().collect();
            if surface_mesh.is_vertex_on_boundary(surface_mesh.vertex_iter().nth(i).unwrap()) {
                num_boundary_vertices += 1;
            } else {
                assert_eq!(row_data, vec![0.0, 0.0]);
            }
        }
        assert_eq!(num_boundary_vertices, 112);
    }

    #[test]
    #[allow(non_snake_case)]
    fn test_harmonic_parameterization() {
        let surface_mesh = load_test_mesh();
        let mut mesh_tex_coords = create_mocked_mesh_tex_coords();
        let B = SurfaceParametrization::harmonic_parameterization_helper::set_boundary_constraints(&surface_mesh, &mut mesh_tex_coords);
        let L = SurfaceParametrization::laplacian_matrix::build_laplace_matrix(&surface_mesh, true);

        // for vertex_id in surface_mesh.vertex_iter() {
        //     // convert vertex_id to usize
        //     let index_as_u32: u32 = *vertex_id;
        //     let index_as_usize: usize = index_as_u32 as usize;
        //     let row_data: Vec<f64> = B.row(index_as_usize).iter().cloned().collect();

        //     if surface_mesh.is_vertex_on_boundary(vertex_id) {
        //         println!("");
        //         println!("L.row({:?}): {:?}", vertex_id, L.row(index_as_usize));
        //         println!("B.row({:?}): {:?}", vertex_id, row_data);
        //         println!("");
        //     } else {
        //         // println!("L.row({:?}): {:?}", vertex_id, L.row(index_as_usize).values());
        //     }
        // }
    }

    fn create_mocked_mesh_tex_coords() -> mesh_definition::MeshTexCoords {
        let surface_mesh = load_test_mesh();
        let mut mesh_tex_coords = mesh_definition::MeshTexCoords::new(&surface_mesh);

        for vertex_id in surface_mesh.vertex_iter() {
            mesh_tex_coords.set_tex_coord(vertex_id, TexCoord(0.0, 0.0)); // Initialize to the origin
        }

        // Insert mocked data
        mesh_tex_coords.set_tex_coord(surface_mesh.vertex_iter().nth(39).unwrap(), TexCoord(0.0, 0.0));
        mesh_tex_coords.set_tex_coord(surface_mesh.vertex_iter().nth(37).unwrap(), TexCoord(0.03571428571428571, 0.0));
        mesh_tex_coords.set_tex_coord(surface_mesh.vertex_iter().nth(92).unwrap(), TexCoord(0.07142857142857142, 0.0));
        mesh_tex_coords.set_tex_coord(surface_mesh.vertex_iter().nth(90).unwrap(), TexCoord(0.10714285714285715, 0.0));
        mesh_tex_coords.set_tex_coord(surface_mesh.vertex_iter().nth(91).unwrap(), TexCoord(0.14285714285714285, 0.0));
        mesh_tex_coords.set_tex_coord(surface_mesh.vertex_iter().nth(4389).unwrap(), TexCoord(0.17857142857142855, 0.0));
        mesh_tex_coords.set_tex_coord(surface_mesh.vertex_iter().nth(4520).unwrap(), TexCoord(0.2142857142857143, 0.0));
        mesh_tex_coords.set_tex_coord(surface_mesh.vertex_iter().nth(4519).unwrap(), TexCoord(0.25, 0.0));
        mesh_tex_coords.set_tex_coord(surface_mesh.vertex_iter().nth(4568).unwrap(), TexCoord(0.2857142857142857, 0.0));
        mesh_tex_coords.set_tex_coord(surface_mesh.vertex_iter().nth(4567).unwrap(), TexCoord(0.3214285714285714, 0.0));
        mesh_tex_coords.set_tex_coord(surface_mesh.vertex_iter().nth(4132).unwrap(), TexCoord(0.3571428571428571, 0.0));
        mesh_tex_coords.set_tex_coord(surface_mesh.vertex_iter().nth(4189).unwrap(), TexCoord(0.39285714285714285, 0.0));
        mesh_tex_coords.set_tex_coord(surface_mesh.vertex_iter().nth(93).unwrap(), TexCoord(0.4285714285714286, 0.0));
        mesh_tex_coords.set_tex_coord(surface_mesh.vertex_iter().nth(95).unwrap(), TexCoord(0.46428571428571425, 0.0));
        mesh_tex_coords.set_tex_coord(surface_mesh.vertex_iter().nth(97).unwrap(), TexCoord(0.5, 0.0));
        mesh_tex_coords.set_tex_coord(surface_mesh.vertex_iter().nth(99).unwrap(), TexCoord(0.5357142857142857, 0.0));
        mesh_tex_coords.set_tex_coord(surface_mesh.vertex_iter().nth(101).unwrap(), TexCoord(0.5714285714285714, 0.0));
        mesh_tex_coords.set_tex_coord(surface_mesh.vertex_iter().nth(103).unwrap(), TexCoord(0.6071428571428571, 0.0));
        mesh_tex_coords.set_tex_coord(surface_mesh.vertex_iter().nth(105).unwrap(), TexCoord(0.6428571428571428, 0.0));
        mesh_tex_coords.set_tex_coord(surface_mesh.vertex_iter().nth(145).unwrap(), TexCoord(0.6785714285714286, 0.0));
        mesh_tex_coords.set_tex_coord(surface_mesh.vertex_iter().nth(146).unwrap(), TexCoord(0.7142857142857142, 0.0));
        mesh_tex_coords.set_tex_coord(surface_mesh.vertex_iter().nth(111).unwrap(), TexCoord(0.75, 0.0));
        mesh_tex_coords.set_tex_coord(surface_mesh.vertex_iter().nth(110).unwrap(), TexCoord(0.7857142857142857, 0.0));
        mesh_tex_coords.set_tex_coord(surface_mesh.vertex_iter().nth(55).unwrap(), TexCoord(0.8214285714285714, 0.0));
        mesh_tex_coords.set_tex_coord(surface_mesh.vertex_iter().nth(57).unwrap(), TexCoord(0.8571428571428572, 0.0));
        mesh_tex_coords.set_tex_coord(surface_mesh.vertex_iter().nth(59).unwrap(), TexCoord(0.8928571428571429, 0.0));
        mesh_tex_coords.set_tex_coord(surface_mesh.vertex_iter().nth(61).unwrap(), TexCoord(0.9285714285714285, 0.0));
        mesh_tex_coords.set_tex_coord(surface_mesh.vertex_iter().nth(3618).unwrap(), TexCoord(0.9642857142857142, 0.0));
        mesh_tex_coords.set_tex_coord(surface_mesh.vertex_iter().nth(3619).unwrap(), TexCoord(1.0, 0.0));
        mesh_tex_coords.set_tex_coord(surface_mesh.vertex_iter().nth(3964).unwrap(), TexCoord(1.0, 0.035714285714285664));
        mesh_tex_coords.set_tex_coord(surface_mesh.vertex_iter().nth(3649).unwrap(), TexCoord(1.0, 0.07142857142857133));
        mesh_tex_coords.set_tex_coord(surface_mesh.vertex_iter().nth(3585).unwrap(), TexCoord(1.0, 0.10714285714285715));
        mesh_tex_coords.set_tex_coord(surface_mesh.vertex_iter().nth(3586).unwrap(), TexCoord(1.0, 0.14285714285714282));
        mesh_tex_coords.set_tex_coord(surface_mesh.vertex_iter().nth(3953).unwrap(), TexCoord(1.0, 0.17857142857142846));
        mesh_tex_coords.set_tex_coord(surface_mesh.vertex_iter().nth(3952).unwrap(), TexCoord(1.0, 0.2142857142857143));
        mesh_tex_coords.set_tex_coord(surface_mesh.vertex_iter().nth(3194).unwrap(), TexCoord(1.0, 0.24999999999999994));
        mesh_tex_coords.set_tex_coord(surface_mesh.vertex_iter().nth(3004).unwrap(), TexCoord(1.0, 0.28571428571428564));
        mesh_tex_coords.set_tex_coord(surface_mesh.vertex_iter().nth(2568).unwrap(), TexCoord(1.0, 0.32142857142857145));
        mesh_tex_coords.set_tex_coord(surface_mesh.vertex_iter().nth(2318).unwrap(), TexCoord(1.0, 0.3571428571428571));
        mesh_tex_coords.set_tex_coord(surface_mesh.vertex_iter().nth(2265).unwrap(), TexCoord(1.0, 0.3928571428571428));
        mesh_tex_coords.set_tex_coord(surface_mesh.vertex_iter().nth(2148).unwrap(), TexCoord(1.0, 0.42857142857142844));
        mesh_tex_coords.set_tex_coord(surface_mesh.vertex_iter().nth(3860).unwrap(), TexCoord(1.0, 0.46428571428571425));
        mesh_tex_coords.set_tex_coord(surface_mesh.vertex_iter().nth(3848).unwrap(), TexCoord(1.0, 0.4999999999999999));
        mesh_tex_coords.set_tex_coord(surface_mesh.vertex_iter().nth(3851).unwrap(), TexCoord(1.0, 0.5357142857142857));
        mesh_tex_coords.set_tex_coord(surface_mesh.vertex_iter().nth(1940).unwrap(), TexCoord(1.0, 0.5714285714285714));
        mesh_tex_coords.set_tex_coord(surface_mesh.vertex_iter().nth(1941).unwrap(), TexCoord(1.0, 0.6071428571428571));
        mesh_tex_coords.set_tex_coord(surface_mesh.vertex_iter().nth(3832).unwrap(), TexCoord(1.0, 0.6428571428571427));
        mesh_tex_coords.set_tex_coord(surface_mesh.vertex_iter().nth(1814).unwrap(), TexCoord(1.0, 0.6785714285714284));
        mesh_tex_coords.set_tex_coord(surface_mesh.vertex_iter().nth(1813).unwrap(), TexCoord(1.0, 0.7142857142857144));
        mesh_tex_coords.set_tex_coord(surface_mesh.vertex_iter().nth(1812).unwrap(), TexCoord(1.0, 0.75));
        mesh_tex_coords.set_tex_coord(surface_mesh.vertex_iter().nth(1846).unwrap(), TexCoord(1.0, 0.7857142857142857));
        mesh_tex_coords.set_tex_coord(surface_mesh.vertex_iter().nth(1878).unwrap(), TexCoord(1.0, 0.8214285714285714));
        mesh_tex_coords.set_tex_coord(surface_mesh.vertex_iter().nth(1931).unwrap(), TexCoord(1.0, 0.857142857142857));
        mesh_tex_coords.set_tex_coord(surface_mesh.vertex_iter().nth(1930).unwrap(), TexCoord(1.0, 0.8928571428571427));
        mesh_tex_coords.set_tex_coord(surface_mesh.vertex_iter().nth(1929).unwrap(), TexCoord(1.0, 0.9285714285714284));
        mesh_tex_coords.set_tex_coord(surface_mesh.vertex_iter().nth(1874).unwrap(), TexCoord(1.0, 0.9642857142857143));
        mesh_tex_coords.set_tex_coord(surface_mesh.vertex_iter().nth(1873).unwrap(), TexCoord(1.0, 1.0));
        mesh_tex_coords.set_tex_coord(surface_mesh.vertex_iter().nth(1872).unwrap(), TexCoord(0.9642857142857143, 1.0));
        mesh_tex_coords.set_tex_coord(surface_mesh.vertex_iter().nth(1871).unwrap(), TexCoord(0.9285714285714287, 1.0));
        mesh_tex_coords.set_tex_coord(surface_mesh.vertex_iter().nth(1870).unwrap(), TexCoord(0.892857142857143, 1.0));
        mesh_tex_coords.set_tex_coord(surface_mesh.vertex_iter().nth(1868).unwrap(), TexCoord(0.8571428571428573, 1.0));
        mesh_tex_coords.set_tex_coord(surface_mesh.vertex_iter().nth(1869).unwrap(), TexCoord(0.8214285714285714, 1.0));
        mesh_tex_coords.set_tex_coord(surface_mesh.vertex_iter().nth(4079).unwrap(), TexCoord(0.7857142857142857, 1.0));
        mesh_tex_coords.set_tex_coord(surface_mesh.vertex_iter().nth(1913).unwrap(), TexCoord(0.75, 1.0));
        mesh_tex_coords.set_tex_coord(surface_mesh.vertex_iter().nth(3840).unwrap(), TexCoord(0.7142857142857144, 1.0));
        mesh_tex_coords.set_tex_coord(surface_mesh.vertex_iter().nth(3839).unwrap(), TexCoord(0.6785714285714287, 1.0));
        mesh_tex_coords.set_tex_coord(surface_mesh.vertex_iter().nth(3842).unwrap(), TexCoord(0.642857142857143, 1.0));
        mesh_tex_coords.set_tex_coord(surface_mesh.vertex_iter().nth(1919).unwrap(), TexCoord(0.6071428571428574, 1.0));
        mesh_tex_coords.set_tex_coord(surface_mesh.vertex_iter().nth(1922).unwrap(), TexCoord(0.5714285714285714, 1.0));
        mesh_tex_coords.set_tex_coord(surface_mesh.vertex_iter().nth(1924).unwrap(), TexCoord(0.5357142857142857, 1.0));
        mesh_tex_coords.set_tex_coord(surface_mesh.vertex_iter().nth(1926).unwrap(), TexCoord(0.5000000000000001, 1.0));
        mesh_tex_coords.set_tex_coord(surface_mesh.vertex_iter().nth(1928).unwrap(), TexCoord(0.4642857142857144, 1.0));
        mesh_tex_coords.set_tex_coord(surface_mesh.vertex_iter().nth(1962).unwrap(), TexCoord(0.42857142857142877, 1.0));
        mesh_tex_coords.set_tex_coord(surface_mesh.vertex_iter().nth(1963).unwrap(), TexCoord(0.3928571428571431, 1.0));
        mesh_tex_coords.set_tex_coord(surface_mesh.vertex_iter().nth(1933).unwrap(), TexCoord(0.3571428571428571, 1.0));
        mesh_tex_coords.set_tex_coord(surface_mesh.vertex_iter().nth(1881).unwrap(), TexCoord(0.32142857142857145, 1.0));
        mesh_tex_coords.set_tex_coord(surface_mesh.vertex_iter().nth(1880).unwrap(), TexCoord(0.2857142857142858, 1.0));
        mesh_tex_coords.set_tex_coord(surface_mesh.vertex_iter().nth(1882).unwrap(), TexCoord(0.2500000000000001, 1.0));
        mesh_tex_coords.set_tex_coord(surface_mesh.vertex_iter().nth(1884).unwrap(), TexCoord(0.21428571428571447, 1.0));
        mesh_tex_coords.set_tex_coord(surface_mesh.vertex_iter().nth(1886).unwrap(), TexCoord(0.1785714285714288, 1.0));
        mesh_tex_coords.set_tex_coord(surface_mesh.vertex_iter().nth(1938).unwrap(), TexCoord(0.14285714285714315, 1.0));
        mesh_tex_coords.set_tex_coord(surface_mesh.vertex_iter().nth(1939).unwrap(), TexCoord(0.10714285714285715, 1.0));
        mesh_tex_coords.set_tex_coord(surface_mesh.vertex_iter().nth(3847).unwrap(), TexCoord(0.0714285714285715, 1.0));
        mesh_tex_coords.set_tex_coord(surface_mesh.vertex_iter().nth(2094).unwrap(), TexCoord(0.03571428571428583, 1.0));
        mesh_tex_coords.set_tex_coord(surface_mesh.vertex_iter().nth(2095).unwrap(), TexCoord(0.0, 1.0));
        mesh_tex_coords.set_tex_coord(surface_mesh.vertex_iter().nth(2205).unwrap(), TexCoord(0.0, 0.9642857142857146));
        mesh_tex_coords.set_tex_coord(surface_mesh.vertex_iter().nth(2204).unwrap(), TexCoord(0.0, 0.9285714285714287));
        mesh_tex_coords.set_tex_coord(surface_mesh.vertex_iter().nth(2317).unwrap(), TexCoord(0.0, 0.8928571428571433));
        mesh_tex_coords.set_tex_coord(surface_mesh.vertex_iter().nth(2516).unwrap(), TexCoord(0.0, 0.8571428571428573));
        mesh_tex_coords.set_tex_coord(surface_mesh.vertex_iter().nth(2662).unwrap(), TexCoord(0.0, 0.8214285714285721));
        mesh_tex_coords.set_tex_coord(surface_mesh.vertex_iter().nth(2742).unwrap(), TexCoord(0.0, 0.785714285714286));
        mesh_tex_coords.set_tex_coord(surface_mesh.vertex_iter().nth(3129).unwrap(), TexCoord(0.0, 0.75));
        mesh_tex_coords.set_tex_coord(surface_mesh.vertex_iter().nth(3220).unwrap(), TexCoord(0.0, 0.7142857142857147));
        mesh_tex_coords.set_tex_coord(surface_mesh.vertex_iter().nth(3276).unwrap(), TexCoord(0.0, 0.6785714285714287));
        mesh_tex_coords.set_tex_coord(surface_mesh.vertex_iter().nth(3489).unwrap(), TexCoord(0.0, 0.6428571428571433));
        mesh_tex_coords.set_tex_coord(surface_mesh.vertex_iter().nth(3555).unwrap(), TexCoord(0.0, 0.6071428571428574));
        mesh_tex_coords.set_tex_coord(surface_mesh.vertex_iter().nth(3554).unwrap(), TexCoord(0.0, 0.5714285714285714));
        mesh_tex_coords.set_tex_coord(surface_mesh.vertex_iter().nth(3553).unwrap(), TexCoord(0.0, 0.535714285714286));
        mesh_tex_coords.set_tex_coord(surface_mesh.vertex_iter().nth(3584).unwrap(), TexCoord(0.0, 0.5000000000000001));
        mesh_tex_coords.set_tex_coord(surface_mesh.vertex_iter().nth(3965).unwrap(), TexCoord(0.0, 0.46428571428571475));
        mesh_tex_coords.set_tex_coord(surface_mesh.vertex_iter().nth(3970).unwrap(), TexCoord(0.0, 0.42857142857142877));
        mesh_tex_coords.set_tex_coord(surface_mesh.vertex_iter().nth(3648).unwrap(), TexCoord(0.0, 0.39285714285714346));
        mesh_tex_coords.set_tex_coord(surface_mesh.vertex_iter().nth(3647).unwrap(), TexCoord(0.0, 0.35714285714285743));
        mesh_tex_coords.set_tex_coord(surface_mesh.vertex_iter().nth(3650).unwrap(), TexCoord(0.0, 0.32142857142857145));
        mesh_tex_coords.set_tex_coord(surface_mesh.vertex_iter().nth(53).unwrap(), TexCoord(0.0, 0.28571428571428614));
        mesh_tex_coords.set_tex_coord(surface_mesh.vertex_iter().nth(108).unwrap(), TexCoord(0.0, 0.2500000000000001));
        mesh_tex_coords.set_tex_coord(surface_mesh.vertex_iter().nth(107).unwrap(), TexCoord(0.0, 0.2142857142857148));
        mesh_tex_coords.set_tex_coord(surface_mesh.vertex_iter().nth(106).unwrap(), TexCoord(0.0, 0.1785714285714288));
        mesh_tex_coords.set_tex_coord(surface_mesh.vertex_iter().nth(47).unwrap(), TexCoord(0.0, 0.1428571428571435));
        mesh_tex_coords.set_tex_coord(surface_mesh.vertex_iter().nth(45).unwrap(), TexCoord(0.0, 0.10714285714285748));
        mesh_tex_coords.set_tex_coord(surface_mesh.vertex_iter().nth(43).unwrap(), TexCoord(0.0, 0.0714285714285715));
        mesh_tex_coords.set_tex_coord(surface_mesh.vertex_iter().nth(41).unwrap(), TexCoord(0.0, 0.03571428571428616));

        mesh_tex_coords
    }
}
