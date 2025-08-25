fn main() {
    // Minimal smoke test with a single triangle mesh using barycentric inputs
    let vertices = vec![[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]];
    let faces = vec![[0u32, 1, 2]];

    // Source at centroid in face 0; goals near v1 and v2 inside the face
    let source_face = 0usize;
    let source_bary = [1.0 / 3.0, 1.0 / 3.0, 1.0 / 3.0];
    let goals = vec![
        rust_cgal_shortest_paths::FaceBary {
            face: 0usize,
            bary: [0.8, 0.1, 0.1],
        },
        rust_cgal_shortest_paths::FaceBary {
            face: 0usize,
            bary: [0.1, 0.1, 0.8],
        },
    ];
    let source = rust_cgal_shortest_paths::FaceBary {
        face: source_face,
        bary: source_bary,
    };

    match rust_cgal_shortest_paths::shortest_paths_barycentric(&vertices, &faces, source, &goals) {
        Ok(paths) => {
            for (i, p) in paths.iter().enumerate() {
                println!("Path {} has {} points", i, p.len());
            }
        }
        Err(e) => eprintln!("Error: {}", e),
    }
}
