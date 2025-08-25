fn main() {
    // Minimal smoke test with a single triangle mesh
    let vertices = vec![[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]];
    let faces = vec![[0u32, 1, 2]];
    let source = [0.05, 0.05, 0.0];
    let goals = vec![[0.9, 0.1, 0.0], [0.1, 0.9, 0.0]];

    match rust_cgal_shortest_paths::shortest_paths(&vertices, &faces, source, &goals) {
        Ok(paths) => {
            for (i, p) in paths.iter().enumerate() {
                println!("Path {} has {} points", i, p.len());
            }
        }
        Err(e) => eprintln!("Error: {}", e),
    }
}
