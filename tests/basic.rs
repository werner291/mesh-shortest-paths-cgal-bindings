use rust_cgal_shortest_paths::shortest_paths;

fn dist(a: [f64; 3], b: [f64; 3]) -> f64 {
    let dx = a[0] - b[0];
    let dy = a[1] - b[1];
    let dz = a[2] - b[2];
    (dx * dx + dy * dy + dz * dz).sqrt()
}

fn polyline_len(poly: &[[f64; 3]]) -> f64 {
    if poly.is_empty() {
        return 0.0;
    }
    let mut s = 0.0;
    for i in 1..poly.len() {
        s += dist(poly[i - 1], poly[i]);
    }
    s
}

fn approx_eq(a: f64, b: f64, eps: f64) -> bool {
    (a - b).abs() <= eps
}

fn approx_pt(a: [f64; 3], b: [f64; 3], eps: f64) -> bool {
    dist(a, b) <= eps
}

#[test]
fn triangle_vertex_to_vertices() {
    // Right triangle with unit legs
    let vertices = vec![
        [0.0, 0.0, 0.0], // v0
        [1.0, 0.0, 0.0], // v1
        [0.0, 1.0, 0.0], // v2
    ];
    let faces = vec![[0u32, 1, 2]];
    let source = vertices[0]; // exactly on v0 so nearest-vertex mapping is exact
    let goals = vec![vertices[1], vertices[2]];

    let paths = shortest_paths(&vertices, &faces, source, &goals).expect("compute paths");
    assert_eq!(paths.len(), 2);

    // v0->v1 should be length 1 along the edge; same for v0->v2
    let eps = 1e-6;
    for (i, &goal) in goals.iter().enumerate() {
        let poly = &paths[i];
        assert!(poly.len() >= 2, "polyline has at least the endpoints");
        println!(
            "first={:?} source={:?}",
            poly.first().copied().unwrap(),
            source
        );
        let first = poly.first().copied().unwrap();
        let last = poly.last().copied().unwrap();
        // Accept either direction
        assert!(
            (approx_pt(first, source, eps) && approx_pt(last, goal, eps))
                || (approx_pt(first, goal, eps) && approx_pt(last, source, eps)),
            "endpoints must be source/goal in some order: first={:?}, last={:?}",
            first,
            last
        );
        let len = polyline_len(poly);
        let expected = dist(source, goal);
        assert!(
            approx_eq(len, expected, 1e-8),
            "len {} ~= {}",
            len,
            expected
        );
    }
}

#[test]
fn square_diagonal_across_faces() {
    // Unit square in the XY plane triangulated into two triangles
    //  v3(0,1) ---- v2(1,1)
    //    |        / |
    //    |      /   |
    //    |    /     |
    //  v0(0,0) ---- v1(1,0)
    // Triangles: [v0,v1,v2] and [v0,v2,v3]
    let vertices = vec![
        [0.0, 0.0, 0.0], // v0
        [1.0, 0.0, 0.0], // v1
        [1.0, 1.0, 0.0], // v2
        [0.0, 1.0, 0.0], // v3
    ];
    let faces = vec![[0u32, 1, 2], [0, 2, 3]];

    let source = vertices[0]; // v0
    let goal = vertices[2]; // v2 opposite corner

    let paths = shortest_paths(&vertices, &faces, source, &[goal]).expect("compute");
    assert_eq!(paths.len(), 1);
    let poly = &paths[0];
    let eps = 1e-6;
    let first = poly.first().copied().unwrap();
    let last = poly.last().copied().unwrap();
    assert!(
        (approx_pt(first, source, eps) && approx_pt(last, goal, eps))
            || (approx_pt(first, goal, eps) && approx_pt(last, source, eps)),
        "endpoints must be source/goal in some order: first={:?}, last={:?}",
        first,
        last
    );

    let len = polyline_len(poly);
    let expected = (2.0f64).sqrt();
    // On a flat square, the geodesic equals Euclidean straight line across faces
    assert!(
        approx_eq(len, expected, 1e-8),
        "len {} ~= {}",
        len,
        expected
    );
}

#[test]
fn square_multiple_goals_lengths() {
    let vertices = vec![
        [0.0, 0.0, 0.0], // v0
        [1.0, 0.0, 0.0], // v1
        [1.0, 1.0, 0.0], // v2
        [0.0, 1.0, 0.0], // v3
    ];
    let faces = vec![[0u32, 1, 2], [0, 2, 3]];
    let source = vertices[0];
    let goals = vec![vertices[1], vertices[3], vertices[2]]; // right, up, diagonal

    let paths = shortest_paths(&vertices, &faces, source, &goals).expect("compute");
    assert_eq!(paths.len(), goals.len());

    let expected = [1.0, 1.0, (2.0f64).sqrt()];
    for i in 0..goals.len() {
        let poly = &paths[i];
        let len = polyline_len(poly);
        assert!(
            approx_eq(len, expected[i], 1e-8),
            "path {} len {} ~= {}",
            i,
            len,
            expected[i]
        );
        // endpoints (accept either order)
        let first = poly.first().copied().unwrap();
        let last = poly.last().copied().unwrap();
        assert!(
            (approx_pt(first, source, 1e-6) && approx_pt(last, goals[i], 1e-6))
                || (approx_pt(first, goals[i], 1e-6) && approx_pt(last, source, 1e-6)),
            "endpoints must be source/goal in some order: first={:?}, last={:?}",
            first,
            last
        );
    }
}
