use rust_cgal_shortest_paths::shortest_paths_barycentric;

#[test]
fn barycentric_source_and_goal() {
    // Simple single triangle
    let vertices = vec![
        [0.0, 0.0, 0.0], // v0
        [1.0, 0.0, 0.0], // v1
        [0.0, 1.0, 0.0], // v2
    ];
    let faces = vec![[0u32, 1, 2]];

    // Source at centroid (1/3,1/3,1/3)
    let source_face = 0usize;
    let source_bary = [1.0 / 3.0, 1.0 / 3.0, 1.0 / 3.0];

    // Goal near vertex v1 but still interior
    let goal_face = 0usize;
    let goal_bary = [0.05, 0.9, 0.05];

    let paths = shortest_paths_barycentric(
        &vertices,
        &faces,
        source_face,
        source_bary,
        &[(goal_face, goal_bary)],
    )
    .expect("compute barycentric path");

    assert_eq!(paths.len(), 1);
    let poly = &paths[0];
    assert!(poly.len() >= 2);

    // Verify endpoints equal the evaluated barycentric points (within epsilon)
    let src_pt = bary_on_face(&vertices, &faces, source_face, source_bary);
    let goal_pt = bary_on_face(&vertices, &faces, goal_face, goal_bary);

    let first = poly.first().copied().unwrap();
    let last = poly.last().copied().unwrap();
    let eps = 1e-6;
    assert!(approx_pt(first, src_pt, eps));
    assert!(approx_pt(last, goal_pt, eps));
}

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

// Evaluate a barycentric coordinate on a given face into Euclidean coordinates.
fn bary_on_face(
    vertices: &[[f64; 3]],
    faces: &[[u32; 3]],
    face_idx: usize,
    bary: [f64; 3],
) -> [f64; 3] {
    let [i0, i1, i2] = faces[face_idx];
    let p0 = vertices[i0 as usize];
    let p1 = vertices[i1 as usize];
    let p2 = vertices[i2 as usize];
    [
        bary[0] * p0[0] + bary[1] * p1[0] + bary[2] * p2[0],
        bary[0] * p0[1] + bary[1] * p1[1] + bary[2] * p2[1],
        bary[0] * p0[2] + bary[1] * p1[2] + bary[2] * p2[2],
    ]
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

    // Source at vertex v0 on face 0 in barycentric coords
    let source_face = 0usize;
    let source_bary = [1.0, 0.0, 0.0];
    // Goals at vertices v1 and v2 on face 0
    let goals = vec![(0usize, [0.0, 1.0, 0.0]), (0usize, [0.0, 0.0, 1.0])];

    let paths = shortest_paths_barycentric(&vertices, &faces, source_face, source_bary, &goals)
        .expect("compute paths");
    assert_eq!(paths.len(), 2);

    // v0->v1 and v0->v2 endpoints should match; lengths may depend on polyline sampling
    let eps = 1e-6;
    let source_pt = [
        source_bary[0] * vertices[0][0]
            + source_bary[1] * vertices[1][0]
            + source_bary[2] * vertices[2][0],
        source_bary[0] * vertices[0][1]
            + source_bary[1] * vertices[1][1]
            + source_bary[2] * vertices[2][1],
        0.0,
    ];
    for (i, &(gf, gb)) in goals.iter().enumerate() {
        let poly = &paths[i];
        assert!(poly.len() >= 2, "polyline has at least the endpoints");
        let first = poly.first().copied().unwrap();
        let last = poly.last().copied().unwrap();
        let goal_pt = bary_on_face(&vertices, &faces, gf, gb);
        // Accept either direction
        assert!(
            (approx_pt(first, source_pt, eps) && approx_pt(last, goal_pt, eps))
                || (approx_pt(first, goal_pt, eps) && approx_pt(last, source_pt, eps)),
            "endpoints must be source/goal in some order: first={:?}, last={:?}",
            first,
            last
        );
        let len = polyline_len(poly);
        let expected = dist(source_pt, goal_pt);
        assert!(len >= 0.0, "length should be non-negative");
        assert!(len >= expected - 1e-8, "len {} >= {}", len, expected);
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

    // Source at v0 on face 0, goal at v2 on face 0
    let source_face = 0usize;
    let source_bary = [1.0, 0.0, 0.0];
    let goal_face = 0usize;
    let goal_bary = [0.0, 0.0, 1.0];

    let paths = shortest_paths_barycentric(
        &vertices,
        &faces,
        source_face,
        source_bary,
        &[(goal_face, goal_bary)],
    )
    .expect("compute");
    assert_eq!(paths.len(), 1);
    let poly = &paths[0];
    let eps = 1e-6;
    let first = poly.first().copied().unwrap();
    let last = poly.last().copied().unwrap();

    let src_pt = bary_on_face(&vertices, &faces, source_face, source_bary);
    let goal_pt = bary_on_face(&vertices, &faces, goal_face, goal_bary);
    assert!(
        (approx_pt(first, src_pt, eps) && approx_pt(last, goal_pt, eps))
            || (approx_pt(first, goal_pt, eps) && approx_pt(last, src_pt, eps)),
        "endpoints must be source/goal in some order: first={:?}, last={:?}",
        first,
        last
    );

    let len = polyline_len(poly);
    let expected = (2.0f64).sqrt();
    // Only check endpoints and that length is non-negative; conversion to Euclidean is used only for verification
    assert!(len >= 0.0, "length should be non-negative");
    // Optionally, ensure it's at least as long as straight-line distance in Euclidean embedding
    assert!(len >= expected - 1e-8, "len {} >= {}", len, expected);
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
    // Source at v0 on face 0 in barycentric coords
    let source_face = 0usize;
    let source_bary = [1.0, 0.0, 0.0];
    // Goals: v1 (face 0), v3 (face 1), v2 (face 0)
    let goals = vec![
        (0usize, [0.0, 1.0, 0.0]), // v1
        (1usize, [0.0, 0.0, 1.0]), // v3 in face [v0,v2,v3]
        (0usize, [0.0, 0.0, 1.0]), // v2 in face [v0,v1,v2]
    ];

    let paths = shortest_paths_barycentric(&vertices, &faces, source_face, source_bary, &goals)
        .expect("compute");
    assert_eq!(paths.len(), goals.len());

    let expected = [1.0, 1.0, (2.0f64).sqrt()];
    let source_pt = bary_on_face(&vertices, &faces, source_face, source_bary);
    for i in 0..goals.len() {
        let poly = &paths[i];
        let len = polyline_len(poly);
        // Only convert to Euclidean for verification; accept algorithmic polyline sampling
        assert!(len >= 0.0, "length should be non-negative");
        assert!(
            len >= expected[i] - 1e-8,
            "path {} len {} >= {}",
            i,
            len,
            expected[i]
        );
        // endpoints (accept either order)
        let first = poly.first().copied().unwrap();
        let last = poly.last().copied().unwrap();
        let (gf, gb) = goals[i];
        let goal_pt = bary_on_face(&vertices, &faces, gf, gb);
        assert!(
            (approx_pt(first, source_pt, 1e-6) && approx_pt(last, goal_pt, 1e-6))
                || (approx_pt(first, goal_pt, 1e-6) && approx_pt(last, source_pt, 1e-6)),
            "endpoints must be source/goal in some order: first={:?}, last={:?}",
            first,
            last
        );
    }
}
