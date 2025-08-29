use rust_cgal_shortest_paths::{FaceBary, FaceTraversal, shortest_paths_barycentric};

fn dist(a: [f64; 3], b: [f64; 3]) -> f64 {
    let dx = a[0] - b[0];
    let dy = a[1] - b[1];
    let dz = a[2] - b[2];
    (dx * dx + dy * dy + dz * dz).sqrt()
}

fn approx(a: [f64; 3], b: [f64; 3], eps: f64) -> bool {
    dist(a, b) <= eps
}

fn grid_mesh(n: usize) -> (Vec<[f64; 3]>, Vec<[u32; 3]>) {
    assert!(n >= 2);
    let mut vertices = Vec::with_capacity(n * n);
    for j in 0..n {
        for i in 0..n {
            let x = i as f64 / (n as f64 - 1.0);
            let y = j as f64 / (n as f64 - 1.0);
            vertices.push([x, y, 0.0]);
        }
    }
    let idx = |i: usize, j: usize| -> u32 { (j * n + i) as u32 };
    let mut faces = Vec::with_capacity((n - 1) * (n - 1) * 2);
    for j in 0..(n - 1) {
        for i in 0..(n - 1) {
            let v00 = idx(i, j);
            let v10 = idx(i + 1, j);
            let v01 = idx(i, j + 1);
            let v11 = idx(i + 1, j + 1);
            // Consistent diagonal from (i,j) to (i+1,j+1)
            faces.push([v00, v10, v11]);
            faces.push([v00, v11, v01]);
        }
    }
    (vertices, faces)
}

fn bary_on_face(verts: &[[f64; 3]], faces: &[[u32; 3]], f: usize, b: [f64; 3]) -> [f64; 3] {
    let [i0, i1, i2] = faces[f];
    let a = verts[i0 as usize];
    let c = verts[i1 as usize];
    let d = verts[i2 as usize];
    [
        b[0] * a[0] + b[1] * c[0] + b[2] * d[0],
        b[0] * a[1] + b[1] * c[1] + b[2] * d[1],
        b[0] * a[2] + b[1] * c[2] + b[2] * d[2],
    ]
}

fn bary_poly_to_euclid(
    vertices: &[[f64; 3]],
    faces: &[[u32; 3]],
    travs: &[FaceTraversal],
) -> Vec<[f64; 3]> {
    let mut out = Vec::new();
    if travs.is_empty() {
        return out;
    }
    for (k, t) in travs.iter().enumerate() {
        let [i0, i1, i2] = faces[t.face];
        let a = vertices[i0 as usize];
        let b = vertices[i1 as usize];
        let c = vertices[i2 as usize];
        let e = [
            t.entry[0] * a[0] + t.entry[1] * b[0] + t.entry[2] * c[0],
            t.entry[0] * a[1] + t.entry[1] * b[1] + t.entry[2] * c[1],
            t.entry[0] * a[2] + t.entry[1] * b[2] + t.entry[2] * c[2],
        ];
        out.push(e);
        if k + 1 == travs.len() {
            let x = [
                t.exit[0] * a[0] + t.exit[1] * b[0] + t.exit[2] * c[0],
                t.exit[0] * a[1] + t.exit[1] * b[1] + t.exit[2] * c[1],
                t.exit[0] * a[2] + t.exit[1] * b[2] + t.exit[2] * c[2],
            ];
            out.push(x);
        }
    }
    out
}

#[test]
fn flat_grid_paths_are_straight() {
    let n = 5;
    let (vertices, faces) = grid_mesh(n);

    // pick a source on the bottom edge of the first triangle (face 0), so the geodesic is along a straight edge
    let source = FaceBary {
        face: 0usize,
        bary: [0.8, 0.2, 0.0],
    }; // on segment v00-v10 at y=0

    // goals: a few other points on the same bottom edge within the same face 0
    let goals = vec![
        FaceBary {
            face: 0,
            bary: [0.9, 0.1, 0.0],
        },
        FaceBary {
            face: 0,
            bary: [0.6, 0.4, 0.0],
        },
        FaceBary {
            face: 0,
            bary: [0.2, 0.8, 0.0],
        },
    ];

    let paths = shortest_paths_barycentric(&vertices, &faces, source, &goals)
        .expect("compute barycentric paths on flat grid");
    assert_eq!(paths.len(), goals.len());

    // Convert to Euclidean for verification of straightness and endpoints
    for (i, poly) in paths.iter().enumerate() {
        // Validate consecutive traversals continuity (accept either global direction)
        for k in 1..poly.len() {
            let prev = poly[k - 1];
            let next = poly[k];
            let p_prev_exit = bary_on_face(&vertices, &faces, prev.face, prev.exit);
            let p_prev_entry = bary_on_face(&vertices, &faces, prev.face, prev.entry);
            let p_next_entry = bary_on_face(&vertices, &faces, next.face, next.entry);
            let p_next_exit = bary_on_face(&vertices, &faces, next.face, next.exit);
            let fwd = approx(p_prev_exit, p_next_entry, 1e-9);
            let bwd = approx(p_prev_entry, p_next_exit, 1e-9);
            if !(fwd || bwd) {
                // Allow degenerate-at-endpoint pairs (library may only emit endpoints)
                let deg_prev = approx(p_prev_entry, p_prev_exit, 1e-12);
                let deg_next = approx(p_next_entry, p_next_exit, 1e-12);
                let spt = bary_on_face(&vertices, &faces, source.face, source.bary);
                let gpt = bary_on_face(&vertices, &faces, goals[i].face, goals[i].bary);
                let prev_is_src =
                    approx(p_prev_entry, spt, 1e-12) && approx(p_prev_exit, spt, 1e-12);
                let next_is_goal =
                    approx(p_next_entry, gpt, 1e-12) && approx(p_next_exit, gpt, 1e-12);
                if !(deg_prev && deg_next && prev_is_src && next_is_goal) {
                    panic!(
                        "consecutive traversal mismatch at path {} segment {}: prev(face={}, entry={:?}, exit={:?}) next(face={}, entry={:?}, exit={:?})",
                        i,
                        k - 1,
                        prev.face,
                        prev.entry,
                        prev.exit,
                        next.face,
                        next.entry,
                        next.exit
                    );
                }
            }
        }
        let ep = bary_poly_to_euclid(&vertices, &faces, poly);
        assert!(ep.len() >= 2);
        // Endpoints check
        let spt = bary_on_face(&vertices, &faces, source.face, source.bary);
        let gpt = bary_on_face(&vertices, &faces, goals[i].face, goals[i].bary);
        let eps = 1e-6;
        let f = ep.first().copied().unwrap();
        let l = ep.last().copied().unwrap();
        assert!(
            (approx(f, spt, eps) && approx(l, gpt, eps))
                || (approx(f, gpt, eps) && approx(l, spt, eps))
        );

        // Straightness proxy: on a flat mesh the geodesic should be the straight segment,
        // so the polyline length should equal the Euclidean distance within a tiny tolerance.
        let mut poly_len = 0.0;
        for k in 1..ep.len() {
            let a = ep[k - 1];
            let b = ep[k];
            let dx = a[0] - b[0];
            let dy = a[1] - b[1];
            let dz = a[2] - b[2];
            poly_len += (dx * dx + dy * dy + dz * dz).sqrt();
        }
        let dx = f[0] - l[0];
        let dy = f[1] - l[1];
        let dz = f[2] - l[2];
        let straight = (dx * dx + dy * dy + dz * dz).sqrt();
        assert!(
            (poly_len - straight).abs() <= (1e-6f64).max(1e-6 * straight),
            "path {} length not straight: poly_len={}, straight={} ep={:?}",
            i,
            poly_len,
            straight,
            ep
        );

        // Also ensure all points lie on the plane z=0 and within the bounding box segment with tolerance
        let tol = 1e-5;
        let minx = f[0].min(l[0]) - tol;
        let maxx = f[0].max(l[0]) + tol;
        let miny = f[1].min(l[1]) - tol;
        let maxy = f[1].max(l[1]) + tol;
        for (j, p) in ep.iter().enumerate() {
            assert!(p[2].abs() <= tol, "point {} must lie on z=0", j);
            assert!(
                p[0] >= minx && p[0] <= maxx && p[1] >= miny && p[1] <= maxy,
                "point {} outside segment bbox",
                j
            );
        }
    }
}
