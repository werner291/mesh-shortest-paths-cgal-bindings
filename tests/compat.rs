use rust_cgal_shortest_paths::{FaceTraversal, Faces, Point3, Points3, Vertices, shortest_paths};

fn dist(a: [f64; 3], b: [f64; 3]) -> f64 {
    let dx = a[0] - b[0];
    let dy = a[1] - b[1];
    let dz = a[2] - b[2];
    (dx * dx + dy * dy + dz * dz).sqrt()
}

fn approx_pt(a: [f64; 3], b: [f64; 3], eps: f64) -> bool {
    dist(a, b) <= eps
}

fn grid_mesh(n: usize) -> (Vec<[f64; 3]>, Vec<[u32; 3]>) {
    assert!(n >= 2);
    // unit grid [0,1]x[0,1] on z=0, (n x n) vertices
    let mut vertices = Vec::with_capacity(n * n);
    for j in 0..n {
        for i in 0..n {
            let x = i as f64 / (n as f64 - 1.0);
            let y = j as f64 / (n as f64 - 1.0);
            vertices.push([x, y, 0.0]);
        }
    }
    let mut faces: Vec<[u32; 3]> = Vec::with_capacity((n - 1) * (n - 1) * 2);
    // Triangulate each cell with diagonal (i,j)->(i+1,j+1)
    let idx = |i: usize, j: usize| -> u32 { (j * n + i) as u32 };
    for j in 0..(n - 1) {
        for i in 0..(n - 1) {
            let v00 = idx(i, j);
            let v10 = idx(i + 1, j);
            let v01 = idx(i, j + 1);
            let v11 = idx(i + 1, j + 1);
            faces.push([v00, v10, v11]);
            faces.push([v00, v11, v01]);
        }
    }
    (vertices, faces)
}

fn nearest_vertex_idx(vertices: &[[f64; 3]], p: [f64; 3]) -> usize {
    let mut best = 0usize;
    let mut best_d = f64::INFINITY;
    for (i, v) in vertices.iter().enumerate() {
        let d = dist(*v, p);
        if d < best_d {
            best_d = d;
            best = i;
        }
    }
    best
}

#[test]
fn compatibility_layer_larger_mesh_endpoints_and_counts() {
    // Build a larger mesh (e.g., 30x30 grid => 900 vertices, 1682 faces)
    let n = 30usize;
    let (vertices, faces) = grid_mesh(n);

    // Define source and multiple goals in Euclidean coordinates; they will be snapped to nearest vertices
    let source = [0.12, 0.08, 0.0]; // near (0.103.., 0.069..), but snapping determines exact
    let goals = vec![
        [0.9, 0.9, 0.0], // near top-right corner
        [0.0, 1.0, 0.0], // exactly a vertex (top-left)
        [0.5, 0.2, 0.0], // interior point
        [0.1, 0.9, 0.0], // near left edge
        [1.0, 0.0, 0.0], // exactly a vertex (bottom-right)
    ];

    // Compute snapped endpoints (nearest vertices in the grid)
    let src_vi = nearest_vertex_idx(&vertices, source);
    let src_snap = vertices[src_vi];
    let goal_snaps: Vec<[f64; 3]> = goals
        .iter()
        .map(|&g| {
            let gi = nearest_vertex_idx(&vertices, g);
            vertices[gi]
        })
        .collect();

    let bary_paths = shortest_paths(
        Vertices(&vertices),
        Faces(&faces),
        Point3(source),
        Points3(&goals),
    )
    .expect("compute paths");

    // Validate consecutive traversals continuity (accept either global direction) for each path
    for (i, poly) in bary_paths.iter().enumerate() {
        for k in 1..poly.len() {
            let prev = poly[k - 1];
            let next = poly[k];
            let [i0, i1, i2] = faces[prev.face];
            let a = vertices[i0 as usize];
            let b = vertices[i1 as usize];
            let c = vertices[i2 as usize];
            let p_prev_exit = [
                prev.exit[0] * a[0] + prev.exit[1] * b[0] + prev.exit[2] * c[0],
                prev.exit[0] * a[1] + prev.exit[1] * b[1] + prev.exit[2] * c[1],
                prev.exit[0] * a[2] + prev.exit[1] * b[2] + prev.exit[2] * c[2],
            ];
            let p_prev_entry = [
                prev.entry[0] * a[0] + prev.entry[1] * b[0] + prev.entry[2] * c[0],
                prev.entry[0] * a[1] + prev.entry[1] * b[1] + prev.entry[2] * c[1],
                prev.entry[0] * a[2] + prev.entry[1] * b[2] + prev.entry[2] * c[2],
            ];
            let [j0, j1, j2] = faces[next.face];
            let a2 = vertices[j0 as usize];
            let b2 = vertices[j1 as usize];
            let c2 = vertices[j2 as usize];
            let p_next_entry = [
                next.entry[0] * a2[0] + next.entry[1] * b2[0] + next.entry[2] * c2[0],
                next.entry[0] * a2[1] + next.entry[1] * b2[1] + next.entry[2] * c2[1],
                next.entry[0] * a2[2] + next.entry[1] * b2[2] + next.entry[2] * c2[2],
            ];
            let p_next_exit = [
                next.exit[0] * a2[0] + next.exit[1] * b2[0] + next.exit[2] * c2[0],
                next.exit[0] * a2[1] + next.exit[1] * b2[1] + next.exit[2] * c2[1],
                next.exit[0] * a2[2] + next.exit[1] * b2[2] + next.exit[2] * c2[2],
            ];
            let fwd = approx_pt(p_prev_exit, p_next_entry, 1e-9);
            let bwd = approx_pt(p_prev_entry, p_next_exit, 1e-9);
            if !(fwd || bwd) {
                // Allow degenerate-at-endpoint pairs (library may only emit endpoints)
                let deg_prev = approx_pt(p_prev_entry, p_prev_exit, 1e-12);
                let deg_next = approx_pt(p_next_entry, p_next_exit, 1e-12);
                let src_snap = src_snap; // already computed above
                let gsnap = goal_snaps[i];
                let prev_is_src = approx_pt(p_prev_entry, src_snap, 1e-12)
                    && approx_pt(p_prev_exit, src_snap, 1e-12);
                let next_is_goal =
                    approx_pt(p_next_entry, gsnap, 1e-12) && approx_pt(p_next_exit, gsnap, 1e-12);
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
    }

    // Convert traversal sequences to Euclidean locally for verification
    let paths: Vec<Vec<[f64; 3]>> = bary_paths
        .iter()
        .map(|poly: &Vec<FaceTraversal>| {
            let mut out: Vec<[f64; 3]> = Vec::new();
            if poly.is_empty() {
                return out;
            }
            for (idx, t) in poly.iter().enumerate() {
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
                if idx + 1 == poly.len() {
                    let x = [
                        t.exit[0] * a[0] + t.exit[1] * b[0] + t.exit[2] * c[0],
                        t.exit[0] * a[1] + t.exit[1] * b[1] + t.exit[2] * c[1],
                        t.exit[0] * a[2] + t.exit[1] * b[2] + t.exit[2] * c[2],
                    ];
                    out.push(x);
                }
            }
            out
        })
        .collect();

    // Validate number of paths returned
    assert_eq!(paths.len(), goals.len(), "must return one path per goal");

    // Validate endpoints approximately equal to snapped vertices; accept either order
    let eps = 1e-9; // endpoints should be exact vertex coordinates, but allow tiny tolerance
    for (i, poly) in paths.iter().enumerate() {
        assert!(
            poly.len() >= 2,
            "polyline {} must have at least 2 points",
            i
        );
        let first = poly.first().copied().unwrap();
        let last = poly.last().copied().unwrap();
        let gsnap = goal_snaps[i];
        assert!(
            (approx_pt(first, src_snap, eps) && approx_pt(last, gsnap, eps))
                || (approx_pt(first, gsnap, eps) && approx_pt(last, src_snap, eps)),
            "path {} endpoints must match snapped source/goal (unordered): first={:?}, last={:?}, src={:?}, goal={:?}",
            i,
            first,
            last,
            src_snap,
            gsnap
        );

        // Additionally, check that every path point lies on the plane z=0 within tolerance
        for (j, p) in poly.iter().enumerate() {
            assert!(
                p[2].abs() <= 1e-9,
                "point {} of path {} should be on z=0 plane: {:?}",
                j,
                i,
                p
            );
        }
    }
}
