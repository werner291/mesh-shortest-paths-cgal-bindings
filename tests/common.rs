use itertools::Itertools;
use rust_cgal_shortest_paths::FaceTraversal;

pub fn dist(a: [f64; 3], b: [f64; 3]) -> f64 {
    let dx = a[0] - b[0];
    let dy = a[1] - b[1];
    let dz = a[2] - b[2];
    (dx * dx + dy * dy + dz * dz).sqrt()
}

pub fn approx_pt(a: [f64; 3], b: [f64; 3], eps: f64) -> bool {
    dist(a, b) <= eps
}

pub fn approx_face_bary(
    a: rust_cgal_shortest_paths::FaceBary,
    b: rust_cgal_shortest_paths::FaceBary,
    eps: f64,
) -> bool {
    a.face == b.face
        && (a.bary[0] - b.bary[0]).abs() <= eps
        && (a.bary[1] - b.bary[1]).abs() <= eps
        && (a.bary[2] - b.bary[2]).abs() <= eps
}

pub fn approx_face_traversal(a: &FaceTraversal, b: &FaceTraversal, eps: f64) -> bool {
    a.face == b.face
        && (a.entry[0] - b.entry[0]).abs() <= eps
        && (a.entry[1] - b.entry[1]).abs() <= eps
        && (a.entry[2] - b.entry[2]).abs() <= eps
        && (a.exit[0] - b.exit[0]).abs() <= eps
        && (a.exit[1] - b.exit[1]).abs() <= eps
        && (a.exit[2] - b.exit[2]).abs() <= eps
}

pub fn polyline_len(poly: &[[f64; 3]]) -> f64 {
    if poly.is_empty() {
        return 0.0;
    }
    let mut s = 0.0;
    for i in 1..poly.len() {
        s += dist(poly[i - 1], poly[i]);
    }
    s
}

/// Validate that, for consecutive traversals, the end of the previous
/// matches the start of the next (within tolerance).
pub fn assert_consecutive_traversals_continuity(
    vertices: &[[f64; 3]],
    faces: &[[u32; 3]],
    path: &[FaceTraversal],
) {
    for (a, b) in path.iter().tuple_windows() {
        let prev_exit = bary_on_face(
            vertices,
            faces,
            rust_cgal_shortest_paths::FaceBary {
                face: a.face,
                bary: a.exit,
            },
        );
        let next_entry = bary_on_face(
            vertices,
            faces,
            rust_cgal_shortest_paths::FaceBary {
                face: b.face,
                bary: b.entry,
            },
        );
        assert!(approx_pt(prev_exit, next_entry, 1e-6));
    }
}

pub fn bary_on_face(
    vertices: &[[f64; 3]],
    faces: &[[u32; 3]],
    fb: rust_cgal_shortest_paths::FaceBary,
) -> [f64; 3] {
    let [i0, i1, i2] = faces[fb.face];
    let p0 = vertices[i0 as usize];
    let p1 = vertices[i1 as usize];
    let p2 = vertices[i2 as usize];
    let b = fb.bary;
    [
        b[0] * p0[0] + b[1] * p1[0] + b[2] * p2[0],
        b[0] * p0[1] + b[1] * p1[1] + b[2] * p2[1],
        b[0] * p0[2] + b[1] * p1[2] + b[2] * p2[2],
    ]
}
