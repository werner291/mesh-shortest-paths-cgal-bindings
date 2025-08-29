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
    face_a: usize,
    bary_a: [f64; 3],
    face_b: usize,
    bary_b: [f64; 3],
    eps: f64,
) -> bool {
    face_a == face_b
        && (bary_a[0] - bary_b[0]).abs() <= eps
        && (bary_a[1] - bary_b[1]).abs() <= eps
        && (bary_a[2] - bary_b[2]).abs() <= eps
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
        let prev_exit = bary_on_face(vertices, faces, a.face, a.exit);
        let next_entry = bary_on_face(vertices, faces, b.face, b.entry);
        assert!(approx_pt(prev_exit, next_entry, 1e-6));
    }
}

pub fn bary_on_face(
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
