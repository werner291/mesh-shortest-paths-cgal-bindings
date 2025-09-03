//! Public types for rust_cgal_shortest_paths.
//!
//! These are small value types used by the API. See crate-level docs for usage examples.

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Point3(pub [f64; 3]);

impl From<[f64; 3]> for Point3 {
    fn from(v: [f64; 3]) -> Self {
        Point3(v)
    }
}

/// A point specified by a face index and barycentric coordinates within that face.
/// b must satisfy b[0]+b[1]+b[2]=1 and each in [0,1].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FaceBary {
    pub face: usize,
    pub bary: [f64; 3],
}

/// A traversal across a single face, given by entry and exit barycentric
/// coordinates on that face. The first traversal's entry is the overall start
/// and the last traversal's exit is the overall end.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FaceTraversal {
    pub face: usize,
    pub entry: [f64; 3],
    pub exit: [f64; 3],
}

/// Events produced by the underlying CGAL visitor. Mostly internal but
/// can be useful for debugging.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum TraversalEvent {
    Edge {
        from_face: usize,
        from_bary: [f64; 3],
        to_face: usize,
        to_bary: [f64; 3],
    },
    BaryPoint {
        face: usize,
        bary: [f64; 3],
    },
    Vertex {
        vertex: usize,
    },
}

/// Newtype helpers used by the high-level API to improve parameter clarity.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Vertices<'a>(pub &'a [[f64; 3]]);

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Faces<'a>(pub &'a [[u32; 3]]);

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Points3<'a>(pub &'a [[f64; 3]]);
