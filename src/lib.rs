//! Safe Rust wrapper around CGAL Surface_mesh_shortest_path via a C++ shim.
//!
//! This crate is organized into three modules:
//! - `types`: small public value types used in the API.
//! - `ffi`: low-level bindgen-generated FFI (crate-private).
//! - `interpretation`: safe wrappers and higher-level path extraction utilities.
//!
//! Quick start:
//! ```
//! use rust_cgal_shortest_paths::{Vertices, Faces, Points3, Point3, shortest_paths};
//! let verts = vec![[0.0,0.0,0.0],[1.0,0.0,0.0],[0.0,1.0,0.0]];
//! let faces = vec![[0u32,1,2]];
//! let source = Point3([0.1,0.1,0.0]);
//! let goals = vec![[0.9,0.1,0.0]];
//! let paths = shortest_paths(Vertices(&verts), Faces(&faces), source, Points3(&goals)).unwrap();
//! assert_eq!(paths.len(), 1);
//! ```

mod ffi;
mod interpretation;
pub mod types;

// Re-export public API surface from types and interpretation
pub use interpretation::{
    group_events_to_traversals, group_events_to_traversals_with_mesh, shortest_paths,
    shortest_paths_barycentric,
};
pub use types::{FaceBary, FaceTraversal, Faces, Point3, Points3, TraversalEvent, Vertices};
