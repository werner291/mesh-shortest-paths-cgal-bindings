//! Higher-level interpretation and safe wrappers on top of the FFI.
//!
//! This module contains the public safe APIs as well as helpers that translate
//! raw events to per-face traversals.
//!
//! Example (doctest):
//! ```
//! use rust_cgal_shortest_paths::{Vertices, Faces, Points3, Point3, shortest_paths};
//! // A single triangle mesh
//! let verts = vec![[0.0,0.0,0.0],[1.0,0.0,0.0],[0.0,1.0,0.0]];
//! let faces = vec![[0u32,1,2]];
//! let source = Point3([0.1,0.1,0.0]);
//! let goals = vec![[0.9,0.1,0.0]];
//! let paths = shortest_paths(Vertices(&verts), Faces(&faces), source, Points3(&goals)).unwrap();
//! assert_eq!(paths.len(), 1);
//! ```

use crate::ffi;
use crate::types::{FaceBary, FaceTraversal, Faces, Point3, Points3, TraversalEvent, Vertices};

pub fn group_events_to_traversals(
    event_paths: Vec<Vec<TraversalEvent>>,
) -> Vec<Vec<FaceTraversal>> {
    event_paths
        .into_iter()
        .map(|evs| single_event_path_to_traversals_generic(None, evs))
        .collect()
}

pub fn group_events_to_traversals_with_mesh(
    faces: &[[u32; 3]],
    event_paths: Vec<Vec<TraversalEvent>>,
) -> Vec<Vec<FaceTraversal>> {
    event_paths
        .into_iter()
        .map(|evs| single_event_path_to_traversals_with_mesh(faces, evs))
        .collect()
}

fn single_event_path_to_traversals_with_mesh(
    faces: &[[u32; 3]],
    events: Vec<TraversalEvent>,
) -> Vec<FaceTraversal> {
    single_event_path_to_traversals_generic(Some(faces), events)
}

fn single_event_path_to_traversals_generic(
    faces_opt: Option<&[[u32; 3]]>,
    events: Vec<TraversalEvent>,
) -> Vec<FaceTraversal> {
    let mut travs: Vec<FaceTraversal> = Vec::new();
    let mut current_face: Option<usize> = None;
    let mut entry: [f64; 3] = [0.0; 3];
    let mut last: [f64; 3] = [0.0; 3];

    for ev in events.into_iter() {
        match ev {
            TraversalEvent::BaryPoint { face, bary } => match current_face {
                None => {
                    current_face = Some(face);
                    entry = bary;
                    last = bary;
                }
                Some(cf) if cf == face => {
                    last = bary;
                }
                Some(cf) => {
                    travs.push(FaceTraversal {
                        face: cf,
                        entry,
                        exit: last,
                    });
                    current_face = Some(face);
                    entry = bary;
                    last = bary;
                }
            },
            TraversalEvent::Edge {
                from_face,
                from_bary,
                to_face,
                to_bary,
            } => {
                match current_face {
                    None => {
                        current_face = Some(from_face);
                        entry = from_bary;
                        last = from_bary;
                    }
                    Some(cf) if cf == from_face => {
                        last = from_bary;
                    }
                    Some(cf) => {
                        travs.push(FaceTraversal {
                            face: cf,
                            entry,
                            exit: last,
                        });
                        current_face = Some(from_face);
                        entry = from_bary;
                        last = from_bary;
                    }
                }
                if let Some(cf) = current_face {
                    travs.push(FaceTraversal {
                        face: cf,
                        entry,
                        exit: last,
                    });
                }
                current_face = Some(to_face);
                entry = to_bary;
                last = to_bary;
            }
            TraversalEvent::Vertex { vertex } => {
                if let (Some(cf), Some(faces)) = (current_face, faces_opt) {
                    let fv = faces[cf];
                    let one_hot = if fv[0] as usize == vertex {
                        Some([1.0, 0.0, 0.0])
                    } else if fv[1] as usize == vertex {
                        Some([0.0, 1.0, 0.0])
                    } else if fv[2] as usize == vertex {
                        Some([0.0, 0.0, 1.0])
                    } else {
                        None
                    };
                    if let Some(vb) = one_hot {
                        travs.push(FaceTraversal {
                            face: cf,
                            entry,
                            exit: vb,
                        });
                        entry = vb;
                        last = vb;
                    } else {
                        panic!("Vertex {} not on current face {}", vertex, cf);
                    }
                }
            }
        }
    }

    if let Some(cf) = current_face {
        travs.push(FaceTraversal {
            face: cf,
            entry,
            exit: last,
        });
    }
    travs
}

fn read_event_paths(
    out_paths: *mut *mut ffi::sp_event,
    out_sizes: *mut usize,
    count: usize,
) -> Vec<Vec<TraversalEvent>> {
    unsafe {
        struct PathsGuard {
            paths: *mut *mut ffi::sp_event,
            sizes: *mut usize,
            count: usize,
        }
        impl Drop for PathsGuard {
            fn drop(&mut self) {
                unsafe { ffi::sp_free_event_paths(self.paths, self.sizes, self.count) }
            }
        }
        let guard = PathsGuard {
            paths: out_paths,
            sizes: out_sizes,
            count,
        };
        let sizes_slice = std::slice::from_raw_parts(guard.sizes, guard.count);
        let paths_slice = std::slice::from_raw_parts(guard.paths, guard.count);
        let mut out: Vec<Vec<TraversalEvent>> = Vec::with_capacity(count);
        for i in 0..guard.count {
            let n = sizes_slice[i];
            let items = std::slice::from_raw_parts(paths_slice[i], n);
            let mut v = Vec::with_capacity(n);
            for it in items {
                let kind = it.kind as i32;
                if kind == ffi::sp_event_kind_SP_EVENT_EDGE as i32 {
                    v.push(TraversalEvent::Edge {
                        from_face: it.fa as usize,
                        from_bary: [it.a0, it.a1, it.a2],
                        to_face: it.fb as usize,
                        to_bary: [it.b0, it.b1, it.b2],
                    });
                } else if kind == ffi::sp_event_kind_SP_EVENT_BARY as i32 {
                    v.push(TraversalEvent::BaryPoint {
                        face: it.fa as usize,
                        bary: [it.a0, it.a1, it.a2],
                    });
                } else if kind == ffi::sp_event_kind_SP_EVENT_VERTEX as i32 {
                    v.push(TraversalEvent::Vertex {
                        vertex: it.vertex as usize,
                    });
                }
            }
            out.push(v);
        }
        out
    }
}

/// Compute shortest paths on a triangle mesh from a Euclidean source to multiple Euclidean goals.
/// Internally, both source and goals are snapped to the nearest vertices and routed using the
/// barycentric-only C++ API; the returned polylines are expressed as barycentric states per segment.
pub fn shortest_paths(
    vertices: Vertices,
    faces: Faces,
    source: Point3,
    goals: Points3,
) -> Result<Vec<Vec<FaceTraversal>>, String> {
    let mut verts_flat = Vec::with_capacity(vertices.0.len() * 3);
    for v in vertices.0.iter() {
        verts_flat.extend_from_slice(v);
    }
    let mut faces_flat = Vec::with_capacity(faces.0.len() * 3);
    for f in faces.0.iter() {
        faces_flat.extend_from_slice(f);
    }

    let ctx = unsafe {
        ffi::sp_create(
            verts_flat.as_ptr(),
            vertices.0.len(),
            faces_flat.as_ptr(),
            faces.0.len(),
        )
    };
    if ctx.is_null() {
        return Err("Failed to create CGAL context".into());
    }
    struct Ctx(*mut ffi::sp_context);
    impl Drop for Ctx {
        fn drop(&mut self) {
            unsafe { ffi::sp_destroy(self.0) }
        }
    }
    let ctx = Ctx(ctx);

    fn dist2(a: [f64; 3], b: [f64; 3]) -> f64 {
        let dx = a[0] - b[0];
        let dy = a[1] - b[1];
        let dz = a[2] - b[2];
        dx * dx + dy * dy + dz * dz
    }
    fn nearest_vertex_idx(vertices: &[[f64; 3]], p: [f64; 3]) -> usize {
        let mut best_i = 0usize;
        let mut best = f64::INFINITY;
        for (i, v) in vertices.iter().enumerate() {
            let d2 = dist2(*v, p);
            if d2 < best {
                best = d2;
                best_i = i;
            }
        }
        best_i
    }
    fn vertex_as_face_bary(faces: &[[u32; 3]], vi: usize) -> Option<(usize, [f64; 3])> {
        for (fi, f) in faces.iter().enumerate() {
            if f[0] as usize == vi {
                return Some((fi, [1.0, 0.0, 0.0]));
            }
            if f[1] as usize == vi {
                return Some((fi, [0.0, 1.0, 0.0]));
            }
            if f[2] as usize == vi {
                return Some((fi, [0.0, 0.0, 1.0]));
            }
        }
        None
    }

    let src_vi = nearest_vertex_idx(vertices.0, source.0);
    let (src_face, src_bary) = vertex_as_face_bary(faces.0, src_vi)
        .ok_or_else(|| "Source vertex is not part of any face".to_string())?;
    let rc =
        unsafe { ffi::sp_set_source_bary(ctx.0, src_face, src_bary[0], src_bary[1], src_bary[2]) };
    if rc != 0 {
        return Err(format!("sp_set_source_bary failed with code {}", rc));
    }

    let mut face_indices: Vec<usize> = Vec::with_capacity(goals.0.len());
    let mut bary_flat: Vec<f64> = Vec::with_capacity(goals.0.len() * 3);
    for g in goals.0.iter() {
        let gvi = nearest_vertex_idx(vertices.0, *g);
        let (g_face, g_bary) = vertex_as_face_bary(faces.0, gvi)
            .ok_or_else(|| format!("Goal vertex {} is not part of any face", gvi))?;
        face_indices.push(g_face);
        bary_flat.extend_from_slice(&g_bary);
    }

    let mut out_paths: *mut *mut ffi::sp_event = std::ptr::null_mut();
    let mut out_sizes: *mut usize = std::ptr::null_mut();
    let rc = unsafe {
        ffi::sp_compute_paths_events(
            ctx.0,
            face_indices.as_ptr(),
            bary_flat.as_ptr(),
            goals.0.len(),
            &mut out_paths,
            &mut out_sizes,
        )
    };
    if rc != 0 {
        return Err(format!(
            "sp_compute_paths_bary_states failed with code {}",
            rc
        ));
    }

    let mut event_paths = read_event_paths(out_paths, out_sizes, goals.0.len());
    for evs in event_paths.iter_mut() {
        evs.insert(
            0,
            TraversalEvent::BaryPoint {
                face: src_face,
                bary: src_bary,
            },
        );
    }
    Ok(group_events_to_traversals(event_paths))
}

/// Compute shortest paths using barycentric coordinates on faces (no vertex snapping).
pub fn shortest_paths_barycentric(
    vertices: &[[f64; 3]],
    faces: &[[u32; 3]],
    source: FaceBary,
    goals: &[FaceBary],
) -> Result<Vec<Vec<FaceTraversal>>, String> {
    let mut verts_flat = Vec::with_capacity(vertices.len() * 3);
    for v in vertices {
        verts_flat.extend_from_slice(v);
    }
    let mut faces_flat = Vec::with_capacity(faces.len() * 3);
    for f in faces {
        faces_flat.extend_from_slice(f);
    }

    let ctx = unsafe {
        ffi::sp_create(
            verts_flat.as_ptr(),
            vertices.len(),
            faces_flat.as_ptr(),
            faces.len(),
        )
    };
    if ctx.is_null() {
        return Err("Failed to create CGAL context".into());
    }
    struct Ctx(*mut ffi::sp_context);
    impl Drop for Ctx {
        fn drop(&mut self) {
            unsafe { ffi::sp_destroy(self.0) }
        }
    }
    let ctx = Ctx(ctx);

    let rc = unsafe {
        ffi::sp_set_source_bary(
            ctx.0,
            source.face,
            source.bary[0],
            source.bary[1],
            source.bary[2],
        )
    };
    if rc != 0 {
        return Err(format!("sp_set_source_bary failed with code {}", rc));
    }

    let mut face_indices: Vec<usize> = Vec::with_capacity(goals.len());
    let mut bary_flat: Vec<f64> = Vec::with_capacity(goals.len() * 3);
    for g in goals.iter() {
        face_indices.push(g.face);
        bary_flat.extend_from_slice(&[g.bary[0], g.bary[1], g.bary[2]]);
    }

    let mut out_paths: *mut *mut ffi::sp_event = std::ptr::null_mut();
    let mut out_sizes: *mut usize = std::ptr::null_mut();
    let rc = unsafe {
        ffi::sp_compute_paths_events(
            ctx.0,
            face_indices.as_ptr(),
            bary_flat.as_ptr(),
            goals.len(),
            &mut out_paths,
            &mut out_sizes,
        )
    };
    if rc != 0 {
        return Err(format!(
            "sp_compute_paths_bary_states failed with code {}",
            rc
        ));
    }

    let mut event_paths = read_event_paths(out_paths, out_sizes, goals.len());
    // Ensure paths begin exactly at the given source barycentric point for stable endpoints
    for evs in event_paths.iter_mut() {
        evs.insert(
            0,
            TraversalEvent::BaryPoint {
                face: source.face,
                bary: source.bary,
            },
        );
    }
    Ok(group_events_to_traversals(event_paths))
}
