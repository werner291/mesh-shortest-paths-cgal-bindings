//! Safe Rust wrapper around CGAL Surface_mesh_shortest_path via a C++ shim.

mod ffi {
    #![allow(non_camel_case_types, non_snake_case, non_upper_case_globals)]
    include!(concat!(env!("OUT_DIR"), "/bindings.rs"));
}

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

// Events coming from C++ visitor
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

pub fn group_events_to_traversals(
    event_paths: Vec<Vec<TraversalEvent>>,
) -> Vec<Vec<FaceTraversal>> {
    event_paths
        .into_iter()
        .map(single_event_path_to_traversals)
        .collect()
}

// Like group_events_to_traversals, but uses the mesh faces to properly handle Vertex events.
// When a Vertex event occurs, we end the current segment at the corresponding 1-hot barycentric
// coordinate on the current face, then continue from that vertex. This requires knowing which
// vertex index corresponds to which corner of the current face.
pub fn group_events_to_traversals_with_mesh(
    faces: &[[u32; 3]],
    event_paths: Vec<Vec<TraversalEvent>>,
) -> Vec<Vec<FaceTraversal>> {
    event_paths
        .into_iter()
        .map(|evs| single_event_path_to_traversals_with_mesh(faces, evs))
        .collect()
}

fn single_event_path_to_traversals(events: Vec<TraversalEvent>) -> Vec<FaceTraversal> {
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
                // Ensure current_face context is set to from_face and last is at from_bary
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
                        // Close the previous face segment before switching to from_face (should rarely happen)
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
                // Finish the segment on from_face up to the edge point
                if let Some(cf) = current_face {
                    travs.push(FaceTraversal {
                        face: cf,
                        entry,
                        exit: last,
                    });
                }
                // Enter next face at to_bary and continue
                current_face = Some(to_face);
                entry = to_bary;
                last = to_bary;
            }
            TraversalEvent::Vertex { .. } => {
                // Vertex events are explicitly represented but not needed for FaceTraversal grouping
                // We ignore them for traversal grouping purposes.
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

fn single_event_path_to_traversals_with_mesh(
    faces: &[[u32; 3]],
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
                    // close previous face run
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
                // Align current context to from_face at from_bary
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
                // finish segment on from_face to the edge point
                if let Some(cf) = current_face {
                    travs.push(FaceTraversal {
                        face: cf,
                        entry,
                        exit: last,
                    });
                }
                // switch to next face at to_bary
                current_face = Some(to_face);
                entry = to_bary;
                last = to_bary;
            }
            TraversalEvent::Vertex { vertex } => {
                if let Some(cf) = current_face {
                    // map vertex index to 1-hot bary on this face if present
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
                        // Close current segment up to last, then start a new run from the vertex
                        travs.push(FaceTraversal {
                            face: cf,
                            entry,
                            exit: last,
                        });
                        entry = vb;
                        last = vb;
                        // keep current_face unchanged
                    } else {
                        panic!("Vertex {} not on current face {}", vertex, cf);
                    }
                } else {
                    // No current face yet; ignore until we get context
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

fn group_bary_to_traversals(bary_paths: Vec<Vec<FaceBary>>) -> Vec<Vec<FaceTraversal>> {
    let mut trav_paths: Vec<Vec<FaceTraversal>> = Vec::with_capacity(bary_paths.len());
    for poly in bary_paths.into_iter() {
        trav_paths.push(single_bary_path_to_traversals(poly));
    }
    trav_paths
}

fn single_bary_path_to_traversals(poly: Vec<FaceBary>) -> Vec<FaceTraversal> {
    assert!(poly.len() >= 2);

    let mut travs: Vec<FaceTraversal> = Vec::new();
    let mut current_face = poly[0].face;
    let mut entry = poly[0].bary;
    let mut last = poly[0].bary;
    for fb in poly.into_iter().skip(1) {
        if fb.face == current_face {
            last = fb.bary;
        } else {
            travs.push(FaceTraversal {
                face: current_face,
                entry,
                exit: last,
            });
            current_face = fb.face;
            entry = fb.bary;
            last = fb.bary;
        }
    }
    // push last run
    travs.push(FaceTraversal {
        face: current_face,
        entry,
        exit: last,
    });

    travs
}

unsafe fn read_event_paths(
    out_paths: *mut *mut ffi::sp_event,
    out_sizes: *mut usize,
    count: usize,
) -> Vec<Vec<TraversalEvent>> {
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

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Vertices<'a>(pub &'a [[f64; 3]]);

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Faces<'a>(pub &'a [[u32; 3]]);

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Points3<'a>(pub &'a [[f64; 3]]);

/// Compute shortest paths on a triangle mesh from a Euclidean source to multiple Euclidean goals.
/// Internally, both source and goals are snapped to the nearest vertices and routed using the
/// barycentric-only C++ API; the returned polylines are expressed as barycentric states per segment.
pub fn shortest_paths(
    vertices: Vertices,
    faces: Faces,
    source: Point3,
    goals: Points3,
) -> Result<Vec<Vec<FaceTraversal>>, String> {
    // Flatten inputs for C++ mesh construction
    let mut verts_flat = Vec::with_capacity(vertices.0.len() * 3);
    for v in vertices.0.iter() {
        verts_flat.extend_from_slice(v);
    }
    let mut faces_flat = Vec::with_capacity(faces.0.len() * 3);
    for f in faces.0.iter() {
        faces_flat.extend_from_slice(f);
    }

    // Create context
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

    // Helper: compute nearest vertex index to a Euclidean point
    fn dist2(a: [f64; 3], b: [f64; 3]) -> f64 {
        let dx = a[0] - b[0];
        let dy = a[1] - b[1];
        let dz = a[2] - b[2];
        dx * dx + dy * dy + dz * dz
    }
    fn nearest_vertex_idx(vertices: &[[f64; 3]], p: [f64; 3]) -> usize {
        let mut best_i = 0usize;
        let mut best_d2 = f64::INFINITY;
        for (i, v) in vertices.iter().enumerate() {
            let d2 = dist2(*v, p);
            if d2 < best_d2 {
                best_d2 = d2;
                best_i = i;
            }
        }
        best_i
    }

    // Map a vertex index to a (face_index, bary) pair where the vertex is a corner of that face
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

    // Source: snap to nearest vertex, then represent as bary on a containing face
    let src_vi = nearest_vertex_idx(vertices.0, source.0);
    let (src_face, src_bary) = vertex_as_face_bary(faces.0, src_vi)
        .ok_or_else(|| "Source vertex is not part of any face".to_string())?;
    let rc =
        unsafe { ffi::sp_set_source_bary(ctx.0, src_face, src_bary[0], src_bary[1], src_bary[2]) };
    if rc != 0 {
        return Err(format!("sp_set_source_bary failed with code {}", rc));
    }

    // Goals: snap each to nearest vertex and express as bary on a containing face
    let mut face_indices: Vec<usize> = Vec::with_capacity(goals.0.len());
    let mut bary_flat: Vec<f64> = Vec::with_capacity(goals.0.len() * 3);
    for g in goals.0.iter() {
        let gvi = nearest_vertex_idx(vertices.0, *g);
        let (g_face, g_bary) = vertex_as_face_bary(faces.0, gvi)
            .ok_or_else(|| format!("Goal vertex {} is not part of any face", gvi))?;
        face_indices.push(g_face);
        bary_flat.extend_from_slice(&g_bary);
    }

    // Invoke event-based path computation and convert events to traversals
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

    // Read back visitation events and derive FaceTraversal sequences
    let mut event_paths: Vec<Vec<TraversalEvent>> =
        unsafe { read_event_paths(out_paths, out_sizes, goals.0.len()) };
    // Prepend explicit source barycentric point to each path to ensure clear start
    for evs in event_paths.iter_mut() {
        evs.insert(
            0,
            TraversalEvent::BaryPoint {
                face: src_face,
                bary: src_bary,
            },
        );
    }
    let trav_paths: Vec<Vec<FaceTraversal>> = group_events_to_traversals(event_paths);
    Ok(trav_paths)
}

/// Convenience adapter for meshes provided by the `geometry` crate.
/// This expects a triangle mesh with:
/// - vertices: `&[nalgebra::Point3<f64>]` or `&[[f64;3]]`
/// - faces: `&[[u32;3]]`
/// If you have a mesh type in `geometry`, convert it to these slices.

/// Compute shortest paths using barycentric coordinates on faces (no vertex snapping).
/// - source: a FaceBary specifying the source point
/// - goals: vector of FaceBary specifying goal points
pub fn shortest_paths_barycentric(
    vertices: &[[f64; 3]],
    faces: &[[u32; 3]],
    source: FaceBary,
    goals: &[FaceBary],
) -> Result<Vec<Vec<FaceTraversal>>, String> {
    // Flatten inputs
    let mut verts_flat = Vec::with_capacity(vertices.len() * 3);
    for v in vertices {
        verts_flat.extend_from_slice(v);
    }
    let mut faces_flat = Vec::with_capacity(faces.len() * 3);
    for f in faces {
        faces_flat.extend_from_slice(f);
    }

    // SAFETY: Same reasoning as the positional variant of `sp_create` above: we pass pointers to contiguous
    // Rust Vec buffers (`verts_flat`, `faces_flat`) with matching element types and correct logical counts.
    // The C++ code constructs its own mesh and does not retain these pointers beyond the call.
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
            // SAFETY: As above, `self.0` comes from a successful `sp_create` and is uniquely owned by this RAII
            // wrapper. `sp_destroy` is the correct matching destructor.
            unsafe { ffi::sp_destroy(self.0) }
        }
    }
    let ctx = Ctx(ctx);

    // SAFETY: `ctx.0` is a valid context. The face index and barycentric coordinates are passed by value.
    // The C++ implementation validates inputs and does not store references to our stack variables.
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
    // SAFETY: We pass valid pointers to contiguous arrays of faces and barycentric coordinates.
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

    // Read back visitation events, then derive FaceTraversal
    let event_paths: Vec<Vec<TraversalEvent>> =
        unsafe { read_event_paths(out_paths, out_sizes, goals.len()) };

    let trav_paths: Vec<Vec<FaceTraversal>> = group_events_to_traversals(event_paths);

    Ok(trav_paths)
}
