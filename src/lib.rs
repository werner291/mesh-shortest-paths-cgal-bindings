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
) -> Result<Vec<Vec<FaceBary>>, String> {
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

    // Invoke barycentric path computation to get barycentric path states directly
    let mut out_paths: *mut *mut ffi::sp_face_bary = std::ptr::null_mut();
    let mut out_sizes: *mut usize = std::ptr::null_mut();
    let rc = unsafe {
        ffi::sp_compute_paths_bary_states(
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

    // Read back barycentric paths directly
    let mut bary_paths: Vec<Vec<FaceBary>> = Vec::with_capacity(goals.0.len());
    struct PathsGuard {
        paths: *mut *mut ffi::sp_face_bary,
        sizes: *mut usize,
        count: usize,
    }
    impl Drop for PathsGuard {
        fn drop(&mut self) {
            unsafe { ffi::sp_free_bary_paths(self.paths, self.sizes, self.count) }
        }
    }
    let guard = PathsGuard {
        paths: out_paths,
        sizes: out_sizes,
        count: goals.0.len(),
    };
    unsafe {
        let sizes_slice = std::slice::from_raw_parts(guard.sizes, guard.count);
        let paths_slice = std::slice::from_raw_parts(guard.paths, guard.count);
        for i in 0..guard.count {
            let n = sizes_slice[i];
            let items = std::slice::from_raw_parts(paths_slice[i], n);
            let mut poly = Vec::with_capacity(n);
            for it in items {
                poly.push(FaceBary {
                    face: it.face as usize,
                    bary: [it.b0, it.b1, it.b2],
                });
            }
            bary_paths.push(poly);
        }
    }
    Ok(bary_paths)
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
) -> Result<Vec<Vec<FaceBary>>, String> {
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

    let mut out_paths: *mut *mut ffi::sp_face_bary = std::ptr::null_mut();
    let mut out_sizes: *mut usize = std::ptr::null_mut();
    // SAFETY: We pass valid pointers to contiguous arrays of faces and barycentric coordinates.
    let rc = unsafe {
        ffi::sp_compute_paths_bary_states(
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

    // Read back barycentric paths directly
    let mut bary_paths: Vec<Vec<FaceBary>> = Vec::with_capacity(goals.len());

    // Guard to free C allocations even if a panic occurs while materializing results.
    struct PathsGuard {
        paths: *mut *mut ffi::sp_face_bary,
        sizes: *mut usize,
        count: usize,
    }
    impl Drop for PathsGuard {
        fn drop(&mut self) {
            unsafe { ffi::sp_free_bary_paths(self.paths, self.sizes, self.count) }
        }
    }
    let guard = PathsGuard {
        paths: out_paths,
        sizes: out_sizes,
        count: goals.len(),
    };

    unsafe {
        let sizes_slice = std::slice::from_raw_parts(guard.sizes, guard.count);
        let paths_slice = std::slice::from_raw_parts(guard.paths, guard.count);
        for i in 0..guard.count {
            let n = sizes_slice[i];
            let items = std::slice::from_raw_parts(paths_slice[i], n);
            let mut poly = Vec::with_capacity(n);
            for it in items {
                poly.push(FaceBary {
                    face: it.face as usize,
                    bary: [it.b0, it.b1, it.b2],
                });
            }
            bary_paths.push(poly);
        }
    }

    Ok(bary_paths)
}
