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

/// Compute shortest paths on a triangle mesh from a source point to multiple goal points.
///
/// - vertices: list of 3D points (x,y,z) as f64
/// - faces: list of triangles as vertex indices (u32), length 3 each
/// - source: source point in 3D; mapped to nearest vertex internally
/// - goals: list of goal points; each mapped to nearest vertex internally
///
/// Returns a vector of polylines; each polyline is a list of 3D points along the path.
pub fn shortest_paths(
    vertices: Vertices,
    faces: Faces,
    source: Point3,
    goals: Points3,
) -> Result<Vec<Vec<[f64; 3]>>, String> {
    // Flatten inputs
    let mut verts_flat = Vec::with_capacity(vertices.0.len() * 3);
    for v in vertices.0.iter() {
        verts_flat.extend_from_slice(v);
    }
    let mut faces_flat = Vec::with_capacity(faces.0.len() * 3);
    for f in faces.0.iter() {
        faces_flat.extend_from_slice(f);
    }

    // SAFETY: We pass raw pointers to contiguous buffers owned by `verts_flat` and `faces_flat`.
    // - Element types match the C signatures: f64 -> double, u32 -> unsigned int.
    // - Lengths are provided as counts of logical vertices/faces; the C++ side multiplies by 3 to read xyz/triangles.
    // - sp_create only reads these pointers during the call and constructs its own CGAL mesh, not storing the pointers
    //   (see cpp/cgal_sp.cpp::sp_create), so the Rust Vecs may continue to be used/freed afterwards.
    // Therefore calling into FFI here is sound.
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

    // Ensure destruction
    struct Ctx(*mut ffi::sp_context);
    impl Drop for Ctx {
        fn drop(&mut self) {
            // SAFETY: `self.0` originates from a successful `sp_create` call where we verified the
            // pointer is non-null. Ownership of the context is uniquely held by this RAII wrapper,
            // and we never move or clone the raw pointer. `sp_destroy` is the matching destructor
            // for the allocation in the C++ shim.
            unsafe { ffi::sp_destroy(self.0) }
        }
    }
    let ctx = Ctx(ctx);

    // SAFETY: `ctx.0` is a valid, live context created above. The three f64 values are trivially FFI-safe.
    // The C++ implementation does not store references to the passed scalars; it only updates internal state.
    let rc = unsafe { ffi::sp_set_source(ctx.0, source.0[0], source.0[1], source.0[2]) };
    if rc != 0 {
        return Err(format!("sp_set_source failed with code {}", rc));
    }

    let mut goals_flat = Vec::with_capacity(goals.0.len() * 3);
    for g in goals.0.iter() {
        goals_flat.extend_from_slice(g);
    }

    let mut out_paths: *mut *mut ffi::sp_point3 = std::ptr::null_mut();
    let mut out_sizes: *mut usize = std::ptr::null_mut();
    // SAFETY: Arguments obey the C contract (see cpp/cgal_sp.h/cpp):
    // - `ctx.0` is a valid context.
    // - `goals_flat.as_ptr()` points to a buffer of `goals.0.len() * 3` f64 values and remains valid for the duration
    //    of the call. The C++ code only reads from it during the call.
    // - `out_paths` and `out_sizes` are valid out-pointers. On success, the callee allocates arrays of length
    //    `goals.0.len()` and writes them to these locations; ownership is transferred to us and must be released
    //    with `sp_free_paths`.
    let rc = unsafe {
        ffi::sp_compute_paths(
            ctx.0,
            goals_flat.as_ptr(),
            goals.0.len(),
            &mut out_paths,
            &mut out_sizes,
        )
    };
    if rc != 0 {
        return Err(format!("sp_compute_paths failed with code {}", rc));
    }

    // Read back
    let mut result: Vec<Vec<[f64; 3]>> = Vec::with_capacity(goals.0.len());

    // Guard to free C allocations even if a panic occurs while materializing results.
    struct PathsGuard {
        paths: *mut *mut ffi::sp_point3,
        sizes: *mut usize,
        count: usize,
    }
    impl Drop for PathsGuard {
        fn drop(&mut self) {
            // SAFETY: This matches the ownership contract: on success the C++ shim allocated arrays
            // for `paths` and `sizes` of length `count`. Calling `sp_free_paths` here is safe because
            // we do not hold any Rust references into that memory when Drop runs. The shim tolerates
            // null pointers when `count == 0` (and also early-returns on nulls), so zero-goal cases
            // are safe.
            unsafe { ffi::sp_free_paths(self.paths, self.sizes, self.count) }
        }
    }
    let guard = PathsGuard {
        paths: out_paths,
        sizes: out_sizes,
        count: goals.0.len(),
    };

    unsafe {
        // SAFETY: On success (`rc == 0`), the C++ side allocated two arrays of length `goals.0.len()` and wrote
        // their addresses to `out_paths` and `out_sizes`. We turn them into Rust slices to read them.
        // Both pointers are either non-null or well-defined for zero-length slices; here length > 0 if goals.len()>0.
        let sizes_slice = std::slice::from_raw_parts(guard.sizes, guard.count);
        let paths_slice = std::slice::from_raw_parts(guard.paths, guard.count);
        for i in 0..guard.count {
            let n = sizes_slice[i];
            // For each entry `i`, the callee guarantees that `paths_slice[i]` points to an array of `n` sp_point3
            // (or is null if `n == 0`). `from_raw_parts(null, 0)` is permitted in Rust.
            let pts = std::slice::from_raw_parts(paths_slice[i], n);
            let mut poly = Vec::with_capacity(n);
            for p in pts {
                poly.push([p.x, p.y, p.z]);
            }
            result.push(poly);
        }
    }
    // On the success path we have fully materialized `result`; dropping `guard` here will free the C buffers.
    drop(guard);

    Ok(result)
}

/// Convenience adapter for meshes provided by the `geometry` crate.
/// This expects a triangle mesh with:
/// - vertices: `&[nalgebra::Point3<f64>]` or `&[[f64;3]]`
/// - faces: `&[[u32;3]]`
/// If you have a mesh type in `geometry`, convert it to these slices.
pub fn shortest_paths_from_geometry_mesh(
    vertices: Vertices,
    faces: Faces,
    source: Point3,
    goals: Points3,
) -> Result<Vec<Vec<[f64; 3]>>, String> {
    shortest_paths(vertices, faces, source, goals)
}

/// Compute shortest paths using barycentric coordinates on faces (no vertex snapping).
/// - source: a FaceBary specifying the source point
/// - goals: vector of FaceBary specifying goal points
pub fn shortest_paths_barycentric(
    vertices: &[[f64; 3]],
    faces: &[[u32; 3]],
    source: FaceBary,
    goals: &[FaceBary],
) -> Result<Vec<Vec<[f64; 3]>>, String> {
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

    let mut out_paths: *mut *mut ffi::sp_point3 = std::ptr::null_mut();
    let mut out_sizes: *mut usize = std::ptr::null_mut();
    // SAFETY: Follows the FFI contract for the barycentric variant:
    // - `ctx.0` is a valid context.
    // - `face_indices.as_ptr()` and `bary_flat.as_ptr()` point to contiguous arrays with at least `goals.len()` and
    //    `goals.len()*3` elements respectively, alive for the duration of the call; the callee only reads from them.
    // - `out_paths` and `out_sizes` are valid out-pointers and will receive ownership of allocations on success.
    let rc = unsafe {
        ffi::sp_compute_paths_bary(
            ctx.0,
            face_indices.as_ptr(),
            bary_flat.as_ptr(),
            goals.len(),
            &mut out_paths,
            &mut out_sizes,
        )
    };
    if rc != 0 {
        return Err(format!("sp_compute_paths_bary failed with code {}", rc));
    }

    // Read back
    let mut result: Vec<Vec<[f64; 3]>> = Vec::with_capacity(goals.len());

    // Guard to free C allocations even if a panic occurs while materializing results.
    struct PathsGuard {
        paths: *mut *mut ffi::sp_point3,
        sizes: *mut usize,
        count: usize,
    }
    impl Drop for PathsGuard {
        fn drop(&mut self) {
            // SAFETY: This matches the ownership contract: on success the C++ shim allocated arrays
            // for `paths` and `sizes` of length `count`. Calling `sp_free_paths` here is safe because
            // we do not hold any Rust references into that memory when Drop runs. The shim tolerates
            // null pointers when `count == 0` (and also early-returns on nulls), so zero-goal cases
            // are safe.
            unsafe { ffi::sp_free_paths(self.paths, self.sizes, self.count) }
        }
    }
    let guard = PathsGuard {
        paths: out_paths,
        sizes: out_sizes,
        count: goals.len(),
    };

    unsafe {
        // SAFETY: On success, the C++ side set `out_sizes` and `out_paths` to arrays of length `goals.len()`.
        // Each entry `paths[i]` points to an array of `sizes[i]` sp_point3 or may be null if the size is zero.
        let sizes_slice = std::slice::from_raw_parts(guard.sizes, guard.count);
        let paths_slice = std::slice::from_raw_parts(guard.paths, guard.count);
        for i in 0..guard.count {
            let n = sizes_slice[i];
            let pts = std::slice::from_raw_parts(paths_slice[i], n);
            let mut poly = Vec::with_capacity(n);
            for p in pts {
                poly.push([p.x, p.y, p.z]);
            }
            result.push(poly);
        }
    }
    // On the success path we have fully materialized `result`; dropping `guard` here will free the C buffers.
    drop(guard);

    Ok(result)
}
