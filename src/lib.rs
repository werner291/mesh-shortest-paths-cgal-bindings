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
/// Returns a vector of polylines; each polyline is a list of barycentric points (face index + [b0,b1,b2]) along the path.
pub fn shortest_paths(
    vertices: Vertices,
    faces: Faces,
    source: Point3,
    goals: Points3,
) -> Result<Vec<Vec<FaceBary>>, String> {
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

    // Read back Euclidean polylines first
    let mut euclid_paths: Vec<Vec<[f64; 3]>> = Vec::with_capacity(goals.0.len());

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
            let pts = std::slice::from_raw_parts(paths_slice[i], n);
            let mut poly = Vec::with_capacity(n);
            for p in pts {
                poly.push([p.x, p.y, p.z]);
            }
            euclid_paths.push(poly);
        }
    }
    // Free C allocations now that we've copied the data.
    drop(guard);

    // Helper to compute barycentric coordinates of point p on triangle (a,b,c).
    fn barycentric_for_point(
        a: [f64; 3],
        b: [f64; 3],
        c: [f64; 3],
        p: [f64; 3],
    ) -> Option<[f64; 3]> {
        let v0 = [b[0] - a[0], b[1] - a[1], b[2] - a[2]];
        let v1 = [c[0] - a[0], c[1] - a[1], c[2] - a[2]];
        let v2 = [p[0] - a[0], p[1] - a[1], p[2] - a[2]];
        let dot = |u: [f64; 3], v: [f64; 3]| u[0] * v[0] + u[1] * v[1] + u[2] * v[2];
        let d00 = dot(v0, v0);
        let d01 = dot(v0, v1);
        let d11 = dot(v1, v1);
        let d20 = dot(v2, v0);
        let d21 = dot(v2, v1);
        let denom = d00 * d11 - d01 * d01;
        if denom.abs() < 1e-20 {
            return None;
        }
        let v = (d11 * d20 - d01 * d21) / denom;
        let w = (d00 * d21 - d01 * d20) / denom;
        let u = 1.0 - v - w;
        // Check if p lies close to the triangle plane and inside the triangle within tolerance.
        // Reconstruct point from bary to assess plane error:
        let rp = [
            u * a[0] + v * b[0] + w * c[0],
            u * a[1] + v * b[1] + w * c[1],
            u * a[2] + v * b[2] + w * c[2],
        ];
        let plane_err =
            ((rp[0] - p[0]).abs() + (rp[1] - p[1]).abs() + (rp[2] - p[2]).abs()).max(0.0);
        let eps = 1e-8;
        if plane_err <= 1e-6
            && u >= -eps
            && v >= -eps
            && w >= -eps
            && u <= 1.0 + eps
            && v <= 1.0 + eps
            && w <= 1.0 + eps
        {
            Some([u, v, w])
        } else {
            None
        }
    }

    // Map each Euclidean point to a FaceBary by searching faces.
    let mut bary_paths: Vec<Vec<FaceBary>> = Vec::with_capacity(euclid_paths.len());
    for poly in euclid_paths.into_iter() {
        let mut out_poly: Vec<FaceBary> = Vec::with_capacity(poly.len());
        'points: for p in poly.into_iter() {
            // Try to find a containing face
            for (fi, f) in faces.0.iter().enumerate() {
                let a = vertices.0[f[0] as usize];
                let b = vertices.0[f[1] as usize];
                let c = vertices.0[f[2] as usize];
                if let Some(bary) = barycentric_for_point(a, b, c, p) {
                    out_poly.push(FaceBary { face: fi, bary });
                    continue 'points;
                }
            }
            // Fallback: try exact vertex match (within small epsilon)
            let mut pushed = false;
            let eps = 1e-9;
            for (fi, f) in faces.0.iter().enumerate() {
                let a = vertices.0[f[0] as usize];
                let b = vertices.0[f[1] as usize];
                let c = vertices.0[f[2] as usize];
                let eq = |x: [f64; 3], y: [f64; 3]| {
                    (x[0] - y[0]).abs() <= eps
                        && (x[1] - y[1]).abs() <= eps
                        && (x[2] - y[2]).abs() <= eps
                };
                if eq(p, a) {
                    out_poly.push(FaceBary {
                        face: fi,
                        bary: [1.0, 0.0, 0.0],
                    });
                    pushed = true;
                    break;
                }
                if eq(p, b) {
                    out_poly.push(FaceBary {
                        face: fi,
                        bary: [0.0, 1.0, 0.0],
                    });
                    pushed = true;
                    break;
                }
                if eq(p, c) {
                    out_poly.push(FaceBary {
                        face: fi,
                        bary: [0.0, 0.0, 1.0],
                    });
                    pushed = true;
                    break;
                }
            }
            if !pushed {
                // As a last resort, assign to face 0 with dummy bary (may happen only on numerical edge cases)
                if let Some(first_face) = faces.0.get(0) {
                    let a = vertices.0[first_face[0] as usize];
                    let b = vertices.0[first_face[1] as usize];
                    let c = vertices.0[first_face[2] as usize];
                    let bary = barycentric_for_point(a, b, c, p).unwrap_or([1.0, 0.0, 0.0]);
                    out_poly.push(FaceBary { face: 0, bary });
                } else {
                    // Mesh with no faces: push a degenerate placeholder
                    out_poly.push(FaceBary {
                        face: 0,
                        bary: [1.0, 0.0, 0.0],
                    });
                }
            }
        }
        bary_paths.push(out_poly);
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
