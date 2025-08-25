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

/// Compute shortest paths on a triangle mesh from a source point to multiple goal points.
///
/// - vertices: list of 3D points (x,y,z) as f64
/// - faces: list of triangles as vertex indices (u32), length 3 each
/// - source: source point in 3D; mapped to nearest vertex internally
/// - goals: list of goal points; each mapped to nearest vertex internally
///
/// Returns a vector of polylines; each polyline is a list of 3D points along the path.
pub fn shortest_paths(
    vertices: &[[f64; 3]],
    faces: &[[u32; 3]],
    source: [f64; 3],
    goals: &[[f64; 3]],
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

    // Safety: build context
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

    // Ensure destruction
    struct Ctx(*mut ffi::sp_context);
    impl Drop for Ctx {
        fn drop(&mut self) {
            unsafe { ffi::sp_destroy(self.0) }
        }
    }
    let ctx = Ctx(ctx);

    let rc = unsafe { ffi::sp_set_source(ctx.0, source[0], source[1], source[2]) };
    if rc != 0 {
        return Err(format!("sp_set_source failed with code {}", rc));
    }

    let mut goals_flat = Vec::with_capacity(goals.len() * 3);
    for g in goals {
        goals_flat.extend_from_slice(g);
    }

    let mut out_paths: *mut *mut ffi::sp_point3 = std::ptr::null_mut();
    let mut out_sizes: *mut usize = std::ptr::null_mut();
    let rc = unsafe {
        ffi::sp_compute_paths(
            ctx.0,
            goals_flat.as_ptr(),
            goals.len(),
            &mut out_paths,
            &mut out_sizes,
        )
    };
    if rc != 0 {
        return Err(format!("sp_compute_paths failed with code {}", rc));
    }

    // Read back
    let mut result: Vec<Vec<[f64; 3]>> = Vec::with_capacity(goals.len());
    unsafe {
        let sizes_slice = std::slice::from_raw_parts(out_sizes, goals.len());
        let paths_slice = std::slice::from_raw_parts(out_paths, goals.len());
        for i in 0..goals.len() {
            let n = sizes_slice[i];
            let pts = std::slice::from_raw_parts(paths_slice[i], n);
            let mut poly = Vec::with_capacity(n);
            for p in pts {
                poly.push([p.x, p.y, p.z]);
            }
            result.push(poly);
        }
        ffi::sp_free_paths(out_paths, out_sizes, goals.len());
    }

    Ok(result)
}

/// Convenience adapter for meshes provided by the `geometry` crate.
/// This expects a triangle mesh with:
/// - vertices: `&[nalgebra::Point3<f64>]` or `&[[f64;3]]`
/// - faces: `&[[u32;3]]`
/// If you have a mesh type in `geometry`, convert it to these slices.
pub fn shortest_paths_from_geometry_mesh(
    vertices: &[[f64; 3]],
    faces: &[[u32; 3]],
    source: [f64; 3],
    goals: &[[f64; 3]],
) -> Result<Vec<Vec<[f64; 3]>>, String> {
    shortest_paths(vertices, faces, source, goals)
}

/// Compute shortest paths using barycentric coordinates on faces (no vertex snapping).
/// - source_face: index into faces array
/// - source_bary: [b0,b1,b2] with b0+b1+b2=1, each in [0,1]
/// - goals: vector of (face_index, [b0,b1,b2])
pub fn shortest_paths_barycentric(
    vertices: &[[f64; 3]],
    faces: &[[u32; 3]],
    source_face: usize,
    source_bary: [f64; 3],
    goals: &[(usize, [f64; 3])],
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

    // Build context
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
            source_face,
            source_bary[0],
            source_bary[1],
            source_bary[2],
        )
    };
    if rc != 0 {
        return Err(format!("sp_set_source_bary failed with code {}", rc));
    }

    let mut face_indices: Vec<usize> = Vec::with_capacity(goals.len());
    let mut bary_flat: Vec<f64> = Vec::with_capacity(goals.len() * 3);
    for (fi, bary) in goals.iter() {
        face_indices.push(*fi);
        bary_flat.extend_from_slice(&[bary[0], bary[1], bary[2]]);
    }

    let mut out_paths: *mut *mut ffi::sp_point3 = std::ptr::null_mut();
    let mut out_sizes: *mut usize = std::ptr::null_mut();
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
    unsafe {
        let sizes_slice = std::slice::from_raw_parts(out_sizes, goals.len());
        let paths_slice = std::slice::from_raw_parts(out_paths, goals.len());
        for i in 0..goals.len() {
            let n = sizes_slice[i];
            let pts = std::slice::from_raw_parts(paths_slice[i], n);
            let mut poly = Vec::with_capacity(n);
            for p in pts {
                poly.push([p.x, p.y, p.z]);
            }
            result.push(poly);
        }
        ffi::sp_free_paths(out_paths, out_sizes, goals.len());
    }

    Ok(result)
}
