#pragma once

#include <cstddef>

#ifdef __cplusplus
extern "C" {
#endif

// Simple 3D point struct for output polylines
typedef struct {
    double x, y, z;
} sp_point3;

// Opaque context holding the mesh and CGAL shortest path state
typedef struct sp_context sp_context;

// Create context from arrays of vertices (len = vertex_count, xyz triples) and faces (len = face_count, each three u32 indices)
sp_context* sp_create(const double* vertices_xyz, size_t vertex_count,
                      const unsigned int* faces_tri, size_t face_count);

// Destroy context
void sp_destroy(sp_context* ctx);

// Set source point by coordinates; internally mapped to nearest vertex. Returns 0 on success, non-zero on error.
int sp_set_source(sp_context* ctx, double x, double y, double z);

// Compute shortest paths to each goal point (mapped to nearest vertex).
// On success returns 0 and fills out_paths with an array of goal_count pointers, each pointing to an array of sp_point3 of length out_sizes[i].
// The memory for paths and sizes must be freed by sp_free_paths.
int sp_compute_paths(sp_context* ctx,
                     const double* goal_xyz, size_t goal_count,
                     sp_point3*** out_paths,
                     size_t** out_sizes);

// Free the arrays allocated by sp_compute_paths
void sp_free_paths(sp_point3** paths, size_t* sizes, size_t goal_count);

#ifdef __cplusplus
} // extern "C"
#endif
