#pragma once

#include <cstddef>

#ifdef __cplusplus
extern "C" {
#endif

// Barycentric location on a face
typedef struct {
    size_t face;
    double b0, b1, b2;
} sp_face_bary;

// Opaque context holding the mesh and CGAL shortest path state
typedef struct sp_context sp_context;

// Create context from arrays of vertices (len = vertex_count, xyz triples) and faces (len = face_count, each three u32 indices)
sp_context* sp_create(const double* vertices_xyz, size_t vertex_count,
                      const unsigned int* faces_tri, size_t face_count);

// Destroy context
void sp_destroy(sp_context* ctx);

// Set source point by barycentric coordinates within a face. face_index in [0, face_count).
// Returns 0 on success, non-zero on error (2 for exceptions, 3 for invalid input).
int sp_set_source_bary(sp_context* ctx, size_t face_index, double b0, double b1, double b2);

// Compute shortest paths specified by barycentric goals and return barycentric path states.
// On success fills out_paths with an array (per goal) of sp_face_bary of length out_sizes[i].
int sp_compute_paths_bary_states(sp_context* ctx,
                                 const size_t* face_indices,
                                 const double* bary_coords,
                                 size_t goal_count,
                                 sp_face_bary*** out_paths,
                                 size_t** out_sizes);

// Free the arrays allocated by sp_compute_paths_bary_states (barycentric)
void sp_free_bary_paths(sp_face_bary** paths, size_t* sizes, size_t goal_count);

#ifdef __cplusplus
} // extern "C"
#endif
