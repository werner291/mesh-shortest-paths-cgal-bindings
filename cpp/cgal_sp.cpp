#include "cgal_sp.h"

#include <vector>
#include <limits>
#include <cmath>
#include <new>
#include <array>

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/Surface_mesh_shortest_path.h>
#include <CGAL/boost/graph/Euler_operations.h>

using Kernel = CGAL::Exact_predicates_inexact_constructions_kernel;
using Point_3 = Kernel::Point_3;
using Surface_mesh = CGAL::Surface_mesh<Point_3>;
using Traits = CGAL::Surface_mesh_shortest_path_traits<Kernel, Surface_mesh>;
using Shortest_paths = CGAL::Surface_mesh_shortest_path<Traits>;
using vertex_descriptor = boost::graph_traits<Surface_mesh>::vertex_descriptor;
using face_descriptor = boost::graph_traits<Surface_mesh>::face_descriptor;
using halfedge_descriptor = boost::graph_traits<Surface_mesh>::halfedge_descriptor;

// Visitor to collect path as Face_location states
template <typename SSSP>
struct PathVisitor {
    const Surface_mesh &mesh;
    SSSP &path_algo;
    std::vector<typename SSSP::Face_location> &states;
    void operator()(typename SSSP::halfedge_descriptor edge, typename SSSP::FT t) {
        states.push_back(path_algo.face_location(edge, t));
        states.push_back(path_algo.face_location(mesh.opposite(edge), 1.0 - t));
    }
    void operator()(typename SSSP::vertex_descriptor vertex) {
        states.push_back(path_algo.face_location(vertex));
    }
    void operator()(typename SSSP::face_descriptor f, typename SSSP::Barycentric_coordinates location) {
        states.push_back({f, location});
    }
};

struct sp_context {
    Surface_mesh mesh;
    Shortest_paths sssp;

    // Input face index to CGAL face descriptor mapping
    std::vector<face_descriptor> faces;

    // Cached source vertex and point (after sp_set_source) OR bary source point (after sp_set_source_bary)
    vertex_descriptor source_v{};
    Point_3 source_p = Point_3(0,0,0);
    bool has_source = false;

    // Barycentric source cache
    size_t source_face_index = static_cast<size_t>(-1);
    Traits::Barycentric_coordinates source_bc = {{1.0,0.0,0.0}};
    bool has_source_bary = false;

    sp_context(Surface_mesh&& m)
        : mesh(std::move(m)), sssp(mesh) {}
};

static double sqr(double x){ return x*x; }

static vertex_descriptor nearest_vertex(const Surface_mesh& mesh, const Point_3& p) {
    double best_d2 = std::numeric_limits<double>::infinity();
    vertex_descriptor best_v;
    for (vertex_descriptor v : vertices(mesh)) {
        const auto& vp = mesh.point(v);
        double d2 = sqr(vp.x() - p.x()) + sqr(vp.y() - p.y()) + sqr(vp.z() - p.z());
        if (d2 < best_d2) { best_d2 = d2; best_v = v; }
    }
    return best_v;
}

static bool is_valid_bary(double b0, double b1, double b2) {
    const double eps = 1e-12;
    if (b0 < -eps || b1 < -eps || b2 < -eps) return false;
    if (b0 > 1.0+eps || b1 > 1.0+eps || b2 > 1.0+eps) return false;
    double s = b0 + b1 + b2;
    return std::abs(s - 1.0) <= 1e-9;
}

static std::array<vertex_descriptor,3> face_vertices(face_descriptor f, const Surface_mesh& m) {
    halfedge_descriptor h0 = halfedge(f, m);
    vertex_descriptor v0 = target(h0, m);
    vertex_descriptor v1 = target(next(h0, m), m);
    vertex_descriptor v2 = target(next(next(h0, m), m), m);
    return {v0, v1, v2};
}

static Point_3 bary_to_point(face_descriptor f, const Surface_mesh& m, double b0, double b1, double b2) {
    auto vs = face_vertices(f, m);
    const Point_3& p0 = m.point(vs[0]);
    const Point_3& p1 = m.point(vs[1]);
    const Point_3& p2 = m.point(vs[2]);
    return Point_3(
        b0*p0.x() + b1*p1.x() + b2*p2.x(),
        b0*p0.y() + b1*p1.y() + b2*p2.y(),
        b0*p0.z() + b1*p1.z() + b2*p2.z()
    );
}

extern "C" {

sp_context* sp_create(const double* vertices_xyz, size_t vertex_count,
                      const unsigned int* faces_tri, size_t face_count) {
    try {
        Surface_mesh mesh;
        std::vector<vertex_descriptor> vds;
        vds.reserve(vertex_count);
        for (size_t i = 0; i < vertex_count; ++i) {
            const double* p = &vertices_xyz[3*i];
            vds.push_back(mesh.add_vertex(Point_3(p[0], p[1], p[2])));
        }
        for (size_t i = 0; i < face_count; ++i) {
            const unsigned int* tri = &faces_tri[3*i];
            // Basic bounds check: ignore faces with bad indices
            if (tri[0] >= vds.size() || tri[1] >= vds.size() || tri[2] >= vds.size()) continue;
            std::array<vertex_descriptor,3> tri_vs = {vds[tri[0]], vds[tri[1]], vds[tri[2]]};
            CGAL::Euler::add_face(tri_vs, mesh);
        }
        // Ensure triangulated (we assume triangles input)
        auto* ctx = new sp_context(std::move(mesh));
        // Build face list mapping in the order of input faces
        ctx->faces.reserve(face_count);
        for (size_t i = 0; i < face_count; ++i) {
            const unsigned int* tri = &faces_tri[3*i];
            if (tri[0] >= vds.size() || tri[1] >= vds.size() || tri[2] >= vds.size()) continue;
            std::array<vertex_descriptor,3> tri_vs = {vds[tri[0]], vds[tri[1]], vds[tri[2]]};
            // Find a face with those three vertices; since we just added in order, we can collect by iterating faces in mesh after construction
            // But Surface_mesh does not guarantee same order; to ensure mapping, we search.
            face_descriptor found = face_descriptor();
            for (face_descriptor f : faces(ctx->mesh)) {
                // gather vertices
                halfedge_descriptor h0 = halfedge(f, ctx->mesh);
                vertex_descriptor fv0 = target(h0, ctx->mesh);
                vertex_descriptor fv1 = target(next(h0, ctx->mesh), ctx->mesh);
                vertex_descriptor fv2 = target(next(next(h0, ctx->mesh), ctx->mesh), ctx->mesh);
                // compare as unordered set
                std::array<vertex_descriptor,3> fvs = {fv0,fv1,fv2};
                int match = 0;
                for (auto &a : tri_vs) {
                    if (a==fvs[0] || a==fvs[1] || a==fvs[2]) ++match;
                }
                if (match==3) { found = f; break; }
            }
            if (found != face_descriptor()) ctx->faces.push_back(found);
        }
        return ctx;
    } catch (...) {
        return nullptr;
    }
}

void sp_destroy(sp_context* ctx) {
    delete ctx;
}

int sp_set_source(sp_context* ctx, double x, double y, double z) {
    if (!ctx) return 1;
    try {
        Point_3 p(x,y,z);
        vertex_descriptor v = nearest_vertex(ctx->mesh, p);
        // Reset the data and set the single source
        ctx->sssp.clear();
        ctx->sssp.add_source_point(v);
        ctx->source_v = v;
        ctx->source_p = ctx->mesh.point(v);
        ctx->has_source = true;
        return 0;
    } catch (...) { return 2; }
}

int sp_compute_paths(sp_context* ctx,
                     const double* goal_xyz, size_t goal_count,
                     sp_point3*** out_paths,
                     size_t** out_sizes) {
    if (!ctx || !out_paths || !out_sizes) return 1;
    try {
        sp_point3** paths = static_cast<sp_point3**>(::operator new[](goal_count * sizeof(sp_point3*)));
        size_t* sizes = static_cast<size_t*>(::operator new[](goal_count * sizeof(size_t)));

        for (size_t i = 0; i < goal_count; ++i) {
            const double* g = &goal_xyz[3*i];
            Point_3 gp(g[0], g[1], g[2]);
            vertex_descriptor gv = nearest_vertex(ctx->mesh, gp);

            std::vector<Point_3> poly;
            ctx->sssp.shortest_path_points_to_source_points(gv, std::back_inserter(poly));
            // CGAL returns points from goal to source; reverse to have source -> goal
            std::reverse(poly.begin(), poly.end());

            // Ensure the polyline starts at the exact stored source point and ends at the goal vertex point
            if (ctx->has_source) {
                if (poly.empty() || poly.front() != ctx->source_p) {
                    poly.insert(poly.begin(), ctx->source_p);
                }
            }
            Point_3 goal_p = ctx->mesh.point(gv);
            if (poly.empty() || poly.back() != goal_p) {
                poly.push_back(goal_p);
            }

            sizes[i] = poly.size();
            sp_point3* arr = static_cast<sp_point3*>(::operator new[](sizes[i] * sizeof(sp_point3)));
            for (size_t j = 0; j < sizes[i]; ++j) {
                arr[j].x = poly[j].x();
                arr[j].y = poly[j].y();
                arr[j].z = poly[j].z();
            }
            paths[i] = arr;
        }

        *out_paths = paths;
        *out_sizes = sizes;
        return 0;
    } catch (...) { return 2; }
}

void sp_free_paths(sp_point3** paths, size_t* sizes, size_t goal_count) {
    if (!paths || !sizes) return;
    for (size_t i = 0; i < goal_count; ++i) {
        ::operator delete[](paths[i]);
    }
    ::operator delete[](paths);
    ::operator delete[](sizes);
}

int sp_set_source_bary(sp_context* ctx, size_t face_index, double b0, double b1, double b2) {
    if (!ctx) return 1;
    try {
        if (face_index >= ctx->faces.size()) return 3;
        if (!is_valid_bary(b0,b1,b2)) return 3;
        face_descriptor f = ctx->faces[face_index];
        Point_3 p = bary_to_point(f, ctx->mesh, b0,b1,b2);
        // Construct a Face_location from face and barycentric coordinates
        Traits::Barycentric_coordinates bc = {{b0, b1, b2}};
        ctx->sssp.clear();
        ctx->sssp.add_source_point(f, bc);
        ctx->source_p = p;
        ctx->has_source = true;
        ctx->source_face_index = face_index;
        ctx->source_bc = bc;
        ctx->has_source_bary = true;
        return 0;
    } catch (...) { return 2; }
}

int sp_compute_paths_bary(sp_context* ctx,
                          const size_t* face_indices,
                          const double* bary_coords,
                          size_t goal_count,
                          sp_point3*** out_paths,
                          size_t** out_sizes) {
    if (!ctx || !face_indices || !bary_coords || !out_paths || !out_sizes) return 1;
    try {
        sp_point3** paths = static_cast<sp_point3**>(::operator new[](goal_count * sizeof(sp_point3*)));
        size_t* sizes = static_cast<size_t*>(::operator new[](goal_count * sizeof(size_t)));
        for (size_t i = 0; i < goal_count; ++i) {
            size_t fi = face_indices[i];
            double b0 = bary_coords[3*i+0];
            double b1 = bary_coords[3*i+1];
            double b2 = bary_coords[3*i+2];
            if (fi >= ctx->faces.size() || !is_valid_bary(b0,b1,b2)) {
                // allocate empty polyline for invalid entry
                sizes[i] = 0;
                paths[i] = nullptr;
                continue;
            }
            face_descriptor f = ctx->faces[fi];
            Point_3 gp = bary_to_point(f, ctx->mesh, b0,b1,b2);
            Traits::Barycentric_coordinates bc = {{b0, b1, b2}};

            std::vector<Point_3> poly;
            ctx->sssp.shortest_path_points_to_source_points(f, bc, std::back_inserter(poly));
            std::reverse(poly.begin(), poly.end());

            if (ctx->has_source) {
                if (poly.empty() || poly.front() != ctx->source_p) {
                    poly.insert(poly.begin(), ctx->source_p);
                }
            }
            if (poly.empty() || poly.back() != gp) {
                poly.push_back(gp);
            }

            sizes[i] = poly.size();
            sp_point3* arr = static_cast<sp_point3*>(::operator new[](sizes[i] * sizeof(sp_point3)));
            for (size_t j = 0; j < sizes[i]; ++j) {
                arr[j].x = poly[j].x();
                arr[j].y = poly[j].y();
                arr[j].z = poly[j].z();
            }
            paths[i] = arr;
        }
        *out_paths = paths;
        *out_sizes = sizes;
        return 0;
    } catch (...) { return 2; }
}

// Helper: find input face index for a face_descriptor
static size_t face_index_of(const sp_context* ctx, face_descriptor f) {
    for (size_t i = 0; i < ctx->faces.size(); ++i) {
        if (ctx->faces[i] == f) return i;
    }
    return static_cast<size_t>(-1);
}

int sp_compute_paths_bary_states(sp_context* ctx,
                                 const size_t* face_indices,
                                 const double* bary_coords,
                                 size_t goal_count,
                                 sp_face_bary*** out_paths,
                                 size_t** out_sizes) {
    if (!ctx || !face_indices || !bary_coords || !out_paths || !out_sizes) return 1;
    try {
        sp_face_bary** paths = static_cast<sp_face_bary**>(::operator new[](goal_count * sizeof(sp_face_bary*)));
        size_t* sizes = static_cast<size_t*>(::operator new[](goal_count * sizeof(size_t)));
        for (size_t i = 0; i < goal_count; ++i) {
            size_t fi = face_indices[i];
            double b0 = bary_coords[3*i+0];
            double b1 = bary_coords[3*i+1];
            double b2 = bary_coords[3*i+2];
            if (fi >= ctx->faces.size() || !is_valid_bary(b0,b1,b2)) {
                sizes[i] = 0;
                paths[i] = nullptr;
                continue;
            }
            face_descriptor f = ctx->faces[fi];
            Traits::Barycentric_coordinates bc = {{b0, b1, b2}};

            std::vector<Shortest_paths::Face_location> states;
            states.reserve(16);
            PathVisitor<Shortest_paths> vis{ctx->mesh, ctx->sssp, states};
            ctx->sssp.shortest_path_sequence_to_source_points(f, bc, vis);
            std::reverse(states.begin(), states.end());

            // Ensure endpoints (source first, goal last)
            if (ctx->has_source_bary) {
                states.insert(states.begin(), { ctx->faces[ctx->source_face_index], ctx->source_bc });
            }
            states.push_back({ f, bc });

            sizes[i] = states.size();
            sp_face_bary* arr = nullptr;
            if (sizes[i] > 0) {
                arr = static_cast<sp_face_bary*>(::operator new[](sizes[i] * sizeof(sp_face_bary)));
                for (size_t j = 0; j < sizes[i]; ++j) {
                    size_t fi_out = face_index_of(ctx, states[j].first);
                    arr[j].face = fi_out;
                    arr[j].b0 = states[j].second[0];
                    arr[j].b1 = states[j].second[1];
                    arr[j].b2 = states[j].second[2];
                }
            }
            paths[i] = arr;
        }
        *out_paths = paths;
        *out_sizes = sizes;
        return 0;
    } catch (...) { return 2; }
}

void sp_free_bary_paths(sp_face_bary** paths, size_t* sizes, size_t goal_count) {
    if (!paths || !sizes) return;
    for (size_t i = 0; i < goal_count; ++i) {
        ::operator delete[](paths[i]);
    }
    ::operator delete[](paths);
    ::operator delete[](sizes);
}

} // extern C
