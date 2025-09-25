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

// Visitor to collect path as event stream (edge traversal, barycentric point, vertex)
template <typename SSSP>
struct PathVisitor {
    const Surface_mesh &mesh;
    SSSP &path_algo;
    std::vector<sp_event> &events;
    void operator()(typename SSSP::halfedge_descriptor edge, typename SSSP::FT t) {
        // Single event representing traversal across an edge: left face point -> right/opposite face point
        auto loc_a = path_algo.face_location(edge, t);
        auto loc_b = path_algo.face_location(mesh.opposite(edge), 1.0 - t);
        sp_event ev{};
        ev.kind = static_cast<int>(SP_EVENT_EDGE);
        ev.fa = static_cast<size_t>(loc_a.first);
        ev.a0 = loc_a.second[1];
        ev.a1 = loc_a.second[2];
        ev.a2 = loc_a.second[0];
        ev.fb = static_cast<size_t>(loc_b.first);
        ev.b0 = loc_b.second[1];
        ev.b1 = loc_b.second[2];
        ev.b2 = loc_b.second[0];
        ev.vertex = 0;
        events.push_back(ev);
    }
    void operator()(typename SSSP::vertex_descriptor vertex) {
        sp_event ev{};
        ev.kind = static_cast<int>(SP_EVENT_VERTEX);
        ev.vertex = static_cast<size_t>(vertex);
        events.push_back(ev);
    }
    void operator()(typename SSSP::face_descriptor f, typename SSSP::Barycentric_coordinates location) {
        sp_event ev{};
        ev.kind = static_cast<int>(SP_EVENT_BARY);
        ev.fa = static_cast<size_t>(f);
        ev.a0 = location[1];
        ev.a1 = location[2];
        ev.a2 = location[0];
        events.push_back(ev);
    }
};

struct sp_context {
    Surface_mesh mesh;
    Shortest_paths sssp;
    // Barycentric source cache
    size_t source_face_index = static_cast<size_t>(-1);
    Traits::Barycentric_coordinates source_bc = {{1.0,0.0,0.0}};
    bool has_source_bary = false;

    sp_context(Surface_mesh&& m)
        : mesh(std::move(m)), sssp(mesh) {}
};

static double sqr(double x){ return x*x; }

static bool is_valid_bary(double b0, double b1, double b2) {
    const double eps = 1e-12;
    if (b0 < -eps || b1 < -eps || b2 < -eps) return false;
    if (b0 > 1.0+eps || b1 > 1.0+eps || b2 > 1.0+eps) return false;
    double s = b0 + b1 + b2;
    return std::abs(s - 1.0) <= 1e-9;
}

extern "C" {

sp_context* sp_create(const double* vertices_xyz, size_t vertex_count,
                      const unsigned int* faces_tri, size_t face_count) {
    Surface_mesh mesh;
    std::vector<vertex_descriptor> vds;
    vds.reserve(vertex_count);
    for (size_t i = 0; i < vertex_count; ++i) {
        const double* p = &vertices_xyz[3*i];
        vds.push_back(mesh.add_vertex(Point_3(p[0], p[1], p[2])));
    }
    for (size_t i = 0; i < face_count; ++i) {
        const unsigned int* tri = &faces_tri[3*i];
        std::array<vertex_descriptor,3> tri_vs = {vds[tri[0]], vds[tri[1]], vds[tri[2]]};
        face_descriptor face = CGAL::Euler::add_face(tri_vs, mesh);
        assert(i == face);
        // Verify that the vertex indices match what we passed in
        halfedge_descriptor h = mesh.halfedge(face);

        // In a cruel twist of fate, CGAL sets the *last* halfedge added as the face's halfedge,
        // so we need to rotate the indices by exactly one.
        //
        // We use an assertion to check that our janky setup doesn't break.
        std::array<unsigned int, 3> vertices;
        vertices[2] = static_cast<unsigned int>(mesh.source(h));
        h = mesh.next(h);
        vertices[0] = static_cast<unsigned int>(mesh.source(h));
        h = mesh.next(h);  
        vertices[1] = static_cast<unsigned int>(mesh.source(h));

        assert(tri[0] == vertices[0]);
        assert(tri[1] == vertices[1]);
        assert(tri[2] == vertices[2]);
    }

    return new sp_context(std::move(mesh));
}

void sp_destroy(sp_context* ctx) {
    delete ctx;
}

int sp_set_source_bary(sp_context* ctx, size_t face_index, double b0, double b1, double b2) {
    if (!ctx) return 1;
    face_descriptor f { face_index };
    Traits::Barycentric_coordinates bc = {{b2, b0, b1}}; // Also swapping indices.
    ctx->sssp.clear();
    ctx->sssp.add_source_point(f, bc);
    ctx->source_face_index = face_index;
    ctx->source_bc = bc;
    ctx->has_source_bary = true;
    return 0;
}

void sp_free_bary_paths(sp_face_bary** paths, size_t* sizes, size_t goal_count) {
    if (!paths || !sizes) return;
    for (size_t i = 0; i < goal_count; ++i) {
        ::operator delete[](paths[i]);
    }
    ::operator delete[](paths);
    ::operator delete[](sizes);
}

int sp_compute_paths_events(sp_context* ctx,
                            const size_t* face_indices,
                            const double* bary_coords,
                            size_t goal_count,
                            sp_event*** out_paths,
                            size_t** out_sizes) {
    if (!ctx) return 1;
    sp_event** paths = static_cast<sp_event**>(::operator new[](goal_count * sizeof(sp_event*)));
    size_t* sizes = static_cast<size_t*>(::operator new[](goal_count * sizeof(size_t)));
    for (size_t i = 0; i < goal_count; ++i) {
        size_t fi = face_indices[i];
        double b0 = bary_coords[3*i+0];
        double b1 = bary_coords[3*i+1];
        double b2 = bary_coords[3*i+2];
        face_descriptor f {fi};
        Traits::Barycentric_coordinates bc = {{b2, b0, b1}};

        std::vector<sp_event> events;
        PathVisitor<Shortest_paths> vis{ctx->mesh, ctx->sssp, events};
        ctx->sssp.shortest_path_sequence_to_source_points(f, bc, vis);
        std::reverse(events.begin(), events.end());
        // After reversing to source->goal order, edge events still encode goal->source transitions.
        // Swap their from/to endpoints to encode source->goal transitions.
        for (auto &ev : events) {
            if (ev.kind == SP_EVENT_EDGE) {
                std::swap(ev.fa, ev.fb);
                std::swap(ev.a0, ev.b0);
                std::swap(ev.a1, ev.b1);
                std::swap(ev.a2, ev.b2);
            }
        }

        // Workaround for a presumed bug in CGAL "Surface_mesh_shortest_path::nearest_on_face"
        // whereby source points sharing a face never emit any events.
        if (events.empty() && ctx->has_source_bary) {
            sp_event start{};
            start.kind = SP_EVENT_BARY;
            start.fa = ctx->source_face_index;
            start.a0 = ctx->source_bc[1];
            start.a1 = ctx->source_bc[2];
            start.a2 = ctx->source_bc[0];
            events.push_back(start);

            sp_event end{};
            end.kind = SP_EVENT_BARY;
            end.fa = fi;
            end.a0 = bc[1];
            end.a1 = bc[2];
            end.a2 = bc[0];
            events.push_back(end);
        }

        sizes[i] = events.size();
        
        sp_event* arr = static_cast<sp_event*>(::operator new[](sizes[i] * sizeof(sp_event)));
        for (size_t j = 0; j < sizes[i]; ++j) {
            arr[j] = events[j];
        }
        paths[i] = arr;
    }
    *out_paths = paths;
    *out_sizes = sizes;
    return 0;
}

void sp_free_event_paths(sp_event** paths, size_t* sizes, size_t goal_count) {
    if (!paths || !sizes) return;
    for (size_t i = 0; i < goal_count; ++i) {
        ::operator delete[](paths[i]);
    }
    ::operator delete[](paths);
    ::operator delete[](sizes);
}

} // extern C
