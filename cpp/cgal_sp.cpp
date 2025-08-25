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

struct sp_context {
    Surface_mesh mesh;
    Shortest_paths sssp;

    // Cached source vertex and point (after sp_set_source)
    vertex_descriptor source_v{};
    Point_3 source_p = Point_3(0,0,0);
    bool has_source = false;

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
                // Avoid duplicating if already equal (approx equality is tricky; exact equality should hold for vertex points)
                if (poly.empty() || poly.back() != goal_p) {
                    poly.push_back(goal_p);
                }
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

} // extern C
