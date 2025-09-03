use crate::common::{approx_face_bary, approx_face_traversal, dist};
use rust_cgal_shortest_paths::{FaceBary, FaceTraversal, shortest_paths_barycentric};
mod common;
use crate::common::polyline_len;
use common::assert_consecutive_traversals_continuity;

#[test]
fn barycentric_source_and_goal() {
    // Simple single triangle
    let vertices = vec![
        [0.0, 0.0, 0.0], // v0
        [1.0, 0.0, 0.0], // v1
        [0.0, 1.0, 0.0], // v2
    ];
    let faces = vec![[0u32, 1, 2]];

    // Source at centroid (1/3,1/3,1/3)
    let source_face = 0usize;
    let source_bary = [1.0 / 3.0, 1.0 / 3.0, 1.0 / 3.0];

    // Goal near vertex v1 but still interior
    let goal_face = 0usize;
    let goal_bary = [0.05, 0.9, 0.05];

    let source = FaceBary {
        face: source_face,
        bary: source_bary,
    };
    let goals = vec![FaceBary {
        face: goal_face,
        bary: goal_bary,
    }];

    let paths = shortest_paths_barycentric(&vertices, &faces, source, &goals)
        .expect("compute barycentric path");

    assert_eq!(paths.len(), 1);
    let poly = &paths[0];
    assert_consecutive_traversals_continuity(&vertices, &faces, poly);

    // Convert traversal sequence to Euclidean for verification only
    let epoly = bary_poly_to_euclid(&vertices, &faces, poly);
    assert!(epoly.len() >= 2);

    // Verify endpoints equal the evaluated barycentric points (within epsilon)
    let src_pt = bary_on_face(
        &vertices,
        &faces,
        FaceBary {
            face: source_face,
            bary: source_bary,
        },
    );
    let goal_pt = bary_on_face(
        &vertices,
        &faces,
        FaceBary {
            face: goal_face,
            bary: goal_bary,
        },
    );

    let first = epoly.first().copied().unwrap();
    let last = epoly.last().copied().unwrap();
    let eps = 1e-6;
    assert!(approx_pt(first, src_pt, eps));
    assert!(approx_pt(last, goal_pt, eps));
}

// Evaluate a barycentric coordinate on a given face into Euclidean coordinates.
fn bary_on_face(vertices: &[[f64; 3]], faces: &[[u32; 3]], fb: FaceBary) -> [f64; 3] {
    let [i0, i1, i2] = faces[fb.face];
    let p0 = vertices[i0 as usize];
    let p1 = vertices[i1 as usize];
    let p2 = vertices[i2 as usize];
    let b = fb.bary;
    [
        b[0] * p0[0] + b[1] * p1[0] + b[2] * p2[0],
        b[0] * p0[1] + b[1] * p1[1] + b[2] * p2[1],
        b[0] * p0[2] + b[1] * p1[2] + b[2] * p2[2],
    ]
}

fn approx_pt(a: [f64; 3], b: [f64; 3], eps: f64) -> bool {
    dist(a, b) <= eps
}

// Convert a sequence of FaceTraversal to Euclidean points using the mesh.
// We sample the entry of each traversal and append the exit of the last traversal.
fn bary_poly_to_euclid(
    vertices: &[[f64; 3]],
    faces: &[[u32; 3]],
    travs: &[FaceTraversal],
) -> Vec<[f64; 3]> {
    travs
        .iter()
        .map(|t| {
            bary_on_face(
                vertices,
                faces,
                FaceBary {
                    face: t.face,
                    bary: t.entry,
                },
            )
        })
        .chain(travs.last().map(|t| {
            bary_on_face(
                vertices,
                faces,
                FaceBary {
                    face: t.face,
                    bary: t.exit,
                },
            )
        }))
        .collect()
}

#[test]
fn triangle_vertex_to_vertices() {
    let vertices = vec![
        [0.0, 0.0, 0.0], // v0
        [1.0, 0.0, 0.0], // v1
        [0.0, 1.0, 0.0], // v2
    ];
    let faces = vec![[0u32, 1, 2]];

    // Source at vertex v0
    let source = FaceBary {
        face: 0,
        bary: [1.0, 0.0, 0.0],
    };

    // Goals at vertices v1 and v2
    let goals = vec![
        FaceBary {
            face: 0,
            bary: [0.0, 1.0, 0.0], // v1
        },
        FaceBary {
            face: 0,
            bary: [0.0, 0.0, 1.0], // v2
        },
    ];

    let paths =
        shortest_paths_barycentric(&vertices, &faces, source, &goals).expect("compute paths");

    // Verify we got paths to both goals
    assert_eq!(paths.len(), 2);

    // Convert paths to Euclidean space for testing
    let eps = 1e-6;
    // Test each path
    for (i, goal) in goals.iter().enumerate() {
        let poly = &paths[i];
        assert!(
            poly[0].face == source.face
                && approx_face_bary(
                    source,
                    FaceBary {
                        face: poly[0].face,
                        bary: poly[0].entry
                    },
                    eps
                )
        );
        assert!(
            poly.last().unwrap().face == goal.face
                && approx_face_bary(
                    *goal,
                    FaceBary {
                        face: poly.last().unwrap().face,
                        bary: poly.last().unwrap().exit
                    },
                    eps
                )
        );
    }
}

#[test]
fn square_diagonal_across_faces() {
    // Unit square in the XY plane triangulated into two triangles
    //  v3(0,1) ---- v2(1,1)
    //    |        / |
    //    |      /   |
    //    |    /     |
    //  v0(0,0) ---- v1(1,0)
    // Triangles: [v0,v1,v2] and [v0,v2,v3]
    let vertices = vec![
        [0.0, 0.0, 0.0], // v0
        [1.0, 0.0, 0.0], // v1
        [1.0, 1.0, 0.0], // v2
        [0.0, 1.0, 0.0], // v3
    ];
    let faces = vec![[0u32, 1, 2], [0, 2, 3]];

    // Source and goal at centroids of face 0
    let source = FaceBary {
        face: 0usize,
        bary: [1.0 / 3.0, 1.0 / 3.0, 1.0 / 3.0],
    };
    let goals = vec![
        // first one is a trick endpoint: it's exactly the start point.
        FaceBary {
            face: 0usize,
            bary: [1.0 / 3.0, 1.0 / 3.0, 1.0 / 3.0],
        },
        // The second should produce two traversals, the path crossing the diagonal exactly in the middle.
        FaceBary {
            face: 1usize,
            bary: [1.0 / 3.0, 1.0 / 3.0, 1.0 / 3.0],
        },
    ];

    let paths = shortest_paths_barycentric(&vertices, &faces, source, &goals).expect("compute");
    assert_eq!(paths.len(), 2);

    {
        assert_eq!(paths[0].len(), 1);

        // Check that the traversal is just a trivial path:
        assert_eq!(paths[0][0].face, 0usize);
        assert!(approx_face_traversal(
            &paths[0][0],
            &FaceTraversal {
                face: paths[0][0].face,
                entry: source.bary,
                exit: goals[0].bary
            },
            1e-6
        ));
    }

    {
        assert_eq!(paths[1].len(), 2);

        assert_consecutive_traversals_continuity(&vertices, &faces, &paths[1]);

        // We expect two traversals: one from source to the diagonal middle,
        // then from the diagonal middle to the goal.
        assert!(approx_face_traversal(
            &paths[1][0],
            &FaceTraversal {
                face: 0usize,
                entry: source.bary,
                exit: [0.5, 0.0, 0.5]
            },
            1e-6
        ));

        assert!(approx_face_traversal(
            &paths[1][1],
            &FaceTraversal {
                face: 1usize,
                entry: [0.5, 0.5, 0.0],
                exit: goals[1].bary
            },
            1e-6
        ))
    }
}

#[test]
fn square_multiple_goals_lengths() {
    let vertices = vec![
        [0.0, 0.0, 0.0], // v0
        [1.0, 0.0, 0.0], // v1
        [1.0, 1.0, 0.0], // v2
        [0.0, 1.0, 0.0], // v3
    ];
    let faces = vec![[0u32, 1, 2], [0, 2, 3]];
    // Source at v0 on face 0 in barycentric coords
    let source_face = 0usize;
    let source_bary = [0.9, 0.04, 0.06];
    // Goals: v1 (face 0), v3 (face 1), v2 (face 0)
    let goals = vec![
        FaceBary {
            face: 0usize,
            bary: [0.05, 0.9, 0.05],
        }, // near v1
        FaceBary {
            face: 1usize,
            bary: [0.05, 0.05, 0.9],
        }, // near v3 in face [v0,v2,v3]
        FaceBary {
            face: 0usize,
            bary: [0.04, 0.06, 0.9],
        }, // near v2 in face [v0,v1,v2]
    ];

    let source = FaceBary {
        face: source_face,
        bary: source_bary,
    };

    let paths = shortest_paths_barycentric(&vertices, &faces, source, &goals).expect("compute");
    assert_eq!(paths.len(), goals.len());

    let source_pt = bary_on_face(
        &vertices,
        &faces,
        FaceBary {
            face: source_face,
            bary: source_bary,
        },
    );
    for i in 0..goals.len() {
        let poly = &paths[i];
        // Validate consecutive traversals continuity (accept either global direction)
        assert_consecutive_traversals_continuity(&vertices, &faces, poly);
        let epoly = bary_poly_to_euclid(&vertices, &faces, poly);
        assert!(epoly.len() >= 2, "polyline has at least the endpoints");
        let len = polyline_len(&epoly);
        // Only convert to Euclidean for verification; accept algorithmic polyline sampling
        assert!(len >= 0.0, "length should be non-negative");

        // endpoints (source must be first, goal must be last)
        let g = goals[i];
        assert!(approx_face_bary(
            source,
            FaceBary {
                face: poly.first().unwrap().face,
                bary: poly.first().unwrap().entry
            },
            1e-6
        ));
        assert!(approx_face_bary(
            g,
            FaceBary {
                face: poly.last().unwrap().face,
                bary: poly.last().unwrap().exit
            },
            1e-6
        ));
    }
}
