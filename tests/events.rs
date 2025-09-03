use rust_cgal_shortest_paths::FaceTraversal;
use rust_cgal_shortest_paths::TraversalEvent;

// Directly test the pure-Rust grouping logic from events into traversals.
#[test]
fn group_events_simple_sequences() {
    // Case 1: single face with multiple bary points
    let events = vec![
        TraversalEvent::BaryPoint {
            face: 0,
            bary: [0.2, 0.3, 0.5],
        },
        TraversalEvent::BaryPoint {
            face: 0,
            bary: [0.3, 0.3, 0.4],
        },
        TraversalEvent::BaryPoint {
            face: 0,
            bary: [0.4, 0.3, 0.3],
        },
    ];
    let grouped = group_single(events.clone());
    assert_eq!(grouped.len(), 1);
    assert_eq!(
        grouped[0],
        FaceTraversal {
            face: 0,
            entry: [0.2, 0.3, 0.5],
            exit: [0.4, 0.3, 0.3]
        }
    );

    // Case 2: two faces separated by an edge event
    let events2 = vec![
        TraversalEvent::BaryPoint {
            face: 5,
            bary: [0.6, 0.2, 0.2],
        },
        TraversalEvent::Edge {
            from_face: 5,
            from_bary: [0.0, 1.0, 0.0],
            to_face: 7,
            to_bary: [0.0, 0.0, 1.0],
        },
        TraversalEvent::BaryPoint {
            face: 7,
            bary: [0.1, 0.2, 0.7],
        },
    ];
    let grouped2 = group_single(events2);
    assert_eq!(grouped2.len(), 2);
    assert_eq!(
        grouped2[0],
        FaceTraversal {
            face: 5,
            entry: [0.6, 0.2, 0.2],
            exit: [0.0, 1.0, 0.0]
        }
    );
    assert_eq!(
        grouped2[1],
        FaceTraversal {
            face: 7,
            entry: [0.0, 0.0, 1.0],
            exit: [0.1, 0.2, 0.7]
        }
    );

    // Case 3: start with an edge event (no prior bary), should create segment on from_face from from_bary to itself then switch
    let events3 = vec![
        TraversalEvent::Edge {
            from_face: 2,
            from_bary: [0.2, 0.3, 0.5],
            to_face: 3,
            to_bary: [0.4, 0.1, 0.5],
        },
        TraversalEvent::BaryPoint {
            face: 3,
            bary: [0.5, 0.1, 0.4],
        },
    ];
    let grouped3 = group_single(events3);
    assert_eq!(grouped3.len(), 2);
    // First segment ends at edge point on from_face (entry == exit because only edge point seen in that face)
    assert_eq!(
        grouped3[0],
        FaceTraversal {
            face: 2,
            entry: [0.2, 0.3, 0.5],
            exit: [0.2, 0.3, 0.5]
        }
    );
    assert_eq!(
        grouped3[1],
        FaceTraversal {
            face: 3,
            entry: [0.4, 0.1, 0.5],
            exit: [0.5, 0.1, 0.4]
        }
    );

    // Case 4: vertex events split the run at the vertex; needs mesh faces to map vertex->corner
    let events4 = vec![
        TraversalEvent::BaryPoint {
            face: 1,
            bary: [0.8, 0.1, 0.1],
        },
        TraversalEvent::Vertex { vertex: 42 },
        TraversalEvent::BaryPoint {
            face: 1,
            bary: [0.7, 0.2, 0.1],
        },
    ];
    // faces[1] must contain vertex 42 in position 1 => one-hot [0,1,0]
    let faces = vec![[0u32, 1, 2], [10, 42, 7], [3, 4, 5]];
    let grouped4 = group_single_with_mesh(&faces, events4);
    assert_eq!(grouped4.len(), 2);
    assert_eq!(
        grouped4[0],
        FaceTraversal {
            face: 1,
            entry: [0.8, 0.1, 0.1],
            exit: [0.0, 1.0, 0.0]
        }
    );
    assert_eq!(
        grouped4[1],
        FaceTraversal {
            face: 1,
            entry: [0.0, 1.0, 0.0],
            exit: [0.7, 0.2, 0.1]
        }
    );
}

fn group_single(events: Vec<TraversalEvent>) -> Vec<FaceTraversal> {
    // Provide a dummy faces mesh large enough to cover all face indices used in these tests
    // Faces content doesn't matter for cases without Vertex events.
    let faces: Vec<[u32; 3]> = (0..8)
        .map(|i| [i as u32, (i + 1) as u32, (i + 2) as u32])
        .collect();
    let paths = vec![events];
    let out = rust_cgal_shortest_paths::group_events_to_traversals(&faces, paths);
    assert_eq!(out.len(), 1);
    out.into_iter().next().unwrap()
}

fn group_single_with_mesh(faces: &[[u32; 3]], events: Vec<TraversalEvent>) -> Vec<FaceTraversal> {
    let paths = vec![events];
    let out = rust_cgal_shortest_paths::group_events_to_traversals(faces, paths);
    assert_eq!(out.len(), 1);
    out.into_iter().next().unwrap()
}
