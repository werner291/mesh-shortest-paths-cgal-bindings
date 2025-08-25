# rust_cgal_shortest_paths

Rust bindings and utilities for computing shortest paths on meshes via CGAL (C++). This crate bridges Rust with a small C++ layer that calls into CGAL to perform geodesic shortest path operations.

⚠️ Status: Largely untested. Use at your own risk.

This repository is experimental. While there are some basic tests and examples, the code hasn’t been comprehensively validated across platforms, CGAL versions, or edge cases. Expect breaking changes and please validate results independently before using in production.

## Getting started

- Requirements:
  - A working C++ toolchain
  - CGAL and its dependencies installed and discoverable (typically via pkg-config)
  - Rust (stable)

- Build:
  - `cargo build`
  - `cargo test` to run the small test suite

## License

This project is licensed under the MIT No Attribution (MIT-0) license. See the `LICENSE` file for details.
