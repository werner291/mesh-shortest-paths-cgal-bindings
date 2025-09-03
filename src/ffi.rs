//! Low-level FFI bindings module.
//!
//! This module exposes the bindgen-generated C FFI items to the rest of the crate.
//! It is intentionally not re-exported publicly in full; higher-level APIs wrap it.

#![allow(non_camel_case_types, non_snake_case, non_upper_case_globals)]

include!(concat!(env!("OUT_DIR"), "/bindings.rs"));
