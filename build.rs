use std::env;
use std::path::PathBuf;

fn main() {
    // Re-run build script if these files change
    println!("cargo:rerun-if-changed=cpp/cgal_sp.h");
    println!("cargo:rerun-if-changed=cpp/cgal_sp.cpp");

    // Generate bindings
    let bindings = bindgen::Builder::default()
        .header("cpp/cgal_sp.h")
        .allowlist_function("sp_.*")
        .allowlist_type("sp_.*")
        .clang_arg("-xc++")
        .generate()
        .expect("Unable to generate bindings");

    let out_path = PathBuf::from(env::var("OUT_DIR").unwrap());
    let mut content = bindings.to_string();
    // Rust 2024 requires extern blocks to be unsafe
    content = content.replace("extern \"C\" {", "unsafe extern \"C\" {");
    std::fs::write(out_path.join("bindings.rs"), content).expect("Couldn't write bindings!");

    // Compile the C++ shim
    let mut build = cc::Build::new();
    build
        .cpp(true)
        .file("cpp/cgal_sp.cpp")
        .flag_if_supported("-std=c++17")
        .flag_if_supported("-fPIC");

    // Try pkg-config to get CGAL flags (includes + link paths)
    if let Ok(lib) = pkg_config::Config::new()
        .probe("CGAL")
        .or_else(|_| pkg_config::Config::new().probe("cgal"))
    {
        for inc in &lib.include_paths {
            build.include(inc);
        }
        for lp in &lib.link_paths {
            println!("cargo:rustc-link-search=native={}", lp.display());
        }
        for l in &lib.libs {
            println!("cargo:rustc-link-lib={}", l);
        }
    } else if let Ok(inc) = env::var("CGAL_INCLUDEDIR") {
        // Fallback to explicit include dir provided by environment
        build.include(&inc);
        // Try to infer libdir from includedir by replacing trailing "/include" with "/lib"
        if let Some(libdir) = inc.strip_suffix("/include").map(|p| format!("{}/lib", p)) {
            println!("cargo:rustc-link-search=native={}", libdir);
        }
    }

    // Try pkg-config for Boost (headers and link paths)
    if let Ok(lib) = pkg_config::Config::new().probe("boost") {
        for inc in &lib.include_paths {
            build.include(inc);
        }
        for lp in &lib.link_paths {
            println!("cargo:rustc-link-search=native={}", lp.display());
        }
        // Some setups require explicitly linking to boost_system
        println!("cargo:rustc-link-lib=boost_system");
    } else if let Ok(inc) = env::var("BOOST_INCLUDEDIR") {
        build.include(inc);
    }

    // Allow manual link-search hints from environment (useful in Nix shells)
    for var in ["CGAL_LIBDIR", "BOOST_LIBDIR", "GMP_LIBDIR", "MPFR_LIBDIR"] {
        if let Ok(dir) = env::var(var) {
            println!("cargo:rustc-link-search=native={}", dir);
        }
    }

    // Also try pkg-config for GMP/MPFR to get link-search paths
    if let Ok(lib) = pkg_config::Config::new().probe("gmp") {
        for lp in &lib.link_paths {
            println!("cargo:rustc-link-search=native={}", lp.display());
        }
    }
    if let Ok(lib) = pkg_config::Config::new().probe("mpfr") {
        for lp in &lib.link_paths {
            println!("cargo:rustc-link-search=native={}", lp.display());
        }
    }

    build.compile("cgal_sp");

    // Link libraries commonly needed by CGAL
    println!("cargo:rustc-link-lib=stdc++");
    // Only link CGAL if discovered via pkg-config (above). On some Nix builds, CGAL parts used here are header-only.
    // GMP/MPFR often required by CGAL
    println!("cargo:rustc-link-lib=gmp");
    println!("cargo:rustc-link-lib=mpfr");
}
