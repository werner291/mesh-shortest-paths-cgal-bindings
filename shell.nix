{ pkgs, pre-commit-hooks, system, toolchain }:
let
  pre-commit-check = pre-commit-hooks.lib.${system}.run {
    src = ./.;
    hooks = {
      nixpkgs-fmt.enable = true;
      rustfmt = {
        enable = true;
        package = toolchain;
      };
    };
  };
  # Ensure we can reference the dev outputs for proper include paths
  cgalDev = pkgs.lib.getDev pkgs.cgal;
  boostDev = pkgs.lib.getDev pkgs.boost;
  # Library outputs for link-search
  cgalLib = pkgs.lib.getLib pkgs.cgal;
  boostLib = pkgs.lib.getLib pkgs.boost;
  gmpLib = pkgs.lib.getLib pkgs.gmp;
  mpfrLib = pkgs.lib.getLib pkgs.mpfr;
  llvm = pkgs.llvmPackages;
in
pkgs.mkShell rec {
  buildInputs = with pkgs; [
    toolchain
    xorg.libXcursor
    xorg.libXrandr
    xorg.libXi
    xorg.libX11
    libxkbcommon
    libGL
    alsa-lib
    pkg-config
    cmake
    udev
    vulkan-loader
    rustPlatform.bindgenHook
    ffmpeg
    fontconfig
    cgal
    boost
    gmp
    mpfr
    llvm.clang
    (pkgs.python312.withPackages (python-pkgs: with python-pkgs; [
      pandas
      numpy
      matplotlib
      seaborn
      msgpack #
    ]))
    bashInteractive
  ] ++ pre-commit-check.enabledPackages;
  # Help cc::Build find headers in case pkg-config for CGAL is not present
  BOOST_INCLUDEDIR = "${boostDev}/include";
  CGAL_INCLUDEDIR = "${cgalDev}/include";
  CPLUS_INCLUDE_PATH = builtins.concatStringsSep ":" [ CGAL_INCLUDEDIR BOOST_INCLUDEDIR ];

  # Help rustc/cc find libraries during linking
  CGAL_LIBDIR = "${cgalLib}/lib";
  BOOST_LIBDIR = "${boostLib}/lib";
  GMP_LIBDIR = "${gmpLib}/lib";
  MPFR_LIBDIR = "${mpfrLib}/lib";

  LD_LIBRARY_PATH = "${pkgs.lib.makeLibraryPath buildInputs}";
  RUST_SRC_PATH = "${toolchain}/lib/rustlib/src/rust/library";
  shellHook = ''
    ${pre-commit-check.shellHook}
  '';
}
