{
  description = "Motion Planning Research";
  inputs = {
    nixpkgs.url = "github:nixos/nixpkgs/nixos-unstable";
    pre-commit-hooks = { url = "github:cachix/pre-commit-hooks.nix"; };
    rust-overlay = { url = "github:oxalica/rust-overlay"; };
    crane = { url = "github:ipetkov/crane"; };
  };
  outputs = { self, nixpkgs, pre-commit-hooks, rust-overlay, crane }:
    let
      supportedSystems = [ "x86_64-linux" ];
      forAllSystems = nixpkgs.lib.genAttrs supportedSystems;
      pkgsFor = system: import nixpkgs {
        inherit system;
        overlays = [ rust-overlay.overlays.default ];
      };

      # Define the custom toolchain once
      rustToolchainFor = system: (pkgsFor system).rust-bin.stable.latest.default.override {
        extensions = [ "rust-src" "rustfmt" "clippy" ];
      };

      # Create a crane lib that uses our custom toolchain
      craneLibFor = system:
        (crane.mkLib (pkgsFor system)).overrideToolchain (rustToolchainFor system);
    in
    {
      formatter.x86_64-linux = nixpkgs.legacyPackages.x86_64-linux.nixpkgs-fmt;
      devShells = forAllSystems (system: {
        default = (pkgsFor system).callPackage ./shell.nix {
          inherit pre-commit-hooks;
          inherit system;
          toolchain = rustToolchainFor system;
        };
      });
      packages = forAllSystems (system:
        let
          pkgs = pkgsFor system;
          tree_models = pkgs.stdenv.mkDerivation {
            name = "tree_models";
            src = pkgs.fetchurl {
              url = "https://research.wernerkroneman.nl/tree_models.tar.xz";
              netrcPhase = ''
                # Create .netrc file with proper permissions
                cat > $PWD/netrc << EOF
                machine research.wernerkroneman.nl
                login research
                password "fruit is healthy"
                EOF
                chmod 600 $PWD/netrc
              '';
              sha256 = "sha256-UQCvkQ+czPQ85O6DPsSFagxrrEP6F4L7z+2ncfmIv7w=";
            };
            buildInputs = [ pkgs.xz ];
            buildCommand = ''
              mkdir -p $out
              tar -xf $src -C $out
            '';
          };
          # Get the crane lib with our custom toolchain
          craneLib = craneLibFor system;
        in
        {
          default = craneLib.buildPackage ({
            src = craneLib.cleanCargoSource ./.;
            buildInputs = with pkgs; [
              (rustToolchainFor system)
              alsa-lib
              cmake
              ffmpeg
              fontconfig
              libGL
              libxkbcommon
              pkg-config
              rustPlatform.bindgenHook
              rustPlatform.bindgenHook
              udev
              vulkan-loader
              xorg.libX11
              xorg.libXcursor
              xorg.libXi
              xorg.libXrandr
            ];
            nativeBuildInputs = with pkgs; [
              pkgs.pkg-config
            ];
            doCheck = false;
          });
          static_site = pkgs.stdenv.mkDerivation {
            name = "static_site";
            src = ./.;
            buildInputs = [ self.packages.${system}.default ];
            buildPhase = ''
              # Set the TREE_MODELS_PATH environment variable to point to the tree_models derivation
              export TREE_MODELS_PATH=${tree_models}/tree_models
              ${self.packages.${system}.default}/bin/motion_planning_research full_report
            '';
            installPhase = ''
              mkdir -p $out
              cp -r reports/* $out/
            '';
          };
        }
      );
    };
}
