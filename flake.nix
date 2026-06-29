{
  # bb_mcuhub dev environment.
  #
  # Two strata live in this repo (see CONTEXT.md / docs/hub-design.html):
  #   - Elixir host code  (Mix app :bb_mcuhub, elixir ~> 1.19) — `mix test`
  #   - C / ESP32 firmware (firmware/, PlatformIO) — `pio run`, plus host-compiled
  #     C test harnesses under firmware/test/ built with a Makefile (cc/clang).
  #
  # This devShell pins both toolchains so any git worktree of this repo gets the
  # same env reproducibly — crucial because the legacy firmware/.pio-venv is
  # gitignored and its launcher hardcodes an absolute main-tree path, so it does
  # NOT exist in a fresh worktree. With this flake, `nix develop` (or direnv)
  # gives a worktree subagent `mix`, `pio`, `cc`, and `make` out of the box.
  description = "bb_mcuhub — Elixir host + ESP32 firmware dev environment";

  inputs = {
    # nixos-unstable: it ships the exact toolchain versions this project targets
    # — elixir 1.19.5 (~> 1.19), platformio 6.1.19 (matches the .pio-venv core),
    # clang 21.x, lefthook. (Switch to "nixos-25.05" for the stable channel.)
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";

    flake-utils.url = "github:numtide/flake-utils";

    treefmt-nix.url = "github:numtide/treefmt-nix";
    treefmt-nix.inputs.nixpkgs.follows = "nixpkgs";
  };

  outputs =
    {
      nixpkgs,
      flake-utils,
      treefmt-nix,
      ...
    }:
    flake-utils.lib.eachDefaultSystem (
      system:
      let
        pkgs = nixpkgs.legacyPackages.${system};

        # macOS needs a libpython symlink so the example's `mjpython` viewer works
        # (ADR-0008); the shellHook below is guarded on this.
        isDarwin = builtins.match ".*-darwin" system != null;

        # treefmt config — one formatter per language in the repo.
        treefmtEval = treefmt-nix.lib.evalModule pkgs {
          projectRootFile = "flake.nix";

          programs.mix-format.enable = true; # Elixir — uses .formatter.exs
          programs.nixfmt.enable = true; # Nix — RFC-style (official)
          programs.prettier.enable = true; # Markdown / YAML / JSON
          programs.shfmt.enable = true; # Shell
          programs.clang-format.enable = true; # C / C++ firmware

          settings.global.excludes = [
            "_build/**"
            "deps/**"
            ".pio/**"
            ".pio-core/**"
            ".pio-venv/**"
            "firmware/.pio/**"
            "docs/**/*.html" # generated design docs — leave untouched
            # GENERATED wire artifacts — the generator's output IS canonical;
            # formatting them breaks the drift test (committed must equal emitted).
            # treefmt globs are project-root-relative, so list both trees.
            "firmware/gen/**"
            "examples/*/firmware/gen/**"
            "test/fixtures/**/parity_vectors.exs"
            "examples/*/test/fixtures/**/parity_vectors.exs"
            "*.lock"
            "mix.lock"
          ];
        };
      in
      {
        devShells.default = pkgs.mkShell {
          packages = with pkgs; [
            # --- Elixir host stratum ---
            # Elixir 1.19 on OTP 28 (both pinned to erlang_28 so `mix` and the
            # standalone BEAM agree). OTP 28 is required by nerves_system_rpi0_2
            # ~> 2.0 (the OTP-28 line the Pi runs); a 1.19-on-OTP-27 elixir makes
            # `mix firmware` fail the host/target OTP-major check. Satisfies the
            # apps' `elixir ~> 1.19` (the `bb` dep >= 0.22 requires ~> 1.19).
            beam.packages.erlang_28.elixir_1_19 # Elixir 1.19.x on OTP 28
            erlang_28 # OTP 28 — matches the elixir above + nerves rpi0_2 2.x

            # --- C / ESP32 firmware stratum ---
            platformio # `pio run` for the ESP32 envs (6.1.x)
            clang # cc/clang for firmware/test host harnesses
            gnumake # `cd firmware/test && make`
            pkg-config # vintage_net_wifi's host NIF needs it (Nerves firmware build)
            # Nerves host tooling for `mix firmware` (assemble the rpi0_2 .fw image):
            fwup # assembles + burns/uploads the .fw image (mix firmware / upload)
            squashfsTools # mksquashfs — the root filesystem image
            xz # firmware payload compression
            coreutils-prefixed # GNU coreutils as g-prefixed (Nerves wants `gstat` on macOS)

            # --- example sim stratum (ADR-0008: virtual robot, EXAMPLE-only) ---
            # The segby_v1 example can run virtually against MuJoCo. MuJoCo itself
            # comes from PyPI into a project-local .venv via uv (it does not build
            # on Darwin via nixpkgs); nix only supplies the interpreter + uv. The
            # shipped library/firmware/host runtime are untouched — this is opt-in
            # tooling for `examples/segby_v1/sim` (its pyproject.toml + Python child).
            python312 # the interpreter uv's .venv is built from (mjpython wheels)
            uv # resolves examples/segby_v1/sim/pyproject.toml into a .venv

            # --- shared dev tooling ---
            lefthook # pre-commit format gate (run `lefthook install`)
            git
          ];

          # PlatformIO downloads its platform/toolchains into PLATFORMIO_CORE_DIR
          # at first `pio run`. Default it to a worktree-local, gitignored path so
          # each worktree keeps its own core dir and never touches the main tree's
          # .pio-core. The existing manual workflow that sets PLATFORMIO_CORE_DIR
          # explicitly still wins (we only set it if unset).
          shellHook = ''
            export PLATFORMIO_CORE_DIR="''${PLATFORMIO_CORE_DIR:-$PWD/.pio-core}"
            echo "bb_mcuhub devShell: elixir $(elixir --version | tail -1 | cut -d' ' -f2), $(pio --version)"
            echo "  PLATFORMIO_CORE_DIR=$PLATFORMIO_CORE_DIR"
          ''
          # macOS: the example sim's `mjpython` re-execs the .venv python from inside
          # a Cocoa .app bundle, then dlopens libpython3.12.dylib relative to it via
          # @executable_path/../lib/. uv's .venv layout doesn't ship that dylib, so
          # symlink the one from the uv-managed interpreter (ADR-0008). Scoped to the
          # example's sim .venv; a no-op until `uv sync` has created it.
          + pkgs.lib.optionalString isDarwin ''

            sim_venv="$PWD/examples/segby_v1/sim/.venv"
            if [ -f "$sim_venv/bin/python" ] && [ ! -e "$sim_venv/lib/libpython3.12.dylib" ]; then
              real_python=$(readlink -f "$sim_venv/bin/python" 2>/dev/null || true)
              if [ -n "$real_python" ]; then
                py_lib_dir="$(dirname "$(dirname "$real_python")")/lib"
                if [ -f "$py_lib_dir/libpython3.12.dylib" ]; then
                  ln -sf "$py_lib_dir/libpython3.12.dylib" "$sim_venv/lib/libpython3.12.dylib"
                  echo "  [sim] linked libpython3.12.dylib into $sim_venv/lib (mjpython)"
                fi
              fi
            fi
          '';
        };

        # `nix fmt` runs treefmt across the repo.
        formatter = treefmtEval.config.build.wrapper;

        # `nix flake check` verifies everything is formatted.
        checks.formatting = treefmtEval.config.build.check ./.;
      }
    );
}
