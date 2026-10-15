defmodule BBMCUHub.MixProject do
  use Mix.Project

  def project do
    [
      app: :bb_mcuhub,
      version: "0.1.0",
      elixir: "~> 1.19",
      elixirc_paths: elixirc_paths(Mix.env()),
      # The generated parity vectors are data the C parity test reads, not a
      # test file — tell `mix test` so it doesn't warn about the .exs name.
      test_ignore_filters: [~r{/parity_vectors\.exs$}],
      start_permanent: Mix.env() == :prod,
      deps: deps(),
      aliases: aliases(),
      # The virtual-hub NIF (test-only, Option B) wraps the real firmware C floor +
      # wire path so the soft-fault e2e runs the ACTUAL safety code. Built by
      # elixir_make ONLY in :test; never shipped (see test/support/c_src/Makefile).
      make_targets: ["all"],
      make_clean: ["clean"],
      make_cwd: "test/support/c_src",
      compilers:
        if(Mix.env() == :test, do: [:elixir_make | Mix.compilers()], else: Mix.compilers()),
      description: description(),
      package: package(),
      name: "bb_mcuhub",
      source_url: "https://github.com/lostbean/bb_mcuhub",
      docs: docs()
    ]
  end

  def application do
    [
      extra_applications: [:logger],
      mod: {BBMCUHub.Application, []}
    ]
  end

  # Run the `ci` and `wire.gen` aliases under MIX_ENV=test end-to-end: their
  # `compile` step must see test/support (the fixture robot), and `ci`'s `test`
  # step must not run in :dev.
  def cli do
    [preferred_envs: [ci: :test, "wire.gen": :test]]
  end

  # The platform lives in lib/; the library's own tests are backed by the fixture
  # robot under test/support (the design's strata, §10). The worked example
  # (segby_v1, blaster/wheels) now lives in its own Mix app under examples/ and
  # depends on this library as a downstream consumer (ADR-0003 / Phase 5).
  defp elixirc_paths(:test), do: base_paths() ++ ["test/support"]
  defp elixirc_paths(_), do: base_paths()

  defp base_paths do
    # Follower + the imu/motor hubs are retired (ADR-0003); segby_v1 moved out to
    # examples/segby_v1/. The library is a self-contained chassis: all its code is
    # in lib/, and its tests are backed by the fixture robot under test/support
    # (added by elixirc_paths(:test)).
    ["lib"]
  end

  defp deps do
    [
      # The BeamBots framework — the seam the hub views sit on (§09). A control
      # loop gates its output on disarm via the documented `[:state_machine]`
      # subscription, so no framework fork is needed (ADR-0010). `BB_VERSION` lets
      # the beam-bots workspace run this package against an in-development `bb`
      # (`local` → `../bb`) in its cross-package integration tests.
      {:bb, bb_dep("~> 0.25")},
      # The host owns a UART to the root hub (§07); Circuits.UART provides the
      # framing behaviour our COBS+CRC framer implements (§03). A consumer pulls
      # this in transitively (Host.Transport.UART uses it).
      {:circuits_uart, "~> 1.5"},
      # NOTE: bb_tui moved to the example app (ADR-0003 / Phase 5) — only the
      # worked example (segby_v1) uses the dashboard, so the library no longer
      # depends on it.
      {:stream_data, "~> 1.0", only: [:dev, :test]},
      # Builds the test-only virtual-hub NIF (Option B). Already a transitive dep
      # via circuits_uart, so this is not a new shipped dependency; the NIF itself
      # is built only in :test (see the :compilers gate in project/0).
      {:elixir_make, "~> 0.8", runtime: false},
      {:ex_check, "~> 0.16", only: [:dev, :test], runtime: false},
      {:ex_doc, "~> 0.34", only: :dev, runtime: false}
    ]
  end

  # Resolve the `bb` dependency against hex, a sibling checkout, or `bb`'s main
  # branch depending on `BB_VERSION` — the beam-bots ecosystem convention that
  # lets the workspace's integration tests run this package against an
  # in-development `bb` (`BB_VERSION=local` → `../bb`).
  defp bb_dep(default) do
    case System.get_env("BB_VERSION") do
      nil -> default
      "local" -> [path: "../bb", override: true]
      "main" -> [git: "https://github.com/beam-bots/bb.git", override: true]
      version -> "~> #{version}"
    end
  end

  defp aliases do
    [
      # Regenerate every artifact (C header, per-hub glue, parity vectors) from
      # the contracts + topology — see §06 / ADR-0003. The drift test fails the
      # build if any committed artifact differs from this output.
      #
      # The library's committed robot is its TEST FIXTURE (the drift/C-parity
      # witness), which lives under test/support and is compiled only in :test. So
      # generation must run in the test env to see it — cli/0's preferred_envs
      # runs `mix wire.gen` under MIX_ENV=test for you. (Not `mix cmd
      # MIX_ENV=test ...`: Mix >= 1.19 no longer shells out, so an env-var
      # prefix would be exec'd as a program name.) The worked example (segby_v1)
      # generates its OWN artifacts from its own app (it passes its own output
      # base; see examples/segby_v1/mix.exs). Implemented by
      # `Mix.Tasks.Wire.Gen.Run` (supports `--robot <Mod>` + `--gen-dir`/
      # `--fixtures-dir`).
      "wire.gen": ["compile", "wire.gen.run"],
      # The one-command local gate — the same checks CI runs (see
      # .github/workflows/ci.yml): formatting, a clean warnings-as-errors compile,
      # and the full suite (which itself builds the C NIF + runs the C parity and
      # drift tests). Run it before pushing. The whole alias runs under MIX_ENV=test
      # (via cli/0's preferred_envs), so the compile sees test/support and the test
      # step doesn't trip Mix's "tests in :dev" guard.
      ci: [
        "format --check-formatted",
        "compile --warnings-as-errors --force",
        # --warnings-as-errors here too: `compile` above only sees lib/ + deps, so
        # without this a warning in a TEST file (compiled by the test task) would
        # slip through the gate.
        "test --warnings-as-errors"
      ]
    ]
  end

  defp description do
    "bb_mcuhub — reach microcontroller hardware from the BeamBots ecosystem " <>
      "through one recursive abstraction: the hub. A correct framed wire, a " <>
      "fail-passive floor, clock-free freshness, and a generated contract."
  end

  defp package do
    [
      licenses: ["Apache-2.0"],
      # Ship the prose the docs config lists as extras — hexdocs builds from the
      # package tarball, so an extra not in `files` silently vanishes there.
      files: ~w(lib mix.exs README.md LICENSE docs/design/CONTEXT.md docs/NEW-ROBOT.md),
      # Absolute URLs — Hex renders these on the package page (relative paths 404).
      links: %{
        "GitHub" => "https://github.com/lostbean/bb_mcuhub",
        "Design" => "https://github.com/lostbean/bb_mcuhub/blob/main/docs/design/design.md"
      }
    ]
  end

  # `mix docs` — make hexdocs land on the README and carry the two prose docs a
  # consumer needs next (the glossary + the build-your-own-robot walkthrough).
  # The 38 lib/ modules all have @moduledoc; the groups give the sidebar the
  # same shape as the README's repository-layout map.
  defp docs do
    [
      main: "readme",
      extras: [
        "README.md",
        "docs/design/CONTEXT.md": [title: "Glossary (CONTEXT)"],
        "docs/NEW-ROBOT.md": [title: "Build your own robot"]
      ],
      source_ref: "main",
      # Render ```mermaid fences as diagrams (the ex_doc-documented recipe) —
      # without this the README's port-flow diagram is a dead code block.
      before_closing_body_tag: &before_closing_body_tag/1,
      groups_for_modules: [
        "DSL & model": [
          BBMCUHub.Dsl,
          BBMCUHub.Hub,
          ~r/^BBMCUHub\.(Dsl|Hub|Robot)\./
        ],
        "Value-types": [BBMCUHub.ValueType, ~r/^BBMCUHub\.ValueType\./],
        Host: [BBMCUHub.Host, ~r/^BBMCUHub\.Host\./],
        "BeamBots seam": [~r/^BBMCUHub\.BBHub\./],
        Wire: [~r/^BBMCUHub\.Wire\./],
        Contract: [BBMCUHub.Contract, ~r/^BBMCUHub\.Contract\./],
        Generator: [~r/^BBMCUHub\.Gen\./, Mix.Tasks.Wire.Gen.Run],
        Sim: [~r/^BBMCUHub\.Sim\./],
        Observer: [BBMCUHub.Observer, ~r/^BBMCUHub\.Observer\./]
      ]
    ]
  end

  # The ex_doc-documented Mermaid hook: render each ```mermaid fence into an
  # SVG after the page loads (theme-aware), replacing the code block.
  defp before_closing_body_tag(:html) do
    """
    <script defer src="https://cdn.jsdelivr.net/npm/mermaid@11/dist/mermaid.min.js"></script>
    <script>
      let initialized = false;
      window.addEventListener("exdoc:loaded", () => {
        if (!initialized) {
          mermaid.initialize({
            startOnLoad: false,
            theme: document.body.className.includes("dark") ? "dark" : "default"
          });
          initialized = true;
        }
        let id = 0;
        for (const codeEl of document.querySelectorAll("pre code.mermaid")) {
          const preEl = codeEl.parentElement;
          const graphDefinition = codeEl.textContent;
          const graphEl = document.createElement("div");
          const graphId = "mermaid-graph-" + id++;
          mermaid.render(graphId, graphDefinition).then(({svg, bindFunctions}) => {
            graphEl.innerHTML = svg;
            bindFunctions?.(graphEl);
            preEl.insertAdjacentElement("afterend", graphEl);
            preEl.remove();
          });
        }
      });
    </script>
    """
  end

  defp before_closing_body_tag(_), do: ""
end
