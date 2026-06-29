defmodule BBMCUHub.MixProject do
  use Mix.Project

  def project do
    [
      app: :bb_mcuhub,
      version: "0.1.0",
      elixir: "~> 1.19",
      elixirc_paths: elixirc_paths(Mix.env()),
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
      source_url: "https://github.com/lostbean/bb_mcuhub"
    ]
  end

  def application do
    [
      extra_applications: [:logger],
      mod: {BBMCUHub.Application, []}
    ]
  end

  # Run the `ci` alias under MIX_ENV=test end-to-end: its `compile` step must see
  # test/support (the fixture robot) and its `test` step must not run in :dev.
  def cli do
    [preferred_envs: [ci: :test]]
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
      # The BeamBots framework — the seam the hub views sit on (§09). Pinned to the
      # lostbean fork for a safety fix: BB.Controller gains handle_safety_state_change
      # so a long-lived control loop can gate its output on disarm (the BB.Command
      # path already had this; controllers did not — see ADR-0010 + beam-bots/bb#160).
      {:bb, github: "lostbean/bb", branch: "feat/controller-safety-state-hook", override: true},
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
      {:ex_doc, "~> 0.34", only: :dev, runtime: false}
    ]
  end

  defp aliases do
    [
      # Regenerate every artifact (C header, per-hub glue, parity vectors) from
      # the contracts + topology — see §06 / ADR-0003. The drift test fails the
      # build if any committed artifact differs from this output.
      #
      # The library's committed robot is its TEST FIXTURE (the drift/C-parity
      # witness), which lives under test/support and is compiled only in :test. So
      # generation must run in the test env to see it — `mix wire.gen` chains the
      # compile+task under MIX_ENV=test for you. The worked example (segby_v1)
      # generates its OWN artifacts from its own app (it passes its own output
      # base; see examples/segby_v1/mix.exs). Implemented by
      # `Mix.Tasks.Wire.Gen.Run` (supports `--robot <Mod>` + `--gen-dir`/
      # `--fixtures-dir`).
      "wire.gen": ["cmd MIX_ENV=test mix do compile + wire.gen.run"],
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
      # Absolute URLs — Hex renders these on the package page (relative paths 404).
      links: %{
        "GitHub" => "https://github.com/lostbean/bb_mcuhub",
        "Design" => "https://github.com/lostbean/bb_mcuhub/blob/main/docs/hub-design.html"
      }
    ]
  end
end
