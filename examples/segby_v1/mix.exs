defmodule SegbyV1.MixProject do
  use Mix.Project

  @moduledoc """
  The segby_v1 example app — a downstream CONSUMER of `:bb_mcuhub` (ADR-0003).

  It depends on the library via a Mix `path` dep (a hex/registry dep later, with
  no change to this app's shape) and owns its own root namespace (`SegbyV1.*`),
  referencing `BBMCUHub.*` only for library seams. It defines its OWN `Range` /
  `Led` value-types (the extension seam) and its own robot/hubs/controllers — the
  library carries the machinery; this app supplies device-specific logic.
  """

  def project do
    [
      app: :segby_v1,
      version: "0.1.0",
      elixir: "~> 1.18",
      elixirc_paths: elixirc_paths(Mix.env()),
      start_permanent: Mix.env() == :prod,
      deps: deps(),
      aliases: aliases(),
      name: "segby_v1",
      description: "segby_v1 — a self-balancing bot built on bb_mcuhub (the worked example)"
    ]
  end

  def application do
    [extra_applications: [:logger]]
  end

  defp elixirc_paths(:test), do: ["lib", "test/support"]
  defp elixirc_paths(_), do: ["lib"]

  defp deps do
    [
      # The library, as a downstream consumer pulls it (a path dep now; a registry
      # dep later, no shape change). circuits_uart rides in transitively (the
      # library's Host.Transport.UART uses it).
      {:bb_mcuhub, path: "../.."},
      # The BeamBots framework — the seam the robot + controllers sit on (§09).
      {:bb, "~> 0.20"},
      # The terminal dashboard over the BeamBots seam (§09). ONLY segby uses it, so
      # it lives here (the library no longer depends on bb_tui — ADR-0003). Pinned
      # to a fork (feat/consumer-renderers) that adds two GENERIC, upstreamable
      # extensions: configurable `:subscribe_paths` (feed the dashboard from the slow
      # `[:observe]` topic, not the control firehose) and a `:renderers` seam (a
      # consumer teaches the dashboard how to render its own payload — here
      # SegbyV1.ObserveRenderer renders our Observer.Sample; bb_tui stays generic).
      # ADR-0004. Both changes are upstreamable; see lostbean/bb_tui.
      {:bb_tui, github: "lostbean/bb_tui", branch: "feat/consumer-renderers"},
      # JSON codec for the sim Port wire to the MuJoCo Python child (ADR-0008).
      # The plant frames commands/state as JSON-per-line over the Port; this is a
      # sim-only concern of THIS consumer (the library ships no JSON dep).
      {:jason, "~> 1.4"},
      {:stream_data, "~> 1.0", only: [:dev, :test]}
    ]
  end

  defp aliases do
    [
      # Regenerate THIS app's wire artifacts into its OWN tree (ADR-0003: WireGen
      # takes an explicit output-base, so each app generates into its own dir). The
      # underlying task is the library's `wire.gen.run`; the `--gen-dir` /
      # `--fixtures-dir` flags point it at this example's firmware/gen +
      # test/fixtures. The drift test fails the build if a committed artifact
      # differs from this output, so the rule after any contract change is one
      # command: `mix wire.gen` + commit.
      "wire.gen": [
        "wire.gen.run --robot SegbyV1.Robot --slug segby_v1 " <>
          "--gen-dir firmware/gen --fixtures-dir test/fixtures"
      ]
    ]
  end
end
