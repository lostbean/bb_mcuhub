defmodule SegbyV1Nerves.MixProject do
  @moduledoc """
  Nerves firmware wrapper that runs the `segby_v1` host on a Raspberry Pi
  Zero 2 W.

  This is a MINIMAL Nerves project (ADR-0003 keeps the example app pure OTP;
  this wrapper is the hardware-phase shell that boots it under Nerves). It is a
  sibling-in-a-subdir of `examples/segby_v1/` — its OWN Mix project, NOT a path
  dep of `:segby_v1`, so it never interferes with `cd examples/segby_v1 && mix
  test`.

  ## What it does

  On `MIX_TARGET=rpi0_2` it boots straight into `SegbyV1.Host` talking to the
  Blaster (the root hub, NODE 0x02) over `/dev/ttyAMA0` at 115200 baud. It carries
  no web endpoint or config system — just the Nerves network + OTA + ssh stack.

  ## Nested path deps (how they resolve)

  Deps chain from THIS project (`examples/segby_v1/nerves_host/`):

    * `{:segby_v1, path: ".."}`        -> `examples/segby_v1/`
    * which declares `{:bb_mcuhub, path: "../.."}` -> repo root (resolved
      relative to `examples/segby_v1/`, i.e. `examples/segby_v1/../..`).

  `:segby_v1` transitively pulls `:bb`, `:bb_tui`, and `:circuits_uart` (the
  library's `Host.Transport.UART` uses circuits_uart). We also list
  `:circuits_uart` directly so the dep is explicit for the firmware build.

  ## UART invariants (DO NOT regress — hardware-verified)

    * `dtoverlay=miniuart-bt` in `config/config.txt` — puts the real PL011 on
      GPIO 14/15 as `/dev/ttyAMA0`. `disable-bt` silently renames it to ttyAMA1.
    * `console=tty1` in `config/cmdline-{a,b}.txt` — drops
      `console=serial0,115200` so the kernel console doesn't fight
      `circuits_uart` for the same UART device.
    * `config :nerves, :firmware, fwup_conf: "config/fwup.conf"` in
      `config/target.exs` — registers the per-project fwup override that
      substitutes this project's `config.txt` + `cmdline-{a,b}.txt`. Without it,
      fwup writes the stock files from the system squashfs and the console
      fights us for ttyAMA0.
  """
  use Mix.Project

  @app :segby_v1_nerves
  @version "0.1.0"
  @all_targets [:rpi0_2]

  def project do
    [
      app: @app,
      version: @version,
      elixir: "~> 1.19",
      archives: [nerves_bootstrap: "~> 1.15"],
      start_permanent: Mix.env() == :prod,
      deps: deps(),
      releases: [{@app, release()}],
      preferred_cli_target: [run: :host, test: :host]
    ]
  end

  # Run type "mix help compile.app" to learn about applications.
  def application do
    [
      extra_applications: [:logger, :runtime_tools],
      mod: {SegbyV1Nerves.Application, []}
    ]
  end

  # Run type "mix help deps" to learn about dependencies.
  defp deps do
    [
      # Dependencies for all targets
      {:nerves, "~> 1.13", runtime: false},
      {:shoehorn, "~> 0.9"},
      {:ring_logger, "~> 0.11"},
      {:toolshed, "~> 0.4"},

      # The example app — pure OTP, depends transitively on :bb_mcuhub (repo
      # root), :bb, :bb_tui, and :circuits_uart. Its path dep {:bb_mcuhub, path:
      # "../.."} resolves relative to examples/segby_v1/, so from here the chain
      # is :segby_v1 at .. -> bb_mcuhub at ../.. = the repo root.
      {:segby_v1, path: ".."},

      # UART transport for SegbyV1.Host (Pi <-> Blaster over GPIO 14/15). Rides
      # in transitively via the library too; listed here so it's explicit.
      {:circuits_uart, "~> 1.5"},

      # Allow Nerves.Runtime on host for development, testing and CI.
      {:nerves_runtime, "~> 0.13"},

      # Dependencies for all targets except :host
      {:nerves_pack, "~> 0.7", targets: @all_targets},

      # Dependencies for specific targets
      # NOTE: It's generally low risk and recommended to follow minor version
      # bumps to Nerves systems. Since these include Linux kernel and Erlang
      # version updates, please review their release notes in case
      # changes to your application are needed.
      {:nerves_system_rpi0_2, "~> 2.0", runtime: false, targets: :rpi0_2}
    ]
  end

  def release do
    [
      overwrite: true,
      # Erlang distribution is not started automatically.
      # See https://hexdocs.pm/nerves_pack/readme.html#erlang-distribution
      cookie: "#{@app}_cookie",
      include_erts: &Nerves.Release.erts/0,
      steps: [&Nerves.Release.init/1, :assemble],
      strip_beams: Mix.env() == :prod or [keep: ["Docs"]]
    ]
  end
end
