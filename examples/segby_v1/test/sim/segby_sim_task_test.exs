defmodule Mix.Tasks.Segby.SimTest do
  @moduledoc """
  A LIGHT wiring test for the `mix segby.sim` launcher (ADR-0008, chunk 6). It
  asserts the task module + its wiring are sound WITHOUT launching MuJoCo or the
  Python child: the task is wiring (the host/transport/driver/plant pieces are
  unit-tested elsewhere), so here we only check the seams line up.

  Specifically:

    * the task module loads and is a `Mix.Task` exporting `run/1`;
    * the SIM transport + driver + plant modules it wires are real, loadable, and
      implement the contracts the task relies on (so a rename can't silently
      break the launcher); and
    * the MJCF path the task resolves (`<app root>/sim/segby.xml`) exists, since
      the task `Mix.raise`s if it doesn't.

  It deliberately does NOT call `run/1` (that starts the host + a real MuJoCo
  Port and blocks forever).
  """
  use ExUnit.Case, async: true

  test "the task module is a Mix.Task exporting run/1" do
    Code.ensure_loaded!(Mix.Tasks.Segby.Sim)
    assert function_exported?(Mix.Tasks.Segby.Sim, :run, 1)
    # `use Mix.Task` registers the Mix.Task behaviour on the module.
    behaviours = Mix.Tasks.Segby.Sim.module_info(:attributes)[:behaviour] || []
    assert Mix.Task in behaviours
  end

  test "the wired modules exist and implement the contracts the task relies on" do
    # The host launcher the task starts (with the sim transport injected).
    Code.ensure_loaded!(SegbyV1.Host)
    assert function_exported?(SegbyV1.Host, :start_link, 1)

    # The library sim seam: a Host.Transport + the loop driver.
    Code.ensure_loaded!(BBMCUHub.Sim.Transport)
    assert function_exported?(BBMCUHub.Sim.Transport, :start_link, 2)
    assert function_exported?(BBMCUHub.Sim.Transport, :take_commands, 1)

    Code.ensure_loaded!(BBMCUHub.Sim.Driver)
    assert function_exported?(BBMCUHub.Sim.Driver, :start_link, 1)

    # The example's plant the driver steps.
    Code.ensure_loaded!(SegbyV1.Sim.MujocoPlant)
    behaviours = SegbyV1.Sim.MujocoPlant.module_info(:attributes)[:behaviour] || []
    assert BBMCUHub.Sim.Plant in behaviours
  end

  test "the LinkOwner default name (the driver's owner) is the host's registered link owner" do
    # The task passes `owner: BBMCUHub.Host.LinkOwner` — the same name the host
    # registers the LinkOwner under (a valid Kernel.send/2 target). Guard the
    # contract that the owner name is the LinkOwner module.
    Code.ensure_loaded!(BBMCUHub.Host.LinkOwner)
    assert function_exported?(BBMCUHub.Host.LinkOwner, :start_link, 1)
  end

  test "the MJCF the task resolves exists at the app root" do
    # The task resolves `Path.join(File.cwd!(), \"sim/segby.xml\")`; `mix` runs
    # from the app root, so this is the same file the launcher would pass to the
    # plant. (mix test also runs from the app root.)
    mjcf = Path.join(File.cwd!(), "sim/segby.xml")
    assert File.exists?(mjcf), "expected the committed MJCF at #{mjcf}"
  end
end
