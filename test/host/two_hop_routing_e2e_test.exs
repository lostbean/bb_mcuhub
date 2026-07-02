defmodule BBMCUHub.Host.TwoHopRoutingE2ETest do
  @moduledoc """
  The two-hop topology (host → root hub → UART leaf), end-to-end through the
  **real firmware C router** (`VHubNif.router_route/5` inside the
  `BBMCUHub.Test.VirtualHub`'s two-hop mode) — the regression e2e for issue #9.

  The fixture robot IS this shape: `sensor_hub` (node 0x02) is the root on the
  host UART; `act_hub` (node 0x05) hangs behind it over a UART downlink. Every
  frame crossing the root here runs the actual C `router_route` with its
  arrival link, exactly as the root firmware relays between its links:

    * **down**: a host command for the leaf's `effort_cmd` arrives on the
      root's up-link and must be dispatched onto downlink 1 to reach the leaf's
      real C floor;
    * **up**: the leaf's `act_status` bodies arrive at the root on downlink 1
      and reach the host ONLY if the router forwards them up (link 0). The
      pre-fix router bounced them back down the downlink they arrived on
      (`route_table[node]` with a SOURCE node), leaving the leaf invisible to
      the host — sensor slots never populated, the bot could not be armed.

  No hardware, explicit simulated time, count-/value-based assertions — the
  coverage the MuJoCo sim structurally can't provide (it bypasses the C router).
  """
  use ExUnit.Case, async: false

  alias BBMCUHub.Contract.PortIndex
  alias BBMCUHub.Host.{LinkOwner, NodeRegistry}
  alias BBMCUHub.Test.VirtualHub
  alias BBMCUHub.Wire.Stats

  @robot BBMCUHub.Test.Fixtures.Robot
  @window_ms 100

  setup do
    NodeRegistry.reset()
    PortIndex.build(@robot)
    Stats.setup()

    {:ok, {root_node, pose_port}} = PortIndex.resolve(:sensor_hub, :pose)
    {:ok, {leaf_node, eff_port}} = PortIndex.resolve(:act_hub, :effort_cmd)
    {:ok, {^leaf_node, status_port}} = PortIndex.resolve(:act_hub, :act_status)

    # The real LinkOwner over a two-hop VirtualHub: the leaf's floored actuator
    # sits behind the root's REAL C router (downlink 1), as on the bench.
    {:ok, owner} =
      LinkOwner.start_link(
        transport: VirtualHub,
        transport_opts: [
          ports: [{leaf_node, eff_port, @window_ms, status_port}],
          router: %{my_node: root_node, downlinks: %{leaf_node => 1}}
        ],
        command_slots: [{leaf_node, eff_port}],
        name: nil
      )

    vhub = :sys.get_state(owner).transport
    on_exit(fn -> if Process.alive?(owner), do: GenServer.stop(owner) end)

    {:ok,
     owner: owner,
     vhub: vhub,
     root_node: root_node,
     pose_port: pose_port,
     leaf_node: leaf_node,
     eff_port: eff_port,
     status_port: status_port}
  end

  defp command(ctx, seq, nm) do
    NodeRegistry.put(ctx.leaf_node, ctx.eff_port, %{nm: nm}, seq, 0)
    LinkOwner.notify_command_slot(ctx.owner, ctx.leaf_node, ctx.eff_port)
    _ = :sys.get_state(ctx.owner)
    _ = :sys.get_state(ctx.vhub)
    :ok
  end

  defp tick_and_read(ctx, now_ms) do
    VirtualHub.tick(ctx.vhub, now_ms)
    _ = :sys.get_state(ctx.owner)

    case NodeRegistry.get(ctx.leaf_node, ctx.status_port) do
      {%{floored: floored?}, _seq, _t} -> floored?
      nil -> :no_status
    end
  end

  test "the leaf's status ascends through the root's C router to the host", ctx do
    # Before the fix this stayed :no_status forever: the root's router bounced
    # every node-0x05 status body back down downlink 1 (its route-table entry),
    # so the host never saw the leaf at all — the exact bench symptom.
    command(ctx, 1, 0.4)

    refute tick_and_read(ctx, 10) == :no_status,
           "the leaf's act_status must reach the host through the root router"
  end

  test "the full two-hop loop: command down through the router, armed status back up", ctx do
    # Descend: host → root router (arrival: up-link) → downlink 1 → the leaf's
    # REAL C floor. Ascend: the floor's status → root router (arrival: downlink
    # 1) → up-link → host registry. Both directions cross the real C router.
    command(ctx, 1, 0.4)
    assert tick_and_read(ctx, 10) == true, "born-disarmed until a witnessed advance"

    command(ctx, 2, 0.4)

    assert tick_and_read(ctx, 20) == false,
           "a witnessed seq advance arms the leaf floor — two hops each way"

    # And the safety loop still holds across the relay: command silence past the
    # window floors the leaf, and the host SEES it floor.
    assert tick_and_read(ctx, 200) == true, "command silence floors the leaf, visibly"
  end

  test "the root's own sensor still ascends (link_send_up bypasses the router)", ctx do
    pose = %{
      qw: 1.0,
      qx: 0.0,
      qy: 0.0,
      qz: 0.0,
      wx: 0.0,
      wy: 0.0,
      wz: 0.0,
      ax: 0.0,
      ay: 0.0,
      az: 9.81
    }

    VirtualHub.emit_sensor(ctx.vhub, ctx.root_node, ctx.pose_port, 7, {:imu, pose, true})
    _ = :sys.get_state(ctx.owner)

    assert {%{az: az}, 7, _t} = NodeRegistry.get(ctx.root_node, ctx.pose_port)
    assert_in_delta az, 9.81, 0.001
  end

  test "a leaf sensor body emitted behind the root ascends through the router", ctx do
    # A scripted leaf-origin body (the leaf's own status port, as its firmware
    # emits it) injected at the DOWNLINK, not at the host seam: it must cross
    # the real C router to be seen.
    VirtualHub.emit_sensor(
      ctx.vhub,
      ctx.leaf_node,
      ctx.status_port,
      3,
      {:status, %{applied_seq: 3, floored: false}, false}
    )

    _ = :sys.get_state(ctx.owner)

    assert {%{applied_seq: 3, floored: false}, 3, _t} =
             NodeRegistry.get(ctx.leaf_node, ctx.status_port)
  end
end
