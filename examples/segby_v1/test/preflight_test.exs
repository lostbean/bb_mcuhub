defmodule SegbyV1.PreflightTest do
  @moduledoc """
  Tier-0 hardware-in-the-loop **pre-flight** checks (see `BRINGUP.md`) — the
  software-side guarantees that, if wrong, would destroy hardware when you flash.
  These run the WHOLE host stack over the loopback transport (real COBS+CRC seam,
  no boards), and assert the **safety- and sign-critical** behaviours the
  happy-path `SegbyV1.HostTest` does not:

    1. **Pitch-sign self-consistency (host side)** — a forward tilt and a backward
       tilt must produce *opposite-sign* torques, and the host convention
       (+pitch → negative torque) must hold. This catches a flipped sign in the
       host control code. It does NOT certify the bot balances: the ABSOLUTE sign
       that decides "drive under the fall vs. amplify it" closes only through the
       motor phase wiring + encoder direction (`encoder.invert`) — a bench
       calibration (BRINGUP Stage 3/4), not a software constant. Verify pose-pitch
       tracks tilt with balance OFF, then confirm the wheel drives the correct way
       at low effort BEFORE enabling balance.
    2. **Born-stale / born-disarmed** — a leftover or single (non-advancing) pose
       must NOT be trusted; the controller produces no balance torque until it
       personally witnesses a `seq` advance since its own boot.
    3. **Command cadence keeps the on-chip floor armed** — while pose flows, the
       host emits commands fast enough (≤ the 100 ms floor window) that a real
       wheel's floor would stay armed; if the host stalls, the wheel floors. The
       floor itself is on the MCU (proven by the C harnesses); this asserts the
       host feeds it in time.
    4. **Status liveness is freshness-gated** — a stale "not floored" status reads
       as unknown, never as driving (so the dashboard can't show a false green).

  The companion `SegbyV1.GoldenFramesTest` captures the exact bytes the host emits
  for a known command, so you can diff a real board's RX against them at the bench.

  Run with: `mix test test/preflight_test.exs` (before flashing anything).
  """
  use ExUnit.Case, async: false

  alias BB.Math.{Quaternion, Vec3}
  alias BBMCUHub.Contract.PortIndex
  alias BBMCUHub.Host.{LinkOwner, NodeRegistry}
  alias BBMCUHub.Wire.Codec
  alias SegbyV1.{Balance, Host, Robot}
  alias SegbyV1.Test.LoopbackTransport

  @robot Robot
  @g 9.81

  # The generated floor window for each wheel: FLOOR_MISSES × CMD_PERIOD_MS.
  # A real wheel de-energises after this much command silence (§05).
  @floor_window_ms 5 * 20

  setup context do
    NodeRegistry.reset()
    PortIndex.build(@robot)

    # Most preflight tests drive the WHOLE host stack — Host starts the production
    # views over the loopback wire. The status-liveness test (:harness_view) drives
    # its OWN ViewHarness actuator view, so it must NOT also run Host's production
    # view for the same command slot (two writers of one slot is what the §07
    # sole-writer capability refuses). It stands up its own PubSub on a view-less
    # robot twin instead — see the test.
    if context[:harness_view] do
      :ok
    else
      sup = start_supervised!({Host, transport: LoopbackTransport, name: nil})

      # The robot boots :disarmed; the balance controller (the wheels' sole
      # commander) stays silent while disarmed, so the on-chip floor reaches its
      # safe state via command-silence (§05 / ADR-0010). Every preflight assertion
      # that a command reaches the wire therefore requires the robot ARMED first;
      # arming publishes the `:armed` transition the controller consumes to resume
      # commanding. (The born-stale test still sees no effort — the view never
      # publishes a pose, so the controller never ticks, armed or not.)
      :ok = BB.Safety.arm(@robot)

      transport = :sys.get_state(LinkOwner).transport
      on_exit(fn -> if Process.alive?(sup), do: Supervisor.stop(sup) end)
      {:ok, transport: transport}
    end
  end

  describe "(1) pitch-sign self-consistency (host side; absolute sign is a bench calibration)" do
    test "a forward tilt produces torque of the opposite sign to a backward tilt",
         %{transport: transport} do
      {:ok, left_slot} = PortIndex.resolve(:wheels, :motor_left)

      Balance.enable(@robot)

      # +pitch (nose up / tilt forward). target_pitch is 0.0, so error = -pitch < 0,
      # and with positive gains the host torque is NEGATIVE. This pins the HOST
      # convention; whether negative-torque physically drives under the fall is
      # decided by motor/encoder wiring (encoder.invert) at the bench — NOT here.
      inject_pose(transport, 1, +0.20)
      Process.sleep(40)
      inject_pose(transport, 2, +0.20)
      fwd = await_wire_effort(transport, left_slot)

      assert fwd < 0.0,
             "forward tilt (+pitch) must give corrective (negative) torque; got #{fwd}"

      # Mirror: a backward tilt must give the opposite (positive) torque.
      NodeRegistry.reset()
      inject_pose(transport, 3, -0.20)
      Process.sleep(40)
      inject_pose(transport, 4, -0.20)
      back = await_wire_effort(transport, left_slot)

      assert back > 0.0,
             "backward tilt (-pitch) must give corrective (positive) torque; got #{back}"

      # And the two tilts must drive in OPPOSITE directions (the sign actually tracks tilt).
      assert fwd * back < 0.0, "opposite tilts must give opposite-sign torque"
    end
  end

  describe "(2) born-stale / born-disarmed — no trust without a witnessed advance" do
    test "a single (non-advancing) pose produces NO balance torque on the wire",
         %{transport: transport} do
      {:ok, left_slot} = PortIndex.resolve(:wheels, :motor_left)

      Balance.enable(@robot)

      # ONE pose at the wire, never advanced. Born-stale: the chassis-IMU view must
      # not publish (it has not witnessed a seq advance since its boot), so the
      # controller never ticks, so no balance torque should reach the wire.
      inject_pose(transport, 1, 0.20)
      Process.sleep(120)

      refute effort_present?(transport, left_slot),
             "a non-advancing pose must not be trusted — no torque should reach the wire"
    end

    test "torque only appears AFTER a second, advancing pose is witnessed",
         %{transport: transport} do
      {:ok, left_slot} = PortIndex.resolve(:wheels, :motor_left)
      Balance.enable(@robot)

      inject_pose(transport, 1, 0.20)
      Process.sleep(60)
      refute effort_present?(transport, left_slot), "still born-stale after one pose"

      inject_pose(transport, 2, 0.20)

      assert await_wire_effort(transport, left_slot) != 0.0,
             "after the witnessed advance, balance torque reaches the wire"
    end
  end

  describe "(3) command cadence keeps the on-chip floor armed" do
    test "while pose flows at the sensor rate, commands reach the wire within the floor window",
         %{transport: transport} do
      {:ok, left_slot} = PortIndex.resolve(:wheels, :motor_left)
      Balance.enable(@robot)

      # Prime past born-stale.
      inject_pose(transport, 1, 0.10)
      Process.sleep(40)
      inject_pose(transport, 2, 0.10)
      assert await_wire_effort(transport, left_slot) != 0.0

      # Now drive pose at the contract rate and confirm a fresh command lands well
      # inside the floor window — i.e. a real wheel's floor would stay ARMED.
      count_before = effort_count(transport, left_slot)
      t0 = System.monotonic_time(:millisecond)

      for seq <- 3..8 do
        inject_pose(transport, seq, 0.10)
        Process.sleep(20)
      end

      gap = System.monotonic_time(:millisecond) - t0
      count_after = effort_count(transport, left_slot)

      assert count_after > count_before,
             "commands must keep flowing while pose flows"

      # The whole 6-tick burst (~120 ms) produced multiple commands, so the
      # inter-command gap is far under the 100 ms floor window — the floor stays armed.
      commands = count_after - count_before

      assert commands >= 3,
             "expected several commands in #{gap} ms (floor window #{@floor_window_ms} ms); got #{commands}"
    end
  end

  describe "(4) status liveness is freshness-gated" do
    @tag :harness_view
    test "a never-witnessed status reads not-driving (no false green)" do
      # The actuator view reports liveness from the status slot through a born-stale
      # monitor: with no status ever witnessed, it must read floored/unknown — the
      # dashboard must never show a confident 'driving' before a real status advance.
      #
      # We drive our OWN ViewHarness view here, so we run a view-less robot twin
      # (its PubSub satisfies the view's BB.subscribe) and NOT Host's production
      # view — so this harness view is the sole writer of the command slot (§07).
      twin = SegbyV1.Test.HarnessRobot
      PortIndex.build(twin)
      sup = start_supervised!(%{id: BB.Supervisor, start: {BB.Supervisor, :start_link, [twin]}})
      on_exit(fn -> if Process.alive?(sup), do: Supervisor.stop(sup) end)

      {:ok, view} =
        SegbyV1.Test.ViewHarness.start(BBMCUHub.BBHub.Actuator,
          bb: %{robot: twin, path: [:base_link, :left_wheel, :left_drive]},
          hub: :wheels,
          port: :motor_left,
          status_port: :status_left,
          fresh_for: 5,
          status_fresh_for: 2
        )

      on_exit(fn -> if Process.alive?(view), do: GenServer.stop(view) end)

      st = SegbyV1.Test.ViewHarness.view_state(view)
      assert BBMCUHub.BBHub.Actuator.live(st) == :floored_or_unknown
    end
  end

  # --- helpers (shared shape with SegbyV1.HostTest) --------------------------

  defp inject_pose(transport, seq, pitch) do
    {:ok, {node, port}} = PortIndex.resolve(:blaster, :pose)

    value = %{
      qw: 1.0,
      qx: 0.0,
      qy: 0.0,
      qz: 0.0,
      wx: 0.0,
      wy: 0.0,
      wz: 0.0,
      ax: -@g * :math.sin(pitch),
      ay: 0.0,
      az: @g * :math.cos(pitch)
    }

    body = Codec.encode_body(node, port, seq, seq * 1000, :imu, value, true)
    LoopbackTransport.inject(transport, body)
  end

  defp decoded_efforts(transport, {node, port}) do
    transport
    |> LoopbackTransport.sent()
    |> Enum.flat_map(fn body ->
      case Codec.decode_body(body) do
        {:ok, %{node: ^node, port_id: ^port, type: :effort, value: %{nm: nm}}} -> [nm]
        _ -> []
      end
    end)
  end

  defp effort_present?(transport, slot), do: decoded_efforts(transport, slot) != []

  defp effort_count(transport, slot), do: length(decoded_efforts(transport, slot))

  defp await_wire_effort(transport, slot, tries \\ 80) do
    case decoded_efforts(transport, slot) do
      [] when tries <= 0 ->
        flunk("no :effort frame reached the wire for slot #{inspect(slot)}")

      [] ->
        Process.sleep(10)
        await_wire_effort(transport, slot, tries - 1)

      efforts ->
        List.last(efforts)
    end
  end
end
