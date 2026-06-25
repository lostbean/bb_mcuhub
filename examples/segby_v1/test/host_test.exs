defmodule SegbyV1.HostTest do
  @moduledoc """
  The segby_v1 host integration proof (§07/§09) — the WHOLE host stack stood up
  by the launcher (`SegbyV1.Host`) over the real COBS+CRC seam, with no hardware:
  the BeamBots tree (sensor + actuator views + the `:balance` controller + the
  `:teleop` command) plus the `LinkOwner` on a loopback transport.

  This is the LIBRARY-CONSUMPTION proof on the host side: every library reference
  here is a public seam (`BBMCUHub.Host`, `BBMCUHub.Contract.PortIndex`,
  `BBMCUHub.Host.LinkOwner`/`NodeRegistry`, `BBMCUHub.Wire.Codec`); the robot,
  controller, value-types, and the launcher wrapper are the consumer's own
  (`SegbyV1.*`). The loopback transport is the consumer's own copy over the
  library's public `BBMCUHub.Host.Transport` behaviour.

  Two end-to-end assertions:

    * **balance → wire**: synthetic pose injected at the wire, lifted by the
      chassis-IMU sensor view, fed to the (enabled) balance controller, turned
      into per-wheel Effort by the actuator views, drained by the LinkOwner —
      reaches the loopback's sent frames for BOTH wheel slots, as a decoded
      `:effort`. This is the full host pipeline through the real framing.

    * **teleop loop closed**: a teleop intent delivered the way `bb_tui` delivers
      it (running the declared `:teleop` command, which publishes a `Twist` onto
      the balance controller's teleop topic) BIASES the per-wheel effort on the
      wire — proving the operator-input loop from the dashboard reaches the
      controller's `last_teleop` and out to the wheels.

  We also prove the DATA PATH `bb_tui` reads (it is interactive, so it can't be
  driven headless): when segby is driven, the robot publishes on the BB PubSub
  paths `bb_tui` subscribes to (`[:sensor | _]`, `[:actuator | _]`), so the
  dashboard WOULD populate. Launch it for real with
  `mix bb.tui --robot SegbyV1.Robot`.
  """
  # Not async: the host stack builds the global PortIndex for segby
  # (:persistent_term) and registers the LinkOwner under its default name.
  use ExUnit.Case, async: false

  alias BB.Math.{Quaternion, Vec3}
  alias BB.Message.Geometry.Twist
  alias BBMCUHub.Contract.PortIndex
  alias BBMCUHub.Host.{LinkOwner, NodeRegistry}
  alias BBMCUHub.Wire.Codec
  alias SegbyV1.Balance
  alias SegbyV1.Host
  alias SegbyV1.Robot
  alias SegbyV1.Test.LoopbackTransport

  @robot Robot

  setup do
    NodeRegistry.reset()
    # the launcher points PortIndex at segby; do it here too so the resolve
    # helpers below see segby's ids regardless of test order.
    PortIndex.build(@robot)

    # The whole host stack via the launcher, on the loopback transport so the
    # real COBS+CRC seam runs without hardware.
    sup = start_supervised!({Host, transport: LoopbackTransport, name: nil})

    # The robot boots :disarmed (BB's safe default) and the balance controller —
    # the wheels' sole commander — publishes NOTHING while disarmed (the on-chip
    # floor reaches safe-state via command-silence; a controller that kept
    # commanding would defeat disarm, see ADR-0010). Every assertion below that a
    # command reaches the wire therefore requires the robot ARMED first. Arming
    # here flips safety state and publishes the `:armed` transition on
    # `[:state_machine]`, which the running controller consumes to re-arm itself.
    :ok = BB.Safety.arm(@robot)

    transport = :sys.get_state(LinkOwner).transport
    on_exit(fn -> if Process.alive?(sup), do: Supervisor.stop(sup) end)

    {:ok, sup: sup, transport: transport}
  end

  describe "the launcher resolves the segby wheel command slots" do
    test "both wheel slots resolve from the segby IR (the per-robot derivation)" do
      assert [{0x05, _left_id}, {0x05, _right_id}] = Host.command_slots()
      {:ok, left} = PortIndex.resolve(:wheels, :motor_left)
      {:ok, right} = PortIndex.resolve(:wheels, :motor_right)
      assert Host.command_slots() == [left, right]
    end
  end

  describe "balance → wire: pose drives Effort to BOTH wheels through the real frame" do
    test "an enabled balance loop reaches the wire for both wheels", %{transport: transport} do
      {:ok, left_slot} = PortIndex.resolve(:wheels, :motor_left)
      {:ok, right_slot} = PortIndex.resolve(:wheels, :motor_right)

      Balance.enable(@robot)

      # Inject a tilted pose at the wire, twice with advancing seq so the
      # born-stale chassis-IMU view trusts it and the controller sees a dt.
      inject_pose(transport, 1, 0.15)
      Process.sleep(40)
      inject_pose(transport, 2, 0.15)

      # the full pipeline drains a non-zero effort to BOTH wheel slots
      left = await_wire_effort(transport, left_slot)
      right = await_wire_effort(transport, right_slot)

      # target_pitch 0.0, pitch +0.15 → error < 0 → kp 0.5 → corrective torque
      # (non-zero) on both wheels, with no teleop the two are equal.
      assert left != 0.0
      assert right != 0.0
      assert_in_delta left, right, 1.0e-6
    end
  end

  describe "teleop loop closed: a bb_tui-delivered intent drives the wire" do
    test "running the :teleop command drives the per-wheel velocity loop on the wire",
         %{transport: transport} do
      {:ok, left_slot} = PortIndex.resolve(:wheels, :motor_left)
      {:ok, right_slot} = PortIndex.resolve(:wheels, :motor_right)

      # balance stays DISABLED so the PID torque is ~0 and the velocity-loop term is
      # the whole signal — the cleanest proof the intent reached the wheels. With
      # balance off the inner velocity loop STILL runs (ADR-0009), so forward
      # teleop drives the wheels. No vel stream is injected here, so measured ≈ 0.
      # Deliver teleop the bb_tui way: execute the declared :teleop command.
      {:ok, cmd} = BB.Robot.Runtime.execute(@robot, :teleop, %{forward: 1.0, turn: 0.0})
      _ = BB.Command.await(cmd)

      # let the Twist land on the controller's teleop_topic before the pose tick
      Process.sleep(30)

      inject_pose(transport, 1, 0.0)
      Process.sleep(40)
      inject_pose(transport, 2, 0.0)

      left = await_wire_effort(transport, left_slot)
      right = await_wire_effort(transport, right_slot)

      # forward 1.0 → target = max_speed (10.0) rad/s; measured ≈ 0 → the velocity
      # term is kv·(10.0 - 0) = 0.02·10.0 = 0.20 onto both wheels (turn 0).
      assert_in_delta left, 0.20, 1.0e-2
      assert_in_delta right, 0.20, 1.0e-2
    end

    test "the controller's BB.Message Twist path updates last_teleop directly" do
      # Unit-level proof of the closed clause (no slot machinery): the controller,
      # subscribed to its teleop_topic, consumes a Twist and biases the next tick.
      {:ok, ctrl} =
        SegbyV1.Test.ViewHarness.start(Balance,
          bb: %{robot: @robot, path: [:balance]},
          pose_topic: [:sensor, :base_link, :chassis_imu],
          teleop_topic: [:teleop, :segby],
          vel_left_topic: [:sensor, :base_link, :vel_left],
          vel_right_topic: [:sensor, :base_link, :vel_right],
          left_actuator_path: [:t, :l],
          right_actuator_path: [:t, :r],
          kp: 0.5,
          ki: 0.05,
          kd: 0.1,
          target_pitch: 0.0,
          integral_clamp: 1.0,
          output_clamp: 1.0,
          max_speed: 10.0,
          max_yaw_rate: 0.5,
          kyaw: 0.012,
          kv: 0.02,
          enabled: true
        )

      on_exit(fn -> if Process.alive?(ctrl), do: GenServer.stop(ctrl) end)

      # Twist.new/3 returns a full BB.Message (payload = the Twist) — exactly what
      # lands on the topic, so deliver it as-is.
      {:ok, msg} = Twist.new(:teleop, Vec3.new(1.0, 0.0, 0.0), Vec3.new(0.0, 0.0, -1.0))
      send(ctrl, {:bb, [:teleop, :segby], msg})

      view = SegbyV1.Test.ViewHarness.view_state(ctrl)
      assert_in_delta view.last_teleop.forward, 1.0, 1.0e-9
      assert_in_delta view.last_teleop.turn, -1.0, 1.0e-9
    end
  end

  describe "the data path bb_tui reads: segby publishes on the paths it subscribes to" do
    test "a driven sensor view publishes pose on [:sensor | _] (bb_tui would populate it)",
         %{transport: transport} do
      # bb_tui subscribes to [:sensor]; subscribing to the chassis-IMU sub-path
      # proves the same data is on the wire it reads.
      BB.subscribe(@robot, [:sensor, :base_link, :chassis_imu])

      inject_pose(transport, 1, 0.05)
      Process.sleep(40)
      inject_pose(transport, 2, 0.05)

      assert_receive {:bb, [:sensor, :base_link, :chassis_imu],
                      %BB.Message{payload: %BB.Message.Sensor.Imu{}}},
                     500
    end

    test "a driven balance loop publishes Effort on [:actuator | _] (bb_tui's joints/actuator path)",
         %{transport: _transport} do
      # bb_tui subscribes to [:actuator]; the actuator views publish nothing
      # themselves, but the balance controller publishes Effort on the actuator
      # paths, which is exactly the [:actuator | _] traffic bb_tui renders.
      BB.subscribe(@robot, [:actuator, :base_link, :left_wheel, :left_drive])

      Balance.enable(@robot)
      send_pose_pubsub(0.2, 0)
      send_pose_pubsub(0.2, 10_000_000)

      assert_receive {:bb, [:actuator, :base_link, :left_wheel, :left_drive],
                      %BB.Message{payload: %BB.Message.Actuator.Command.Effort{}}},
                     500
    end
  end

  # --- helpers ---------------------------------------------------------------

  # one g, m/s² — the segby MCU ships accel in engineering units.
  @g 9.81

  # Inject a pose frame at the wire (the loopback delivers it to the LinkOwner as
  # if a hub produced it). The segby Blaster MCU ships an IDENTITY orientation +
  # real accel/gyro, so a body-Y tilt `pitch` rides as the gravity projection in
  # the accel vector (ax = -g·sin(pitch), az = g·cos(pitch)) — exactly what the
  # live complementary-filter path reads. Advancing `seq`.
  defp inject_pose(transport, seq, pitch) do
    {:ok, {node, port}} = PortIndex.resolve(:blaster, :pose)

    value = %{
      # identity orientation — the MCU does not fuse (the host filter does)
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

  # Publish a pose directly on the chassis-IMU topic (bypasses the wire) — for the
  # actuator-publish data-path test where we only need the controller to tick. As
  # on the wire: identity orientation, the tilt carried in the accel vector.
  defp send_pose_pubsub(pitch, mono) do
    msg = %BB.Message{
      monotonic_time: mono,
      wall_time: mono,
      node: Node.self(),
      frame_id: :chassis_imu,
      robot: @robot,
      payload: %BB.Message.Sensor.Imu{
        orientation: Quaternion.identity(),
        angular_velocity: Vec3.zero(),
        linear_acceleration: Vec3.new(-@g * :math.sin(pitch), 0.0, @g * :math.cos(pitch))
      }
    }

    BB.publish(@robot, [:sensor, :base_link, :chassis_imu], msg)
  end

  # Poll the loopback's sent frames for the latest decoded :effort value on a
  # given wheel command slot. Fails if none arrives.
  defp await_wire_effort(transport, {node, port}, tries \\ 80) do
    last =
      transport
      |> LoopbackTransport.sent()
      |> Enum.reverse()
      |> Enum.find_value(fn body ->
        case Codec.decode_body(body) do
          {:ok, %{node: ^node, port_id: ^port, type: :effort, value: %{nm: nm}}} -> {:ok, nm}
          _ -> nil
        end
      end)

    cond do
      last != nil ->
        {:ok, nm} = last
        nm

      tries <= 0 ->
        flunk("no :effort frame reached the wire for slot #{inspect({node, port})}")

      true ->
        Process.sleep(10)
        await_wire_effort(transport, {node, port}, tries - 1)
    end
  end
end
