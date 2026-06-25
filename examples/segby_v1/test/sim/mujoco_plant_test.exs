defmodule SegbyV1.Sim.MujocoPlantTest do
  @moduledoc """
  Unit test for the segby MuJoCo `Plant` (ADR-0008, chunk 4) — a FAKE child, no
  MuJoCo, no Port. The fake child records every line the plant writes and replays
  canned lines, so we assert the command mapping (effort → ctrl index) and the
  sensor parsing (MuJoCo arrays → `:imu` / `:status` values) deterministically.
  """
  use ExUnit.Case, async: true

  alias BBMCUHub.Contract.PortIndex
  alias SegbyV1.Sim.MujocoPlant

  @robot SegbyV1.Robot

  # A canned MuJoCo `state` reply: a slight tilt (qx≠0), nonzero gyro/acc, so the
  # parsed :imu value is unambiguous. The `qvel` carries the wheel hinge angular
  # velocities at indices 6 (left) and 7 (right) — for segby's freejoint base the
  # root's 6 DOF occupy qvel[0..5], then the two wheel hinges (verified against
  # sim/segby.xml: nv=8, wheel dofadr [6, 7]). Distinct + one negative so the two
  # WheelSpeed sensors are unambiguous: qvel[6] = 2.5 (left), qvel[7] = -1.5 (right).
  @state_line Jason.encode!(%{
                "event" => "state",
                "time" => 0.02,
                "qpos" => [0.0, 0.0, 0.22, 0.999, 0.0436, 0.0, 0.0, 0.1, 0.2],
                "qvel" => [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.5, -1.5],
                "framequat" => [0.999, 0.0436, 0.0, 0.0],
                "gyro" => [0.01, -0.02, 0.03],
                "acc" => [0.4, 0.0, 9.7]
              })

  defmodule FakeChild do
    @moduledoc """
    A fake `MujocoPlant.Child`: an Agent holding the canned reply lines to hand out
    (a queue) and the lines the plant wrote (for assertions). The plant passes this
    Agent pid as the child handle via `:child_opts`.
    """
    @behaviour SegbyV1.Sim.MujocoPlant.Child

    use Agent

    def start(replies) when is_list(replies) do
      Agent.start_link(fn -> %{replies: replies, sent: []} end)
    end

    def sent(pid), do: Agent.get(pid, fn st -> Enum.reverse(st.sent) end)

    @impl true
    def open(opts) do
      # The test injects the already-started Agent pid via :pid.
      {:ok, Keyword.fetch!(opts, :pid)}
    end

    @impl true
    def send_line(pid, line) do
      Agent.update(pid, fn st -> %{st | sent: [IO.iodata_to_binary(line) | st.sent]} end)
      :ok
    end

    @impl true
    def recv_line(pid) do
      Agent.get_and_update(pid, fn
        %{replies: [next | rest]} = st -> {{:ok, next}, %{st | replies: rest}}
        %{replies: []} = st -> {{:error, :no_more_replies}, st}
      end)
    end

    @impl true
    def close(_pid), do: :ok
  end

  setup do
    # The plant resolves slots against the PortIndex; build it for the segby robot.
    PortIndex.build(@robot)
    :ok
  end

  # Start a plant wired to a FakeChild preloaded with `replies` (after the ready
  # line). Returns {plant_state, fake_pid}.
  defp start_plant(replies) do
    ready = Jason.encode!(%{"event" => "ready", "actuators" => [], "sensors" => []})
    {:ok, fake} = FakeChild.start([ready | replies])

    {:ok, state} =
      MujocoPlant.init(child: FakeChild, child_opts: [pid: fake], mjcf_timestep_s: 0.002)

    {state, fake}
  end

  test "init/1 reads the ready line and resolves slots" do
    {state, _fake} = start_plant([])

    # The blaster pose slot + the two wheel status/motor/vel slots are resolved.
    assert {:ok, state.pose_slot} == PortIndex.resolve(:blaster, :pose)
    assert {:ok, state.motor_left_slot} == PortIndex.resolve(:wheels, :motor_left)
    assert {:ok, state.status_left_slot} == PortIndex.resolve(:wheels, :status_left)
    assert {:ok, state.vel_left_slot} == PortIndex.resolve(:wheels, :vel_left)
    assert {:ok, state.vel_right_slot} == PortIndex.resolve(:wheels, :vel_right)
  end

  test "step/1 writes a ctrl with the left effort at index 0 and right at index 1" do
    {state, fake} = start_plant([@state_line])

    {:ok, left_slot} = PortIndex.resolve(:wheels, :motor_left)
    {:ok, right_slot} = PortIndex.resolve(:wheels, :motor_right)

    commands = %{left_slot => %{nm: 0.5}, right_slot => %{nm: -0.25}}

    {_sensors, _state} = MujocoPlant.step(commands, 0.02, state)

    # The plant wrote exactly one command line; decode it and assert the ctrl order.
    [sent] = FakeChild.sent(fake)
    decoded = Jason.decode!(sent)

    assert decoded["op"] == "set_ctrl_and_step"
    assert [left, right] = decoded["ctrl"]
    assert left == 0.5
    assert right == -0.25
    # dt 0.02 / timestep 0.002 = 10 substeps.
    assert decoded["n"] == 10
  end

  test "step/1 with no command for a slot writes 0.0 at that ctrl index" do
    {state, fake} = start_plant([@state_line])

    {:ok, left_slot} = PortIndex.resolve(:wheels, :motor_left)
    {_sensors, _state} = MujocoPlant.step(%{left_slot => %{nm: 0.7}}, 0.02, state)

    [sent] = FakeChild.sent(fake)
    assert [0.7, right] = Jason.decode!(sent)["ctrl"]
    assert right == 0.0
  end

  test "step/1 parses the state line into a valid :imu and two :status sensors" do
    {state, _fake} = start_plant([@state_line])

    {sensors, _state} = MujocoPlant.step(%{}, 0.02, state)

    {:ok, pose_slot} = PortIndex.resolve(:blaster, :pose)
    {:ok, status_left_slot} = PortIndex.resolve(:wheels, :status_left)
    {:ok, status_right_slot} = PortIndex.resolve(:wheels, :status_right)

    {pn, pp} = pose_slot
    {sln, slp} = status_left_slot
    {srn, srp} = status_right_slot

    # --- the IMU on the pose slot ---
    imu = Enum.find(sensors, fn {n, p, type, _v} -> {n, p} == {pn, pp} and type == :imu end)
    assert {^pn, ^pp, :imu, value} = imu

    # all 10 fields present and float
    for key <- [:qw, :qx, :qy, :qz, :wx, :wy, :wz, :ax, :ay, :az] do
      assert is_float(Map.fetch!(value, key)), "imu field #{key} must be a float"
    end

    # quaternion from framequat, gyro → angular velocity, acc → linear accel
    assert value.qw == 0.999
    assert value.qx == 0.0436
    assert value.wx == 0.01
    assert value.wy == -0.02
    assert value.wz == 0.03
    assert value.ax == 0.4
    assert value.az == 9.7

    # --- the two wheel statuses ---
    status_left =
      Enum.find(sensors, fn {n, p, type, _v} -> {n, p} == {sln, slp} and type == :status end)

    status_right =
      Enum.find(sensors, fn {n, p, type, _v} -> {n, p} == {srn, srp} and type == :status end)

    assert {^sln, ^slp, :status, lval} = status_left
    assert {^srn, ^srp, :status, rval} = status_right

    assert is_integer(lval.applied_seq)
    assert is_integer(rval.applied_seq)
    assert lval.floored == false
    assert rval.floored == false
    # first step → applied_seq advances to 1 on each wheel
    assert lval.applied_seq == 1
    assert rval.applied_seq == 1
  end

  test "step/1 maps qvel[6]/qvel[7] into two WheelSpeed sensors on the vel slots" do
    {state, _fake} = start_plant([@state_line])

    {sensors, _state} = MujocoPlant.step(%{}, 0.02, state)

    {:ok, vel_left_slot} = PortIndex.resolve(:wheels, :vel_left)
    {:ok, vel_right_slot} = PortIndex.resolve(:wheels, :vel_right)

    {vln, vlp} = vel_left_slot
    {vrn, vrp} = vel_right_slot

    # The sensor tuple carries the value-type MODULE (not a stock atom): the codec
    # resolves it via BBMCUHub.ValueType.resolve/1, which passes a module through.
    vel_left =
      Enum.find(sensors, fn {n, p, type, _v} ->
        {n, p} == {vln, vlp} and type == SegbyV1.ValueTypes.WheelSpeed
      end)

    vel_right =
      Enum.find(sensors, fn {n, p, type, _v} ->
        {n, p} == {vrn, vrp} and type == SegbyV1.ValueTypes.WheelSpeed
      end)

    assert {^vln, ^vlp, SegbyV1.ValueTypes.WheelSpeed, lval} = vel_left
    assert {^vrn, ^vrp, SegbyV1.ValueTypes.WheelSpeed, rval} = vel_right

    # qvel[6] = 2.5 (left), qvel[7] = -1.5 (right), in the WheelSpeed layout.
    assert lval == %{rad_s: 2.5}
    assert rval == %{rad_s: -1.5}
    assert is_float(lval.rad_s)
    assert is_float(rval.rad_s)
  end

  test "applied_seq advances per wheel across successive steps" do
    {state, _fake} = start_plant([@state_line, @state_line])

    {sensors1, state} = MujocoPlant.step(%{}, 0.02, state)
    {sensors2, _state} = MujocoPlant.step(%{}, 0.02, state)

    seq = fn sensors ->
      sensors
      |> Enum.find(fn {_n, _p, type, _v} -> type == :status end)
      |> elem(3)
      |> Map.fetch!(:applied_seq)
    end

    assert seq.(sensors1) == 1
    assert seq.(sensors2) == 2
  end

  test "close/1 sends quit and closes cleanly" do
    {state, fake} = start_plant([])

    assert :ok = MujocoPlant.close(state)

    [sent] = FakeChild.sent(fake)
    assert Jason.decode!(sent) == %{"op" => "quit"}
  end

  # Regression guard for the smoke-run bug: the real Port child is invoked
  # `<interpreter> <script.py> <mjcf>`. The args MUST lead with the Python script
  # (segby_sim.py beside the MJCF), then the MJCF — passing only the MJCF made the
  # interpreter try to run segby.xml AS Python (SyntaxError on the XML).
  test "child_args leads with the python script, then the mjcf" do
    mjcf = "/some/app/sim/segby.xml"

    assert [script, ^mjcf] = MujocoPlant.child_args(mjcf)
    assert script == "/some/app/sim/segby_sim.py"
  end
end
