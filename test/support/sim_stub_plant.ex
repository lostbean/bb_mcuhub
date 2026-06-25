defmodule BBMCUHub.Test.SimStubPlant do
  @moduledoc """
  A trivial, fully deterministic `BBMCUHub.Sim.Plant` for the library's own tests
  (ADR-0008). It proves the behaviour is implementable and gives later chunks
  (the sim transport + driver) a hardware-free, engine-free plant to test against.

  No randomness, no wall-clock: `step/3` is a pure function of the step counter,
  the incoming commands, and `dt_s`. It carries a step counter forward (so a test
  can assert state advances) and folds the latest command into the produced sensor
  value (so a test can assert commands flow through). The sensor it emits is a
  single `:imu`-shaped value on a fixed slot, in the `{node, port_id, type, value}`
  wire shape the driver expects.
  """

  @behaviour BBMCUHub.Sim.Plant

  # The slot the stub reports its (synthetic) sensor on. Arbitrary but fixed so a
  # test can address it; mirrors a hub/port pair without depending on any robot.
  @sensor_node 0x02
  @sensor_port 0x00
  @sensor_type :imu

  defstruct step: 0

  @impl true
  def init(opts) do
    # Accept and ignore an optional :sensors template / arbitrary opts; the stub is
    # self-contained. The step counter starts at 0.
    _ = Keyword.get(opts, :sensors)
    {:ok, %__MODULE__{step: 0}}
  end

  @impl true
  def step(commands, dt_s, %__MODULE__{step: n} = state) do
    # Reduce the incoming per-slot commands to a single deterministic scalar so the
    # produced sensor visibly reflects "did a command arrive, and what was in it".
    cmd_signal =
      commands
      |> Map.values()
      |> Enum.flat_map(&Map.values/1)
      |> Enum.sum()

    # Derive a single :imu-shaped value from the counter, dt, and the command
    # signal — every field a float, so it round-trips through the codec layout.
    base = n * dt_s + cmd_signal

    value = %{
      qw: 1.0,
      qx: 0.0,
      qy: 0.0,
      qz: 0.0,
      wx: base,
      wy: 0.0,
      wz: 0.0,
      ax: cmd_signal,
      ay: 0.0,
      az: 0.0
    }

    sensors = [{@sensor_node, @sensor_port, @sensor_type, value}]
    {sensors, %{state | step: n + 1}}
  end

  @impl true
  def close(%__MODULE__{}), do: :ok
end
