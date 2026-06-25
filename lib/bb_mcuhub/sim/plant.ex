defmodule BBMCUHub.Sim.Plant do
  @moduledoc """
  The dynamics seam for running a robot **virtually**, with no hardware (ADR-0008).

  A `bb_mcuhub` robot is driven entirely above the `BBMCUHub.Host.Transport`
  boundary — the transport is the *one and only* hardware boundary the whole stack
  pivots on. A sim transport stands in for everything below it: the wire, the hubs,
  the floors, the silicon, and the **physical plant**. This behaviour is that
  physical plant: the consumer implements it to supply robot dynamics, and the
  (later) `BBMCUHub.Sim.Driver` runs a real-time loop that feeds it the captured
  per-slot commands, advances it, and injects the sensors it returns back up the
  real host stack as wire bodies.

  ## What a plant speaks (ADR-0005 / ADR-0008)

  A `Plant` is **engine-agnostic and robot-agnostic at this seam**: it speaks
  **value-type values keyed by wire slot**, never robot-specific structs. This
  mirrors ADR-0005 (the floor is byte/value-generic) — the behaviour itself names
  no concrete value types; the shapes below are only *examples* of what flows
  through it.

  - A **command** for an actuator slot is a value-type value, e.g. `%{nm: 0.5}` for
    an `:effort` port.
  - A **sensor** the plant produces is a `{node, port_id, type, value}` tuple — the
    exact shape the driver hands to `BBMCUHub.Wire.Codec.encode_body/7`. Example
    values: an `:imu` is `%{qw: _, qx: _, qy: _, qz: _, wx: _, wy: _, wz: _, ax: _,
    ay: _, az: _}` (all floats); a `:status` is `%{applied_seq: _, floored: _}`.

  A consumer who wants a MuJoCo bot, a pure-Elixir toy, a different engine, or a
  recorded-trace plant just writes a different `Plant` — the library ships no
  physics engine. (The worked example's `SegbyV1.Sim.MujocoPlant` is one such
  implementation, owning a `Port` to a Python/MuJoCo child.)

  ## Lifecycle

  `init/1` builds the opaque plant state; `step/3` advances the world one tick and
  returns the sensor readings the hubs would have produced; `close/1` releases any
  resources (ports, files) the plant holds and must be idempotent.
  """

  @typedoc "A wire slot: `{node id, port id}`, both `0..255`."
  @type slot :: {0..255, 0..255}

  @typedoc "A value-type value: a raw `%{field => number}` map, e.g. `%{nm: 0.5}` for `:effort`."
  @type value :: %{atom() => number()}

  @typedoc "A sensor reading the plant produces: `{node, port_id, value_type_atom, value}`."
  @type sensor :: {0..255, 0..255, atom(), value()}

  @typedoc "The plant's opaque, implementation-defined state, threaded through `step/3`."
  @type state :: term()

  @doc "Initialise the plant from opts; return its opaque state."
  @callback init(opts :: keyword()) :: {:ok, state()}

  @doc """
  Advance the simulated world by `dt_s` seconds under the latest per-slot commands,
  and return the sensor readings the hubs would have produced this step.

  `commands` is the newest command value per actuator slot (may be empty).
  """
  @callback step(commands :: %{slot() => value()}, dt_s :: float(), state()) ::
              {sensors :: [sensor()], state()}

  @doc "Release any resources the plant holds (ports, files). Idempotent."
  @callback close(state()) :: :ok
end
