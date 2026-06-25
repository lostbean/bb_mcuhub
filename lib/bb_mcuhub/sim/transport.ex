defmodule BBMCUHub.Sim.Transport do
  @moduledoc """
  A `BBMCUHub.Host.Transport` that stands in for the wire + hubs + silicon, so
  the real host stack runs unchanged against a simulated robot (ADR-0008:
  *Virtual robot — sim in the loop*; the transport is the hardware boundary).

  Instead of writing bodies to a UART, this transport **decodes each outbound
  body and captures the newest command value per actuator wire slot**
  (`{node, port_id}`). A sibling `BBMCUHub.Sim.Driver` reads those captured
  commands once per simulated tick, steps the plant, and injects the resulting
  sensor bodies back to the owner.

  This module is a **pure capture** — it owns no clock, no timer, no plant call.
  It only decodes + stores. The loop lives in `Sim.Driver`, not here (ADR-0008):
  keeping the transport stateless about time is what lets the driver advance
  simulated time deterministically.

  ## Semantics

    * `send/2` decodes the body via the real `BBMCUHub.Wire.Codec.decode_body/1`
      and stores `{node, port_id} => value`, **overwriting** any prior value for
      that slot (newest wins). An undecodable body is dropped silently — mirroring
      how the real `LinkOwner` counts a `decode_fail` and moves on — so a torn or
      garbage body never crashes the transport.
    * `take_commands/1` returns the current **newest-per-slot snapshot** and does
      **not** clear it. The driver wants "the latest command for each slot right
      now"; if a read cleared the map, a slot that received no new command between
      two ticks would vanish, which is wrong. So the read is non-clearing: a slot
      persists until a newer command for it arrives.

  Inbound (hub → host) traffic is the driver's job, not this transport's. The
  only inbound concern here is recording the `owner` pid at `start_link` (the
  driver is later handed the transport + owner to inject sensor bodies).
  """

  @behaviour BBMCUHub.Host.Transport
  use GenServer
  import Kernel, except: [send: 2]

  alias BBMCUHub.Wire.Codec

  @type slot :: {0..255, 0..255}
  @type value :: %{atom() => number()}

  # --- Host.Transport callbacks ---

  @impl BBMCUHub.Host.Transport
  @spec start_link(pid(), keyword()) :: {:ok, pid()} | {:error, term()}
  def start_link(owner, opts) when is_pid(owner) and is_list(opts) do
    # An optional `:name` lets a consumer locate this transport after the LinkOwner
    # starts it (the LinkOwner holds the pid privately), so a sibling `Sim.Driver`
    # can `take_commands/1` from it by name. Unnamed by default (tests pass a pid).
    case Keyword.get(opts, :name) do
      nil -> GenServer.start_link(__MODULE__, owner)
      name -> GenServer.start_link(__MODULE__, owner, name: name)
    end
  end

  @impl BBMCUHub.Host.Transport
  @spec send(GenServer.server(), binary()) :: :ok
  def send(t, body) when is_binary(body) do
    GenServer.call(t, {:send, body})
  end

  @impl BBMCUHub.Host.Transport
  @spec close(GenServer.server()) :: :ok
  def close(t) do
    GenServer.stop(t)
  end

  # --- sim seam (read by Sim.Driver) ---

  @doc """
  The newest captured command value per actuator slot, a map
  `%{{node, port_id} => value}`.

  This is a **non-clearing** snapshot of the newest-per-slot state: a slot stays
  until a newer command for it arrives (see the moduledoc for why a clearing read
  would be wrong for the driver).
  """
  @spec take_commands(GenServer.server()) :: %{slot() => value()}
  def take_commands(t) do
    GenServer.call(t, :take_commands)
  end

  @doc "The pid that owns this transport (the inbound target; recorded at start_link)."
  @spec owner(GenServer.server()) :: pid()
  def owner(t) do
    GenServer.call(t, :owner)
  end

  # --- GenServer ---

  @impl GenServer
  def init(owner) do
    {:ok, %{owner: owner, commands: %{}, drops: 0}}
  end

  @impl GenServer
  def handle_call({:send, body}, _from, st) do
    case Codec.decode_body(body) do
      {:ok, %{node: node, port_id: port_id, value: value}} ->
        commands = Map.put(st.commands, {node, port_id}, value)
        {:reply, :ok, %{st | commands: commands}}

      :error ->
        # Mirror the real LinkOwner: a decode_fail is dropped, not fatal.
        {:reply, :ok, %{st | drops: st.drops + 1}}
    end
  end

  def handle_call(:take_commands, _from, st) do
    {:reply, st.commands, st}
  end

  def handle_call(:owner, _from, st) do
    {:reply, st.owner, st}
  end
end
