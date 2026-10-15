defmodule BBMCUHub.Test.ViewHarness do
  @moduledoc """
  A minimal GenServer that drives a BB view callback module (`BB.Sensor` /
  `BB.Actuator`) in isolation for tests.

  In production BeamBots wraps a view in `BB.Sensor.Server` / `BB.Actuator.Server`
  and delegates to its callbacks. Here we delegate `init/1`, `handle_info/2` and
  `handle_command/2` ourselves, so a test can `send/2` the view a `:beat` or a
  `{:bb, ...}` command and observe the effect, without standing up the whole BB
  supervision tree.

  As of bb 0.23 an actuator receives commands at `handle_command/2` rather than at
  a callback per transport, so a `{:bb, _, %BB.Message{}}` sent here is routed
  there — but only if its payload is one the module declared in
  `command_payloads/1`. That mirrors both halves of what `BB.Actuator.Server`
  does: it funnels published, cast and called commands into the one callback, and
  it admits only the declared payloads. Anything else — including every message
  to a sensor view, which declares none — goes to `handle_info/2`.

  Two things the real server does that this harness deliberately does not: refuse
  commands while the robot is disarmed, and apply the joint's transmission. Both
  are framework concerns a view never sees, so omitting them keeps these tests
  scoped to the view itself.

  Like the real servers, opts are run through `BB.Component.OptionsSchema.validate`
  before `init/1`, so the view sees **schema-validated** opts with defaults filled —
  exactly what production hands it. (Without this the harness would call `init/1`
  with raw opts, so any value sourced from a schema default would be `nil` here and
  the view's behaviour would silently diverge from production.) `:bb` is the only
  framework-injected key the views read (they never touch sensor/motor profiles).
  """
  use GenServer

  alias BB.Component.OptionsSchema

  @framework_keys [:bb]

  def start(module, opts), do: GenServer.start_link(__MODULE__, {module, opts})

  @doc "The current view (callback-module) state — for asserting on view internals."
  def view_state(pid), do: GenServer.call(pid, :__view_state__)

  @impl true
  def init({module, opts}) do
    with {:ok, opts} <- OptionsSchema.validate(module, opts, @framework_keys) do
      init_view(module, opts)
    else
      {:error, error} -> {:stop, error}
    end
  end

  defp init_view(module, opts) do
    # The payloads are asked for BEFORE `init/1`, exactly as `BB.Actuator.Server`
    # asks: a view deriving them from its options cannot lean on its own state.
    st = %{module: module, command_payloads: declared_payloads(module, opts), view: nil}

    case module.init(opts) do
      {:ok, view_state} -> {:ok, %{st | view: view_state}}
      {:ok, view_state, _extra} -> {:ok, %{st | view: view_state}}
      {:stop, reason} -> {:stop, reason}
      other -> {:stop, {:bad_init, other}}
    end
  end

  # A sensor view declares none, and takes every message at `handle_info/2`.
  defp declared_payloads(module, opts) do
    if function_exported?(module, :command_payloads, 1),
      do: module.command_payloads(opts),
      else: []
  end

  @impl true
  def handle_call(:__view_state__, _from, st), do: {:reply, st.view, st}

  @impl true
  def handle_info({:bb, _topic, %BB.Message{} = message} = msg, st) do
    if command?(message, st) do
      apply_reply(st.module.handle_command(message, st.view), st)
    else
      apply_reply(st.module.handle_info(msg, st.view), st)
    end
  end

  def handle_info(msg, st) do
    apply_reply(st.module.handle_info(msg, st.view), st)
  end

  # Only a payload the module declared is a command — the same test
  # `BB.Actuator.Server` applies before it dispatches one.
  defp command?(%BB.Message{payload: %payload_module{}}, st),
    do: payload_module in st.command_payloads

  defp command?(_message, _st), do: false

  # `handle_command/2` may also reply, for the synchronous transport. Nothing here
  # is waiting on it, so it is discarded exactly as it would be for a published or
  # cast command.
  defp apply_reply({:reply, _reply, view}, st), do: {:noreply, %{st | view: view}}
  defp apply_reply({:reply, _reply, view, _extra}, st), do: {:noreply, %{st | view: view}}
  defp apply_reply({:noreply, view}, st), do: {:noreply, %{st | view: view}}
  defp apply_reply({:noreply, view, _extra}, st), do: {:noreply, %{st | view: view}}
  defp apply_reply({:stop, reason, view}, st), do: {:stop, reason, %{st | view: view}}
end
