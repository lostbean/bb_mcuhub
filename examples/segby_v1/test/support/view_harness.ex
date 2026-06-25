defmodule SegbyV1.Test.ViewHarness do
  @moduledoc """
  A minimal GenServer that drives a BB view / controller callback module
  (`BB.Sensor` / `BB.Actuator` / `BB.Controller`) in isolation for the example's
  tests.

  In production BeamBots wraps a view in its server and delegates to its
  callbacks. Here we delegate `init/1` and `handle_info/2` ourselves, so a test
  can `send/2` the callback module a `:beat` or a `{:bb, ...}` message and observe
  the effect, without standing up the whole BB supervision tree.

  ## Safety-state transitions (mirroring `BB.Controller.Server`)

  The real `BB.Controller.Server` does NOT hand disarm transitions straight to a
  controller's `handle_info/2`: it intercepts a `[:state_machine]` transition to
  `:disarming` / `:disarmed` / `:error` and routes it to the controller's
  `handle_safety_state_change/2` callback (the `:armed` transition is NOT a disarm
  state, so it falls through to `handle_info/2`). This harness mirrors that split
  so a harness-driven controller exercises the SAME disarm path as production —
  otherwise a disarm transition would silently hit the controller's catch-all and
  the safety gate could never be tested here. The routing only applies to callback
  modules that implement `handle_safety_state_change/2` (controllers); views that
  don't get the plain `handle_info/2` delivery.
  """
  use GenServer

  alias BB.StateMachine.Transition

  @disarm_states [:disarming, :disarmed, :error]

  def start(module, opts), do: GenServer.start_link(__MODULE__, {module, opts})

  @doc "The current view (callback-module) state — for asserting on view internals."
  def view_state(pid), do: GenServer.call(pid, :__view_state__)

  @impl true
  def init({module, opts}) do
    case module.init(opts) do
      {:ok, view_state} -> {:ok, %{module: module, view: view_state}}
      {:ok, view_state, _extra} -> {:ok, %{module: module, view: view_state}}
      {:stop, reason} -> {:stop, reason}
      other -> {:stop, {:bad_init, other}}
    end
  end

  @impl true
  def handle_call(:__view_state__, _from, st), do: {:reply, st.view, st}

  @impl true
  def handle_info(
        {:bb, [:state_machine], %BB.Message{payload: %Transition{to: to}}},
        %{module: module} = st
      )
      when to in @disarm_states do
    if function_exported?(module, :handle_safety_state_change, 2) do
      case module.handle_safety_state_change(to, st.view) do
        {:continue, view} -> {:noreply, %{st | view: view}}
        {:stop, reason, view} -> {:stop, reason, %{st | view: view}}
      end
    else
      delegate_handle_info({:bb, [:state_machine], %{payload: %Transition{to: to}}}, st)
    end
  end

  def handle_info(msg, st), do: delegate_handle_info(msg, st)

  defp delegate_handle_info(msg, st) do
    case st.module.handle_info(msg, st.view) do
      {:noreply, view} -> {:noreply, %{st | view: view}}
      {:noreply, view, _extra} -> {:noreply, %{st | view: view}}
      {:stop, reason, view} -> {:stop, reason, %{st | view: view}}
    end
  end
end
