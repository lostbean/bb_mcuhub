defmodule BBMCUHub.Test.VHubNif do
  @moduledoc """
  TEST-ONLY NIF over the REAL firmware C chassis (Option B) — the floor + the wire
  framing/decode path, compiled from the same `firmware/src/*.c` the
  `firmware/test/` harnesses host-compile. The soft-fault e2e suite runs the
  **actual** safety code through this, so it cannot drift from the device the way
  an Elixir re-implementation of the floor would.

  Purely functional: every C state value (a `Floor`, a `TransportDecoder`) is
  carried as an opaque Erlang binary and threaded through each call — the NIF holds
  no resources, globals, or threads. See `test/support/c_src/vhub_nif.c`.

  This module is compiled only in `:test` (it lives under `test/support`), and the
  NIF object is built by `elixir_make` (a `:test`-only dep). It is never part of
  the shipped library.
  """
  @on_load :load_nif

  @doc false
  def load_nif do
    path = :filename.join(:code.priv_dir(:bb_mcuhub), ~c"vhub_nif")
    :erlang.load_nif(path, 0)
  end

  @typedoc "Opaque C `Floor` struct, carried as a binary."
  @type floor :: binary()
  @typedoc "Opaque C `TransportDecoder` struct, carried as a binary."
  @type decoder :: binary()

  @doc """
  A born-disarmed floor with `window_ms` and the PACKED `safe_action` bytes
  already selected (ADR-0005: the floor is byte-generic — the safe action is the
  port's value-type value, packed by `BBMCUHub.Wire.Codec.encode_fields/2`).
  """
  @spec floor_init(non_neg_integer(), binary()) :: floor()
  def floor_init(_window_ms, _safe_bytes), do: nif_error()

  @doc "Record a new command (its seq + the PACKED value bytes) on the floor."
  @spec floor_on_command(floor(), 0..0xFFFF, binary()) :: floor()
  def floor_on_command(_floor, _seq, _value_bytes), do: nif_error()

  @doc """
  Run one floor control tick at explicit simulated `now_ms`. Returns the updated
  floor, the PACKED value bytes to DRIVE (the target while armed, the safe action
  otherwise), and the armed flag.
  """
  @spec floor_tick(floor(), non_neg_integer()) :: {floor(), binary(), boolean()}
  def floor_tick(_floor, _now_ms), do: nif_error()

  @doc "Frame a CRC-covered body for the wire: `COBS(body || CRC16(body)) || 0x00`."
  @spec transport_encode(binary()) :: binary() | :error
  def transport_encode(_body), do: nif_error()

  @doc "A fresh streaming frame decoder."
  @spec decoder_new() :: decoder()
  def decoder_new, do: nif_error()

  @doc """
  Feed raw wire bytes; returns the updated decoder, every verified (CRC-clean)
  body peeled off in order, and the running `rx_drop` count (corrupt frames
  dropped at the seam).
  """
  @spec decoder_feed(decoder(), binary()) :: {decoder(), [binary()], non_neg_integer()}
  def decoder_feed(_decoder, _bytes), do: nif_error()

  @doc """
  Parse NODE/PORT/SEQ out of a verified body via the REAL C decoder. `stamped`
  says whether a `t_dev` follows the base header (a per-(node,port) contract fact).
  """
  @spec frame_decode_body(binary(), boolean()) ::
          {:ok, node :: 0..255, port :: 0..255, seq :: 0..0xFFFF} | :error
  def frame_decode_body(_body, _stamped), do: nif_error()

  @doc """
  One hop of the REAL C router (direction-aware, meaning-blind). The `Router` is
  rebuilt from `my_node` + a 256-byte `route_table` binary (`0xFF` = local, else
  a local link index; link 0 = the up-link); the verified `body` is decoded by
  the real C frame path, routed with the LOCAL LINK INDEX it ARRIVED on, and the
  chosen sink's frame is re-framed by the real C encoder — the same
  decode → route → re-frame relay a root hub runs between its links. A frame the
  router refuses to forward (split horizon) comes back as `:drop`.
  """
  @spec router_route(0..255, binary(), 0..255, binary(), boolean()) ::
          {:deliver_local, binary()} | {:send_on_link, 0..255, binary()} | :drop | :error
  def router_route(_my_node, _route_table, _arrival_link, _body, _stamped), do: nif_error()

  defp nif_error, do: :erlang.nif_error(:nif_not_loaded)
end
