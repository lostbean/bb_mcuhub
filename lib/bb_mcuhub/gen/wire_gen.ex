defmodule BBMCUHub.Gen.WireGen do
  @moduledoc """
  The one generator (§06): contracts + topology → one IR → committed artifacts,
  drift-tested so the C and Elixir sides physically cannot diverge.

  The design names four renderings of one model. In this implementation the
  **Elixir codec is data-driven** — `BBMCUHub.Wire.Codec` reads each port's
  value-type layout (`BBMCUHub.ValueType`) and the `BBMCUHub.Contract` header at
  runtime, so it *cannot* drift from the model within Elixir (there is no generated
  Elixir file to fall stale). That leaves the emitters whose output crosses a
  boundary the in-language guarantee can't reach, so they are emitted to disk and
  drift-tested:

    * `emit_c_header/1`     → `firmware/gen/<slug>/wire_contract.h` — port ids,
      packed structs, the floor window constants, and a contract hash.
    * `emit_glue/2`         → `firmware/gen/<slug>/<hub>.glue.h` — the GENERATED
      mechanical per-hub firmware glue: the router table,
      `hub_on_body`, command dispatch, the floor init/`on_command`/`control_tick`/
      status plumbing, the sense ticks, and the `hub_tasks` schedule. This replaces
      the hand-written `main_<hub>.cpp` + the old `schedule.gen.h`.
    * `emit_device_header/2`→ `firmware/gen/<slug>/<hub>.device.h` — the prototypes
      for the hand-written device hooks the glue calls (the contract the
      `mcu/<hub>.{c,cpp}` file owes), signatures owned by each port's value-type.
    * `emit_parity/1`       → `test/fixtures/<slug>/parity_vectors.exs` — `{port,
      value, body, crc}` rows computed by running the *real* encoder, the
      cross-language witness (§03). Numbers are correct by construction.

  Artifacts are **robot-scoped** (§09): each robot owns a `firmware/gen/<slug>/`
  dir (now holding wire_contract.h + the per-hub glue/device headers) and a
  `test/fixtures/<slug>/` dir, so robots can coexist without one clobbering the
  other's artifacts. `<slug>` is the robot module's last segment, underscored
  (`Robot` → `robot`, `SegbyV1` → `segby_v1`). Each hub appears in exactly one
  robot, so `<hub>.glue.h` never collides across robots.

  `write_all!/0` regenerates everything for every committed LIBRARY robot — now
  just the library's own test fixture (the `mix wire.gen` alias). A consumer app
  (e.g. `examples/segby_v1`) generates ITS robot itself via `write_all!/2` with
  its own output base. The drift test asserts each file on
  disk equals what these emitters produce *now*, per robot.
  """

  alias BBMCUHub.Contract
  alias BBMCUHub.Robot.Info
  alias BBMCUHub.ValueType
  alias BBMCUHub.Wire.{Codec, CRC16}

  # Representative scalar per wire type — used to build deterministic parity
  # vectors. Chosen so each field is distinguishable in the bytes.
  @sample_seq 42
  @sample_t_dev 1234

  # The default output base — cwd-relative `firmware/gen` for the C headers and
  # `test/fixtures` for the parity-vector fixture (ADR-0003: WireGen takes an
  # explicit output-base so each app generates into its own tree; the library
  # uses this default, Phase 5's example passes its own base). A consumer can
  # override either via `write_all!/2`.
  @default_base %{gen: ["firmware", "gen"], fixtures: ["test", "fixtures"]}

  # --- top-level ---

  # The robots whose artifacts the LIBRARY commits and drift-tests (ADR-0003):
  # its own test fixture (the drift/C-parity witness). segby_v1 moved to the
  # example app (Phase 5), which generates its own artifacts. There is NO
  # default-robot — the library always generates for an explicit set. `mix
  # wire.gen` (no arg) regenerates ALL of them.
  @robots [BBMCUHub.Test.Fixtures.Robot]

  @doc "The library's committed robots (just the test fixture)."
  @spec robots() :: [module()]
  def robots, do: @robots

  @doc "Regenerate every artifact for every committed library robot. Returns the paths written."
  @spec write_all!() :: [Path.t()]
  def write_all!, do: Enum.flat_map(@robots, &write_all!(&1, @default_base))

  @doc """
  Regenerate every artifact for one explicit robot into the default output base.
  The robot is ALWAYS explicit (no library default).
  """
  @spec write_all!(module()) :: [Path.t()]
  def write_all!(robot), do: write_all!(robot, @default_base)

  @doc """
  Regenerate every artifact for one robot into an explicit output `base`.

  `base` is `%{gen: [path, segments], fixtures: [path, segments]}` — the C headers
  go under `base.gen/<slug>/` and the parity fixture under
  `base.fixtures/<slug>/parity_vectors.exs`. Defaults to the library's own tree;
  a consumer app passes its own base so each app generates into its own tree.

  The `<slug>` defaults to `slug(robot)` (the robot module's last segment,
  underscored). A consumer whose robot module's last segment is generic (e.g.
  `SegbyV1.Robot` → `robot`) can PIN a meaningful slug by putting `:slug` in the
  `base` map (e.g. `%{... , slug: "segby_v1"}`) — the example does this so its
  artifacts land in `firmware/gen/segby_v1/`. Returns the paths written.
  """
  @spec write_all!(module(), map()) :: [Path.t()]
  def write_all!(robot, base) do
    ir = ir(robot)
    slug = Map.get(base, :slug) || slug(robot)

    header = {gen_dir(base, slug, "wire_contract.h"), emit_c_header(ir)}
    parity = {fixtures_path(base, slug), emit_parity(ir)}
    parity_c = {gen_dir(base, slug, "parity_vectors.h"), emit_parity_c(ir)}

    # The per-hub glue + device-prototype headers go in the SAME robot-scoped gen
    # dir as wire_contract.h (§08, ADR-0003). The glue is fully generated and
    # drift-tested; the device header is the contract the hand-written mcu/<hub>.c
    # owes. Each hub lives in exactly one robot, so the per-robot dir never
    # collides across robots.
    glue =
      for hub <- hubs(ir) do
        [
          {gen_dir(base, slug, "#{hub}.glue.h"), emit_glue(ir, hub)},
          {gen_dir(base, slug, "#{hub}.device.h"), emit_device_header(ir, hub)}
        ]
      end
      |> List.flatten()

    for {path, contents} <- [header, parity, parity_c | glue] do
      File.mkdir_p!(Path.dirname(path))
      File.write!(path, contents)
      path
    end
  end

  @doc """
  The robot's short slug — its module's last segment, underscored. Drives the
  per-robot artifact dirs (`Follower` → `follower`, `SegbyV1` → `segby_v1`).
  """
  @spec slug(module()) :: String.t()
  def slug(robot) do
    robot
    |> Module.split()
    |> List.last()
    |> Macro.underscore()
  end

  @doc "The default output base (the library's own tree)."
  @spec default_base() :: map()
  def default_base, do: @default_base

  @doc "The robot-scoped generated-C dir under the DEFAULT base, joined with `file` (§09)."
  @spec gen_dir(String.t(), String.t()) :: Path.t()
  def gen_dir(slug, file), do: gen_dir(@default_base, slug, file)

  @doc "The robot-scoped generated-C dir under `base`, joined with `file` (§09)."
  @spec gen_dir(map(), String.t(), String.t()) :: Path.t()
  def gen_dir(base, slug, file), do: Path.join(base.gen ++ [slug, file])

  @doc "The robot-scoped parity-vector fixture path under the DEFAULT base (§09)."
  @spec fixtures_path(String.t()) :: Path.t()
  def fixtures_path(slug), do: fixtures_path(@default_base, slug)

  @doc "The robot-scoped parity-vector fixture path under `base` (§09)."
  @spec fixtures_path(map(), String.t()) :: Path.t()
  def fixtures_path(base, slug), do: Path.join(base.fixtures ++ [slug, "parity_vectors.exs"])

  @doc "The IR for an explicit robot — the single model the emitters render (no default)."
  @spec ir(module()) :: [Contract.ir_row()]
  def ir(robot) do
    Info.ir(robot)
  end

  defp hubs(ir), do: ir |> Enum.map(& &1.hub) |> Enum.uniq() |> Enum.sort()

  # --- emit: C header ---

  @doc "Render `wire_contract.h` from the IR."
  @spec emit_c_header([Contract.ir_row()]) :: String.t()
  def emit_c_header(ir) do
    types = ir |> Enum.map(& &1.type) |> Enum.uniq() |> Enum.sort()

    body =
      [
        header_banner("wire_contract.h", "ids · structs · floor window · hash"),
        ~s(#ifndef BB_MCUHUB_WIRE_CONTRACT_H\n#define BB_MCUHUB_WIRE_CONTRACT_H\n),
        "#include <stdint.h>\n#include <stdbool.h>\n",
        "/* The CRC-covered body header, before every payload (§03). t_dev is",
        "   per-port (§04): an unstamped port omits it. */",
        "#define WIRE_HEADER_BASE_SIZE #{Contract.header_size(false)}",
        "#define WIRE_HEADER_STAMPED_SIZE #{Contract.header_size(true)}",
        "#define WIRE_BROADCAST_NODE 0x#{hex2(Contract.broadcast_node())}",
        "",
        "/* The root hub's node id (ADR-0006). Root-ness is DECLARED — the hub with",
        "   parent: :host is root — so the firmware derives IS_ROOT = (MY_NODE ==",
        "   ROOT_NODE) instead of hand-setting a -DROOT_HUB build flag. Always emitted",
        "   (a single-hub robot still declares parent: :host). */",
        root_node_define(ir),
        "",
        "/* Per-link transport of the root hub's DOWNLINKS (ADR-0006). Transport is a",
        "   property of a LINK, not a robot-wide flag: each downlink k carries",
        "   LINK<k>_TRANSPORT_UART = 1 (a plain UART carrying the same COBS+CRC frame,",
        "   no CAN segmentation) or 0 (the default CAN/TWAI backplane). For the",
        "   single-downlink example boards this is just LINK1_*, equivalent to the old",
        "   single-backplane flag but sourced from the child's declared uplink, not",
        "   inferred. The board's link_esp32.cpp realizes only the links it has. */",
        root_link_transport_defines(ir),
        "",
        "/* Port ids — generated, stable, never hand-assigned (§06). */",
        Enum.map_join(ir, "\n", &port_define/1),
        "",
        "/* Which ports carry t_dev (1) vs omit it (0) — see §04. */",
        Enum.map_join(ir, "\n", &stamped_define/1),
        "",
        "/* Decode-time lookup: does the frame for (node, port) carry t_dev? The",
        "   inbound path peeks the base header for (node, port), then calls this to",
        "   learn the header shape — exactly as the host learns it from PortIndex. */",
        stamped_lookup_fn(ir),
        "",
        "/* Packed value structs — field order MATCHES Contract.Layouts (§06). */",
        Enum.map_join(types, "\n\n", &struct_def/1),
        "",
        "/* Per-actuator floor window, derived from one number (§05). */",
        Enum.map_join(actuators(ir), "\n", &floor_defines/1),
        "",
        "/* Contract hash — the drift test compares this. */",
        ~s(#define WIRE_CONTRACT_SHA "#{contract_sha(ir)}"),
        "",
        "#endif /* BB_MCUHUB_WIRE_CONTRACT_H */"
      ]

    finalize(body)
  end

  defp port_define(row) do
    "#define PORT_#{up(row.hub)}_#{up(row.port)} 0x#{hex2(row.port_id)}"
  end

  defp stamped_define(row) do
    "#define PORT_#{up(row.hub)}_#{up(row.port)}_STAMPED #{if row.stamped, do: 1, else: 0}"
  end

  defp stamped_lookup_fn(ir) do
    cases =
      ir
      |> Enum.filter(& &1.stamped)
      |> Enum.map_join("\n", fn row ->
        "    if (node == 0x#{hex2(row.node)} && port == 0x#{hex2(row.port_id)}) return true;"
      end)

    """
    static inline bool wire_port_stamped(uint8_t node, uint8_t port) {
    #{cases}
      return false;
    }\
    """
  end

  defp struct_def(type) do
    fields =
      ValueType.resolve(type).layout()
      |> Enum.map_join("\n", fn {field, wt} -> "  #{c_type(wt)} #{field};" end)

    "typedef struct __attribute__((packed)) {\n#{fields}\n} #{c_struct_name(type)};"
  end

  defp floor_defines(row) do
    period_ms = div(1000, row.rate)
    # The safe action, packed to bytes by the SAME layout codec the wire uses
    # (ADR-0005). The floor stores/drives these opaque bytes verbatim; emitting
    # them here makes the safe-state contract a byte-exact, drift-tested artifact.
    safe = Codec.encode_fields(row.layout, row.safe_action)
    safe_bytes = safe |> :binary.bin_to_list() |> Enum.map_join(", ", &"0x#{hex2(&1)}")

    [
      "#define FLOOR_MISSES_#{up(row.hub)}_#{up(row.port)} #{row.fresh_for}",
      "#define CMD_PERIOD_MS_#{up(row.hub)}_#{up(row.port)} #{period_ms}",
      "#define SAFE_N_#{up(row.hub)}_#{up(row.port)} #{byte_size(safe)}",
      "static const uint8_t SAFE_#{up(row.hub)}_#{up(row.port)}[] = { #{safe_bytes} }; /* packed safe_action #{inspect(row.safe_action, custom_options: [sort_maps: true])} */"
    ]
    |> Enum.join("\n")
  end

  # --- emit: per-hub generated glue (§08, ADR-0003) ---
  #
  # The hand-written `main_<hub>.cpp` was mechanical: a router table, hub_on_body,
  # command dispatch, the floor init/on_command/control_tick/status plumbing, the
  # sense ticks, and the hub_tasks schedule assembly. ALL of it is derivable from
  # the IR, so it is generated here into `<hub>.glue.h`. The user is left only the
  # device hooks (`<hub>_device_setup`, per-port read/drive) declared in
  # `<hub>.device.h` and implemented in a hand-authored `mcu/<hub>.{c,cpp}`.
  #
  # Derivations, all from the IR (documented for the safety review):
  #
  #   * ROOT hub  — the hub that DECLARES `parent: :host` (ADR-0006), no more
  #     Enum.min inference. Topology is declared by parent links; each hub gets a
  #     LOCAL link list (link 0 = the up-link toward its parent/host; downlinks
  #     1..N from its children, grouped by transport) and a node → local-link-index
  #     route table: MY_NODE → LINK_LOCAL_IDX, a descendant reached via downlink k
  #     → k, everything else (ancestors/unknown) → 0 (the up-link). hub_on_body
  #     routes onto `link_send_on_link(idx, f)`; the board realizes the links it
  #     physically has and stubs the rest. A LEAF has only link 0 (up), so its
  #     route table is the default-0 fill + MY_NODE → LINK_LOCAL_IDX.
  #
  #   * FLOORED command port — `dir: :in` AND `has_safe_action == true`
  #     (ADR-0005). Gets a Floor, an `on_command_<port>` that hands the floor the
  #     RAW PAYLOAD BYTES + seq (no value decode), and a per-loop drive in
  #     control_tick that feeds the floor's CURRENT BYTES to the device hook. The
  #     floor window = FLOOR_MISSES_* * CMD_PERIOD_MS_* (from wire_contract.h); the
  #     safe_action is a value of the port's value-type, packed by the SAME layout
  #     codec the wire uses and emitted as the C byte array SAFE_<HUB>_<PORT>.
  #
  #   * NON-FLOORED command port — `dir: :in` AND `has_safe_action == false`
  #     (segby's `status_led`). No floor: `on_command_<port>` decodes the value and
  #     calls the device drive hook directly, reproducing the hand-written LED path.
  #
  #   * STATUS port — `dir: :out`, `type: :status`. Reports {applied_seq, floored?}
  #     of its PAIRED floored command port (see `status_pair/2`).
  #
  #   * SENSE port — `dir: :out`, not :status. A scheduled read → pack → send-up.
  #
  #   * DRIVE/READ hook signatures are owned by the value-type (§ Firmware hook): a
  #     single numeric field → scalar `(float)` (effort) or `bool read(<Struct>*)`;
  #     a multi-field value → struct-pointer (`const <Struct>*` to drive, `<Struct>*`
  #     to read). See `drive_sig/2` / `read_call/2`.
  #
  #   * OPTIONAL post-control hook — the glue ALWAYS calls `<hub>_post_control()` at
  #     the end of control_tick AND emits a `__attribute__((weak))` empty default,
  #     so a device that needs it (wheels' current sense) overrides it and one that
  #     doesn't links fine with the no-op. Portable C, no #ifdef per hub.

  @doc """
  Render a hub's `<hub>.glue.h` — all mechanical per-hub firmware wiring (§08).

  This replaces the hand-written `main_<hub>.cpp`. It defines the C-linkage
  `hub_setup`/`hub_on_body`/`hub_tasks` the generic `hub_main.cpp` expects, plus
  the router table, command dispatch, floor plumbing, status + sense ticks, and
  the schedule. ARDUINO-guarded so the host C harnesses (which never compile it)
  still build.
  """
  @spec emit_glue([Contract.ir_row()], atom()) :: String.t()
  def emit_glue(ir, hub) do
    rows = ir |> Enum.filter(&(&1.hub == hub)) |> Enum.sort_by(& &1.port_id)
    root? = root_hub?(ir, hub)
    my_node = rows |> hd() |> Map.fetch!(:node)

    floored = Enum.filter(rows, &floored_command?/1)
    nonfloored = Enum.filter(rows, &nonfloored_command?/1)
    statuses = Enum.filter(rows, &status_port?/1)
    senses = Enum.filter(rows, &sense_port?/1)
    actuator? = floored != []

    finalize([
      header_banner("#{hub}.glue.h", "generated per-hub glue (router, floor, ticks) — §08"),
      glue_intro(hub, root?),
      "#if defined(ARDUINO)",
      "",
      ~s(#include <Arduino.h>),
      "extern \"C\" {",
      "#include \"frame.h\"",
      "#include \"scheduler.h\"",
      "#include \"router.h\"",
      "#include \"link.h\"",
      if(actuator?, do: "#include \"floor.h\"", else: nil),
      "#include \"wire_contract.h\"",
      ~s(#include "#{hub}.device.h" /* the hand-written device hooks */),
      "}",
      "",
      "#ifndef MY_NODE",
      "#define MY_NODE 0x#{hex2(my_node)}",
      "#endif",
      "",
      glue_link_model(hub, ir),
      glue_post_control_default(hub),
      glue_floor_state(floored),
      glue_sense_state(senses),
      glue_status_state(statuses),
      glue_on_commands(hub, floored, nonfloored),
      glue_control_tick(hub, floored, actuator?),
      glue_cmd_ticks(rows),
      glue_status_ticks(hub, statuses, floored),
      glue_sense_ticks(hub, senses),
      glue_router(hub, rows, root?),
      glue_setup(hub, floored, ir),
      glue_tasks(hub, rows, actuator?),
      "#endif /* ARDUINO */"
    ])
  end

  @doc """
  Render a hub's `<hub>.device.h` — the prototypes for the hand-written device
  hooks (the contract the `mcu/<hub>.{c,cpp}` file owes). Legible, link-time
  resolved. Signatures are owned by each port's value-type.
  """
  @spec emit_device_header([Contract.ir_row()], atom()) :: String.t()
  def emit_device_header(ir, hub) do
    rows = ir |> Enum.filter(&(&1.hub == hub)) |> Enum.sort_by(& &1.port_id)
    cmds = Enum.filter(rows, &(&1.dir == :in))
    senses = Enum.filter(rows, &sense_port?/1)
    guard = "BB_MCUHUB_#{up(hub)}_DEVICE_H"

    setup_proto =
      "void #{hub}_device_setup(void); /* one-time bring-up (pins, peripherals,\n" <>
        "                                    FOC/encoder init). May take as long as it\n" <>
        "                                    needs: the task watchdog is armed AFTER this\n" <>
        "                                    returns, so a slow initFOC/i2c settle is safe. */"

    read_protos =
      Enum.map(senses, fn row ->
        "bool #{hub}_#{row.port}_read(#{c_struct_name(row.type)} *out); /* bounded read; false on timeout */"
      end)

    drive_protos =
      Enum.map(cmds, fn row ->
        "void #{hub}_#{row.port}_drive(#{drive_param(row)}); /* apply to the plant */"
      end)

    post_control =
      "void #{hub}_post_control(void); /* OPTIONAL: per-loop telemetry. To provide one, `#define #{up(hub)}_POST_CONTROL_OVERRIDE` before #include'ing #{hub}.glue.h; else a no-op default is used. */"

    finalize([
      header_banner(
        "#{hub}.device.h",
        "device-hook prototypes — the hand-written contract (§ Firmware hook)"
      ),
      "#ifndef #{guard}",
      "#define #{guard}",
      "#include <stdint.h>",
      "#include <stdbool.h>",
      "#include \"wire_contract.h\" /* the packed value structs the hooks fill/take */",
      "",
      "#ifdef __cplusplus",
      "extern \"C\" {",
      "#endif",
      "",
      "/* Implemented by the hand-authored mcu/#{hub}.{c,cpp}; the generated glue calls these. */",
      [setup_proto] |> Enum.join("\n"),
      if(read_protos == [], do: nil, else: Enum.join(read_protos, "\n")),
      if(drive_protos == [], do: nil, else: Enum.join(drive_protos, "\n")),
      post_control,
      "",
      "#ifdef __cplusplus",
      "}",
      "#endif",
      "",
      "#endif /* #{guard} */"
    ])
  end

  # --- glue: section renderers ---

  # The per-hub LOCAL link model (ADR-0006): link 0 is the up-link; downlinks
  # 1..N are this hub's children grouped by transport (CAN siblings → one bus;
  # each UART child → its own link). Emitted as a legible comment + a
  # <HUB>_N_LINKS define so the link layer knows how many links this hub has;
  # link_esp32.cpp realizes the ones the board physically owns and stubs the rest.
  defp glue_link_model(hub, ir) do
    links = downlinks(ir, hub)
    n_links = 1 + length(links)

    down_lines =
      Enum.map(links, fn link ->
        nodes =
          link.members
          |> Enum.flat_map(&subtree_nodes(ir, &1))
          |> Enum.sort()
          |> Enum.map_join(", ", &"0x#{hex2(&1)}")

        "      link #{link.idx}: #{link.transport} → node(s) #{nodes}"
      end)

    up_line =
      if root_hub?(ir, hub),
        do: "      link 0 (up): the host UART (the root owns the host link)",
        else: "      link 0 (up): the parent backplane"

    comment =
      (["/* This hub's #{n_links} LOCAL link(s) (ADR-0006):", up_line] ++ down_lines ++ ["   */"])
      |> Enum.join("\n")

    "#{comment}\n#define #{up(hub)}_N_LINKS #{n_links}\n"
  end

  defp glue_intro(hub, root?) do
    role =
      if root?,
        do: "ROOT hub (host UART ↔ backplane + local ports)",
        else: "LEAF hub (local ports only)"

    [
      "/* GENERATED firmware glue for the #{hub} hub — #{role}.",
      "   Mechanical wiring derived from the IR (§08): router table, hub_on_body,",
      "   command dispatch, the floor init/on_command/control_tick/status plumbing,",
      "   the sense ticks, and the hub_tasks schedule. The user writes ONLY the",
      "   device hooks in mcu/#{hub}.{c,cpp} (prototypes in #{hub}.device.h).",
      "   This header is #include'd by the device file, which the build compiles. */"
    ]
    |> Enum.join("\n")
  end

  # The optional per-loop hook. The glue ALWAYS calls `<hub>_post_control()` at the
  # end of control_tick; here it emits a no-op DEFAULT body UNLESS the device file
  # signalled it provides its own by `#define <HUB>_POST_CONTROL_OVERRIDE` before
  # including the glue. A weak attribute can't be used because the glue is included
  # INTO the device TU (a weak default + a strong override in one TU is a
  # redefinition error), so this compile-time switch is the portable form: a hub
  # that needs it (wheels' current sense) defines the macro and supplies the real
  # body; one that doesn't gets the no-op for free.
  defp glue_post_control_default(hub) do
    """
    /* Optional per-loop telemetry hook (§08). The glue calls #{hub}_post_control() at
       the end of every control_tick. The device file can provide its own by doing
       `#define #{up(hub)}_POST_CONTROL_OVERRIDE` BEFORE including this header (then
       implementing #{hub}_post_control() itself); otherwise this no-op default is used. */
    #ifndef #{up(hub)}_POST_CONTROL_OVERRIDE
    extern "C" void #{hub}_post_control(void) {}
    #endif
    """
  end

  defp glue_floor_state([]), do: nil

  defp glue_floor_state(floored) do
    decls =
      Enum.flat_map(floored, fn row ->
        win =
          "(FLOOR_MISSES_#{up(row.hub)}_#{up(row.port)} * CMD_PERIOD_MS_#{up(row.hub)}_#{up(row.port)})"

        [
          "static Floor g_floor_#{row.port};",
          "static uint16_t g_applied_seq_#{row.port} = 0;  /* last command seq we handed the floor */",
          "#define FLOOR_WINDOW_MS_#{up(row.port)} #{win}"
        ]
      end)

    "/* One floor per floored command port (§05) — born-disarmed, safe at boot. */\n" <>
      Enum.join(decls, "\n")
  end

  defp glue_sense_state([]), do: nil

  defp glue_sense_state(senses) do
    "/* Per-sensor seq — advanced only on a real new value (§04). */\n" <>
      Enum.map_join(senses, "\n", &"static uint16_t g_seq_#{&1.port} = 0;")
  end

  defp glue_status_state([]), do: nil

  defp glue_status_state(statuses) do
    "/* Per-status-port seq. */\n" <>
      Enum.map_join(statuses, "\n", &"static uint16_t g_status_seq_#{&1.port} = 0;")
  end

  defp glue_on_commands(_hub, [], []), do: nil

  defp glue_on_commands(hub, floored, nonfloored) do
    floored_fns =
      Enum.map(floored, fn row ->
        min_len = payload_min_len(ValueType.resolve(row.type).layout())

        """
        /* A command for #{row.port}: hand the floor the RAW PACKED VALUE BYTES +
           its seq — the floor watches the seq, not the value, and stores the bytes
           opaquely (ADR-0005). Record applied_seq for the status report (§05). */
        static void on_command_#{row.port}(const Frame *f) {
          if (f->port != PORT_#{up(hub)}_#{up(row.port)}) return;
          if (f->payload_len < #{min_len}) return;
          floor_on_command(&g_floor_#{row.port}, f->seq, f->payload, f->payload_len);
          g_applied_seq_#{row.port} = f->seq;
        }\
        """
      end)

    nonfloored_fns =
      Enum.map(nonfloored, fn row ->
        glue_nonfloored_on_command(hub, row)
      end)

    Enum.join(floored_fns ++ nonfloored_fns, "\n\n")
  end

  # A non-floored command port (e.g. segby's decorative LED): no safe-state, just
  # decode the value and drive directly. Reproduces the hand-written LED path.
  defp glue_nonfloored_on_command(hub, row) do
    layout = ValueType.resolve(row.type).layout()
    min_len = payload_min_len(layout)

    {decode, drive_arg} =
      case layout do
        [{_field, _wt}] ->
          # single field → pass the scalar (only effort uses this path today;
          # a single-:u8 would also land here but no such command port exists)
          {"  #{c_type(elem(hd(layout), 1))} v = #{scalar_get(elem(hd(layout), 1), 0)};", "v"}

        _ ->
          # multi-field → build the packed struct, pass a pointer (LED's r,g,b)
          fields =
            layout
            |> Enum.with_index()
            |> Enum.map_join("\n", fn {{field, wt}, idx} ->
              "  v.#{field} = #{scalar_get(wt, field_offset(layout, idx))};"
            end)

          {"  #{c_struct_name(row.type)} v;\n" <> fields, "&v"}
      end

    """
    /* A command for #{row.port}: NOT floored (safe_action == nil), so decode the
       value and drive the device directly — a stale command is harmless (§09). */
    static void on_command_#{row.port}(const Frame *f) {
      if (f->port != PORT_#{up(hub)}_#{up(row.port)}) return;
      if (f->payload_len < #{min_len}) return;
    #{decode}
      #{hub}_#{row.port}_drive(#{drive_arg});
    }\
    """
  end

  defp glue_control_tick(_hub, [], false), do: nil

  defp glue_control_tick(hub, floored, true) do
    drives = Enum.map_join(floored, "\n", &glue_floor_drive(hub, &1))

    """
    /* The drive loop — period 0 so it runs every loop pass, never starved (§08).
       Each floor gates its own port: target while armed, safe action otherwise
       (default safe). The floor hands back the PACKED VALUE BYTES (ADR-0005); the
       glue decodes them to the hook's typed param. Then the optional per-loop
       device hook (telemetry). */
    static void control_tick(uint32_t now_us) {
      uint32_t now_ms = now_us / 1000u;
    #{drives}
      #{hub}_post_control();
    }\
    """
  end

  defp glue_control_tick(_hub, _floored, _actuator?), do: nil

  # One floored port's drive: tick the floor into a packed-bytes buffer, then
  # decode those bytes to the device hook's expected param (ADR-0005). The HOOK
  # SIGNATURE is stable — a single-f32 port still gets a scalar `float`, a
  # multi-field port a `const <Struct> *`. The floor itself stays byte-generic.
  defp glue_floor_drive(hub, row) do
    layout = ValueType.resolve(row.type).layout()
    buf = "drive_#{row.port}"

    case layout do
      [{_field, wt}] ->
        # single field → decode the one scalar from the floor's bytes, pass it.
        """
          uint8_t #{buf}[FLOOR_MAX_VALUE];
          floor_tick(&g_floor_#{row.port}, now_ms, #{buf});
          #{hub}_#{row.port}_drive(#{buf_get(wt, buf, 0)});\
        """

      _ ->
        # multi-field → rebuild the packed struct from the floor's bytes, pass a
        # pointer (matches the drive-hook's `const <Struct> *` signature).
        fields =
          layout
          |> Enum.with_index()
          |> Enum.map_join("\n", fn {{field, wt}, idx} ->
            "    #{buf}_v.#{field} = #{buf_get(wt, buf, field_offset(layout, idx))};"
          end)

        """
          uint8_t #{buf}[FLOOR_MAX_VALUE];
          floor_tick(&g_floor_#{row.port}, now_ms, #{buf});
          #{c_struct_name(row.type)} #{buf}_v;
        #{fields}
          #{hub}_#{row.port}_drive(&#{buf}_v);\
        """
    end
  end

  # Big-endian field GET at `off` from a plain `uint8_t *` buffer (the floor's
  # packed-bytes output), for wire type `wt`.
  defp buf_get(:f32, b, off), do: "be_get_f32(&#{b}[#{off}])"
  defp buf_get(:f64, b, off), do: "be_get_f64(&#{b}[#{off}])"
  defp buf_get(:u8, b, off), do: "#{b}[#{off}]"
  defp buf_get(:u16, b, off), do: "be_get_u16(&#{b}[#{off}])"
  defp buf_get(:u32, b, off), do: "be_get_u32(&#{b}[#{off}])"
  defp buf_get(:u64, b, off), do: "be_get_u64(&#{b}[#{off}])"
  defp buf_get(:bool, b, off), do: "(#{b}[#{off}] != 0)"

  # Command ports are event-driven via on_command; their scheduled cmd tick (the
  # schedule lists every port) is a no-op, matching the hand-written hubs.
  defp glue_cmd_ticks(rows) do
    cmds = Enum.filter(rows, &(&1.dir == :in))
    if cmds == [], do: nil, else: cmd_ticks_body(cmds)
  end

  defp cmd_ticks_body(cmds) do
    "/* IN ports are event-driven via on_command; the scheduled cmd tick is a no-op. */\n" <>
      Enum.map_join(
        cmds,
        "\n",
        &"static void #{&1.port}_cmd_tick(uint32_t now_us) { (void)now_us; }"
      )
  end

  defp glue_status_ticks(_hub, [], _floored), do: nil

  defp glue_status_ticks(hub, statuses, floored) do
    Enum.map_join(statuses, "\n\n", fn st ->
      pair = status_pair(st, floored)

      """
      /* Report #{st.port}'s reported truth (§05): the paired floor's applied_seq +
         floored? flag, so the host reads truth instead of inferring it. */
      static void #{st.port}_sample_tick(uint32_t now_us) {
        (void)now_us; /* status omits t_dev, so the tick needs no clock */
        Frame f;
        f.node = MY_NODE;
        f.port = PORT_#{up(hub)}_#{up(st.port)};
        f.seq = ++g_status_seq_#{st.port};
        f.stamped = PORT_#{up(hub)}_#{up(st.port)}_STAMPED;
        f.t_dev = 0;

        be_put_u16(&f.payload[0], g_applied_seq_#{pair.port});
        f.payload[2] = g_floor_#{pair.port}.armed ? 0 : 1; /* floored? = not armed */
        f.payload_len = 3;

        link_send_up(&f);
      }\
      """
    end)
  end

  defp glue_sense_ticks(_hub, []), do: nil

  defp glue_sense_ticks(hub, senses) do
    Enum.map_join(senses, "\n\n", fn row ->
      layout = ValueType.resolve(row.type).layout()
      struct = c_struct_name(row.type)

      pack =
        layout
        |> Enum.with_index()
        |> Enum.map_join("\n", fn {{field, wt}, idx} ->
          "  #{scalar_put(wt, field_offset(layout, idx), "raw.#{field}")}"
        end)

      payload_len = payload_min_len(layout)

      tdev =
        if row.stamped do
          "  f.t_dev = now_us;                 /* this hub's own µs, same-device use only */"
        else
          "  f.t_dev = 0;\n  (void)now_us;"
        end

      """
      /* Sense #{row.port} (§08): one bounded read, pack big-endian, send up. A read
         that fails returns → no write → the seq stalls → the reader goes stale. */
      static void #{row.port}_sample_tick(uint32_t now_us) {
        #{struct} raw;
        if (!#{hub}_#{row.port}_read(&raw)) return;

        Frame f;
        f.node = MY_NODE;
        f.port = PORT_#{up(hub)}_#{up(row.port)};
        f.seq = ++g_seq_#{row.port};        /* advance only on a real new value */
        f.stamped = PORT_#{up(hub)}_#{up(row.port)}_STAMPED;
      #{tdev}

      #{pack}
        f.payload_len = #{payload_len};

        link_send_up(&f);
      }\
      """
    end)
  end

  defp glue_router(hub, rows, root?) do
    cmds = Enum.filter(rows, &(&1.dir == :in))

    deliver =
      if cmds == [] do
        """
        static void deliver_local(const Frame *f, void *) {
          (void)f; /* sense-only hub: no local command ports */
        }\
        """
      else
        dispatch =
          cmds
          |> Enum.with_index()
          |> Enum.map_join("\n", fn {row, idx} ->
            kw = if idx == 0, do: "if", else: "else if"
            "  #{kw} (f->port == PORT_#{up(hub)}_#{up(row.port)}) on_command_#{row.port}(f);"
          end)

        """
        static void deliver_local(const Frame *f, void *) {
        #{dispatch}
        }\
        """
      end

    # The router dispatches onto a per-hub-local LINK INDEX (ADR-0006): link 0 is
    # the up-link (toward parent/host); downlinks are 1..N. The board's
    # `link_send_on_link` maps the index to its peripheral (and stubs links it
    # doesn't physically have). A LEAF only ever has link 0 requested; a branch
    # routes descendants to their downlink index.
    send_note =
      if root?,
        do: "/* link 0 = host UART (up); downlinks 1..N = backplanes (ADR-0006) */",
        else: "/* leaf: only link 0 (up toward the parent) is ever requested */"

    send_decl =
      """
      static void send_on_link(uint8_t link, const Frame *f, void *) {
        link_send_on_link(link, f); #{send_note}
      }
      """

    [
      "static Router g_router;",
      "",
      deliver,
      send_decl,
      """

      /* Meaning-blind inbound (§04): decode the body (CRC-clean at the seam), learn
         t_dev-ness per port just-in-time, route by DIRECTION + NODE (ADR-0006,
         ADR-0011): a downlink arrival ascends to link 0; a parent arrival is
         dispatched by NODE → a LOCAL LINK INDEX. seq/t_dev never touched. */
      extern "C" void hub_on_body(uint8_t arrival_link, const uint8_t *body, size_t len) {
        if (len < FRAME_HEADER_BASE_SIZE) return;
        Frame f;
        bool stamped = wire_port_stamped(body[0], body[1]); /* per-port t_dev (§04) */
        if (!frame_decode_body(body, len, stamped, &f)) return;
        RouterSinks sinks = {deliver_local, send_on_link, nullptr};
        router_route(&g_router, &f, arrival_link, &sinks);
      }\
      """
    ]
    |> Enum.reject(&is_nil/1)
    |> Enum.join("\n")
  end

  defp glue_setup(hub, floored, ir) do
    floor_inits =
      Enum.map_join(floored, "\n", fn row ->
        "  floor_init(&g_floor_#{row.port}, FLOOR_WINDOW_MS_#{up(row.port)}, SAFE_#{up(row.hub)}_#{up(row.port)}, SAFE_N_#{up(row.hub)}_#{up(row.port)}); /* #{inspect(row.safe_action, custom_options: [sort_maps: true])} */"
      end)

    # The node → LOCAL LINK INDEX route fill (ADR-0006). Default 0 (the up-link
    # toward the parent/host); MY_NODE → LINK_LOCAL_IDX; a descendant reached via
    # downlink k → k. A leaf has no downlinks, so only the default + self appear.
    entries = route_entries(ir, hub)

    routes =
      Enum.map_join(entries, "\n", fn
        {node, :local} ->
          "  g_router.route_table[0x#{hex2(node)}] = LINK_LOCAL_IDX; /* MY_NODE — delivered to a local port */"

        {node, idx} ->
          "  g_router.route_table[0x#{hex2(node)}] = #{idx}; /* reached via downlink #{idx} (ADR-0006) */"
      end)

    route_fill =
      """
        g_router.my_node = MY_NODE;
        for (int i = 0; i < 256; i++) g_router.route_table[i] = 0; /* default: link 0, the up-link toward the parent/host */
      #{routes}\
      """

    floor_block =
      if floored == [] do
        nil
      else
        "  /* born-disarmed floors first (§05), so the safe output is selected before any drive */\n" <>
          floor_inits <> "\n"
      end

    [
      """
      extern "C" void hub_setup(void) {
      """
      |> String.trim_trailing(),
      floor_block,
      route_fill,
      "",
      "  #{hub}_device_setup(); /* the hand-written hardware bring-up */",
      "}"
    ]
    |> Enum.reject(&is_nil/1)
    |> Enum.join("\n")
  end

  defp glue_tasks(hub, rows, actuator?) do
    sorted = Enum.sort_by(rows, & &1.port_id)

    table =
      sorted
      |> Enum.map_join(",\n", fn row ->
        period_us = div(1_000_000, row.rate)
        "  { #{period_us}, 0, #{tick_name(row)} }  /* #{row.port} @ #{row.rate} Hz */"
      end)

    if actuator? do
      """
      /* The schedule (§08): one {period_us, tick} row per port from its rate. The
         drive loop runs EVERY loop (period 0) and is prepended here, never starved. */
      static Task #{hub}_tasks[] = {
        { 0, 0, control_tick },  /* period 0 → every loop pass */
      #{table}
      };
      #define #{up(hub)}_N_TASKS (sizeof(#{hub}_tasks) / sizeof(#{hub}_tasks[0]))

      extern "C" Task *hub_tasks(size_t *n_tasks) {
        *n_tasks = #{up(hub)}_N_TASKS;
        return #{hub}_tasks;
      }\
      """
    else
      """
      /* The schedule (§08): one {period_us, tick} row per port from its rate. A
         sense-only hub has no control_tick. */
      static Task #{hub}_tasks[] = {
      #{table}
      };
      #define #{up(hub)}_N_TASKS (sizeof(#{hub}_tasks) / sizeof(#{hub}_tasks[0]))

      extern "C" Task *hub_tasks(size_t *n_tasks) {
        *n_tasks = #{up(hub)}_N_TASKS;
        return #{hub}_tasks;
      }\
      """
    end
  end

  # --- glue: topology / per-hub link model (ADR-0006) ---
  #
  # Topology is DECLARED, not inferred. Each IR row carries its hub's `parent`
  # (another hub's name, or :host for the root) and `uplink` (the transport of
  # this hub's parent link). From those we build, per hub, its LOCAL link list and
  # a node → local-link-index route table.
  #
  # Per-hub link derivation:
  #   * Link 0 is ALWAYS the up-link (toward the parent; the host UART for root).
  #   * Downlinks 1..N are this hub's CHILDREN grouped by transport: children that
  #     share a CAN uplink share ONE bus ⇒ one link; each UART child is its OWN
  #     link. Groups are ordered deterministically by the min child node id in the
  #     group, so generation is stable and drift-testable.
  #   * route_table[node]: MY_NODE → LINK_LOCAL_IDX; a descendant reached via
  #     downlink k → k; everything else (ancestors / unknown) → 0 (the up-link).

  @host :host

  # The unique hub models from the IR: %{name, node, parent, uplink}, sorted by
  # node for determinism. Each hub's rows all carry the same parent/uplink/node.
  defp hub_models(ir) do
    ir
    |> Enum.group_by(& &1.hub)
    |> Enum.map(fn {name, [row | _]} ->
      %{name: name, node: row.node, parent: row.parent, uplink: row.uplink}
    end)
    |> Enum.sort_by(& &1.node)
  end

  defp hub_model(ir, hub), do: Enum.find(hub_models(ir), &(&1.name == hub))

  # The ROOT hub DECLARES parent: :host (ADR-0006) — no more Enum.min inference.
  defp root_hub?(ir, hub), do: hub_model(ir, hub).parent == @host

  # The hub's direct children (hubs whose parent is this hub), as hub models.
  defp children_of(ir, hub) do
    hub_models(ir) |> Enum.filter(&(&1.parent == hub))
  end

  # This hub's DOWNLINKS as a list of %{idx, transport, members}, idx starting at
  # 1. Children are grouped by (shared CAN uplink ⇒ one bus) vs (each UART child ⇒
  # its own link); groups ordered by min member node id (deterministic).
  defp downlinks(ir, hub) do
    children = children_of(ir, hub)

    # CAN children all share ONE bus (one link); each UART child is its own link.
    {can_children, uart_children} = Enum.split_with(children, &(&1.uplink == :can))

    can_group =
      if can_children == [], do: [], else: [%{transport: :can, members: can_children}]

    uart_groups = Enum.map(uart_children, &%{transport: :uart, members: [&1]})

    (can_group ++ uart_groups)
    |> Enum.sort_by(fn g -> g.members |> Enum.map(& &1.node) |> Enum.min() end)
    |> Enum.with_index(1)
    |> Enum.map(fn {g, idx} -> Map.put(g, :idx, idx) end)
  end

  # All NODE ids in the subtree rooted at a child hub (the child + its descendants).
  defp subtree_nodes(ir, hub_model) do
    descendants =
      children_of(ir, hub_model.name)
      |> Enum.flat_map(&subtree_nodes(ir, &1))

    [hub_model.node | descendants]
  end

  # The route table for a hub: a list of {node, link_index_or_local} for every
  # node that resolves to something other than the up-link default. MY_NODE →
  # :local; a node in downlink k's subtree → k. Everything else falls through to
  # the up-link (link 0) by the default fill, so it is NOT listed here.
  defp route_entries(ir, hub) do
    my_node = hub_model(ir, hub).node

    down_entries =
      for link <- downlinks(ir, hub),
          member <- link.members,
          node <- subtree_nodes(ir, member) do
        {node, link.idx}
      end

    [{my_node, :local} | down_entries]
    |> Enum.sort_by(fn {node, _} -> node end)
  end

  defp floored_command?(row), do: row.dir == :in and row.has_safe_action == true
  defp nonfloored_command?(row), do: row.dir == :in and row.has_safe_action == false
  defp status_port?(row), do: row.dir == :out and row.type == :status
  defp sense_port?(row), do: row.dir == :out and row.type != :status

  # Map a status OUT-port to the floored command port whose truth it reports.
  #
  #   * one floored command port → every status reports that one floor (motor).
  #   * many → pair by a shared name suffix (status_LEFT ↔ motor_LEFT,
  #     status_RIGHT ↔ motor_RIGHT on wheels). The in-tree hubs (motor, wheels)
  #     are the only actuators this phase; the rule reproduces their exact pairing.
  defp status_pair(_status, [only]), do: only

  defp status_pair(status, floored) do
    suffix = name_suffix(status.port)

    Enum.find(floored, fn cmd -> name_suffix(cmd.port) == suffix end) ||
      raise "no floored command pairs status port #{status.port} (suffix #{suffix})"
  end

  defp name_suffix(port), do: port |> Atom.to_string() |> String.split("_") |> List.last()

  # The drive-hook *parameter* declaration, from the value-type layout (§ Firmware
  # hook): a single numeric field → the scalar by value; a multi-field value → a
  # const pointer to the packed struct.
  defp drive_param(row) do
    case ValueType.resolve(row.type).layout() do
      [{_f, wt}] -> c_type(wt)
      _ -> "const #{c_struct_name(row.type)} *v"
    end
  end

  # Big-endian field GET at `off` for wire type `wt` (decode in on_command).
  defp scalar_get(:f32, off), do: "be_get_f32(&f->payload[#{off}])"
  defp scalar_get(:f64, off), do: "be_get_f64(&f->payload[#{off}])"
  defp scalar_get(:u8, off), do: "f->payload[#{off}]"
  defp scalar_get(:u16, off), do: "be_get_u16(&f->payload[#{off}])"
  defp scalar_get(:u32, off), do: "be_get_u32(&f->payload[#{off}])"
  defp scalar_get(:u64, off), do: "be_get_u64(&f->payload[#{off}])"
  defp scalar_get(:bool, off), do: "(f->payload[#{off}] != 0)"

  # Big-endian field PUT at `off` for wire type `wt` (pack in a sense/status tick).
  defp scalar_put(:f32, off, src), do: "be_put_f32(&f.payload[#{off}], #{src});"
  defp scalar_put(:f64, off, src), do: "be_put_f64(&f.payload[#{off}], #{src});"
  defp scalar_put(:u8, off, src), do: "f.payload[#{off}] = #{src};"
  defp scalar_put(:u16, off, src), do: "be_put_u16(&f.payload[#{off}], #{src});"
  defp scalar_put(:u32, off, src), do: "be_put_u32(&f.payload[#{off}], #{src});"
  defp scalar_put(:u64, off, src), do: "be_put_u64(&f.payload[#{off}], #{src});"
  defp scalar_put(:bool, off, src), do: "f.payload[#{off}] = (#{src}) ? 1 : 0;"

  # Byte offset of the field at `idx` in a layout (sum of preceding widths).
  defp field_offset(layout, idx) do
    layout
    |> Enum.take(idx)
    |> Enum.reduce(0, fn {_f, wt}, acc -> acc + Contract.Layouts.width(wt) end)
  end

  # Total packed payload length for a layout (the body's payload byte count).
  defp payload_min_len(layout) do
    Enum.reduce(layout, 0, fn {_f, wt}, acc -> acc + Contract.Layouts.width(wt) end)
  end

  # --- emit: parity vectors ---

  @doc """
  Render the parity-vector fixture by running the **real** encoder over a
  representative value per port. The bytes are correct by construction (§03/§06).
  """
  @spec emit_parity([Contract.ir_row()]) :: String.t()
  def emit_parity(ir) do
    rows =
      ir
      |> Enum.map_join(",\n", fn row ->
        value = sample_value(row.type)

        body =
          Codec.encode_body(
            row.node,
            row.port_id,
            @sample_seq,
            @sample_t_dev,
            row.type,
            value,
            row.stamped
          )

        crc = CRC16.crc(body)

        "  %{\n" <>
          "    hub: #{inspect(row.hub)}, port: #{inspect(row.port)}, type: #{inspect(row.type)},\n" <>
          "    node: 0x#{hex2(row.node)}, port_id: 0x#{hex2(row.port_id)},\n" <>
          "    seq: #{@sample_seq}, t_dev: #{@sample_t_dev}, stamped: #{row.stamped},\n" <>
          "    value: #{inspect(value, custom_options: [sort_maps: true])},\n" <>
          "    body: #{inspect(body, base: :hex, limit: :infinity)},\n" <>
          "    crc: 0x#{hex4(crc)}\n" <>
          "  }"
      end)

    finalize([
      "# GENERATED parity vectors (§03/§06) — the cross-language witness.",
      "# Each row: a value, its exact CRC-covered body bytes, and the CRC, computed",
      "# by running the real encoder. Asserted by the Elixir suite AND the",
      "# host-compiled C harness. Regenerate with `mix wire.gen`; a non-empty diff",
      "# means a contract moved and the bytes moved with it. Hand-editing a row is",
      "# the tell.",
      "[",
      rows,
      "]"
    ])
  end

  @doc """
  Render the parity vectors as a C header of byte arrays, so the host-compiled C
  harness can assert the **same** rows with no Elixir dependency (§03/§06). Each
  row is `{node, port, seq, t_dev, body bytes, crc}` — the C side rebuilds the
  body from the value via its own codec and must match these bytes exactly.
  """
  @spec emit_parity_c([Contract.ir_row()]) :: String.t()
  def emit_parity_c(ir) do
    rows =
      ir
      |> Enum.map(fn row ->
        value = sample_value(row.type)

        body =
          Codec.encode_body(
            row.node,
            row.port_id,
            @sample_seq,
            @sample_t_dev,
            row.type,
            value,
            row.stamped
          )

        crc = CRC16.crc(body)
        {row, value, body, crc}
      end)

    entries =
      rows
      |> Enum.with_index()
      |> Enum.map_join(",\n", fn {{row, value, body, crc}, idx} ->
        bytes = body |> :binary.bin_to_list() |> Enum.map_join(", ", &"0x#{hex2(&1)}")
        payload = build_payload_call(row, value)

        ~s(  { #{inspect(to_string(row.hub))}, #{inspect(to_string(row.port))},\n) <>
          ~s(    0x#{hex2(row.node)}, 0x#{hex2(row.port_id)}, #{@sample_seq}, #{@sample_t_dev}ULL, #{if row.stamped, do: "true", else: "false"},\n) <>
          ~s(    { #{bytes} }, #{byte_size(body)}, 0x#{hex4(crc)}, #{payload} } /* #{idx} */)
      end)

    finalize([
      header_banner("parity_vectors.h", "byte-exact rows for the C parity harness (§03)"),
      "#ifndef BB_MCUHUB_PARITY_VECTORS_H",
      "#define BB_MCUHUB_PARITY_VECTORS_H",
      "#include <stdint.h>",
      "#include <stdbool.h>",
      "#include \"frame.h\"",
      "",
      "/* The representative value per row, as a Frame payload built by the test's",
      "   per-type packers — see firmware/test/test_parity.c. */",
      "typedef void (*pack_fn)(Frame *f);",
      "",
      "typedef struct {",
      "  const char *hub;",
      "  const char *port;",
      "  uint8_t node;",
      "  uint8_t port_id;",
      "  uint16_t seq;",
      "  uint64_t t_dev;",
      "  bool stamped;     /* does this port carry t_dev? (§04) */",
      "  uint8_t body[#{max_body_len(rows)}];",
      "  size_t body_len;",
      "  uint16_t crc;",
      "  pack_fn pack;",
      "} ParityVector;",
      "",
      "static const ParityVector PARITY_VECTORS[] = {",
      entries,
      "};",
      "#define N_PARITY_VECTORS (sizeof(PARITY_VECTORS) / sizeof(PARITY_VECTORS[0]))",
      "",
      "#endif /* BB_MCUHUB_PARITY_VECTORS_H */"
    ])
  end

  # A reference to the test-side packer that builds this row's payload into a
  # Frame. The test defines one packer per (hub, port).
  defp build_payload_call(row, _value), do: "pack_#{row.hub}_#{row.port}"

  defp max_body_len(rows),
    do: rows |> Enum.map(fn {_r, _v, body, _c} -> byte_size(body) end) |> Enum.max()

  # --- shared helpers ---

  @doc """
  Deterministic SHA over the whole IR — the drift hash.

  Hashed over each row's plain field map (the `__struct__` tag stripped), so the
  hash reflects the SEMANTIC contract, not the Elixir representation — promoting
  the IR row from a map to a typed struct (candidate 1) leaves the hash unchanged.
  """
  @spec contract_sha([Contract.ir_row()]) :: String.t()
  def contract_sha(ir) do
    semantic = Enum.map(ir, &Map.delete(Map.from_struct(&1), :__struct__))
    canonical = inspect(semantic, custom_options: [sort_maps: true], limit: :infinity)
    :crypto.hash(:sha256, canonical) |> Base.encode16(case: :lower) |> binary_part(0, 16)
  end

  @doc "A deterministic representative value map for a type's parity row."
  @spec sample_value(atom()) :: map()
  def sample_value(type) do
    ValueType.resolve(type).layout()
    |> Enum.with_index()
    |> Map.new(fn {{field, wt}, idx} -> {field, sample_scalar(wt, idx)} end)
  end

  # Distinguishable, exactly-representable values per field.
  defp sample_scalar(:f32, 0), do: 1.0
  defp sample_scalar(:f32, idx), do: idx * 0.5
  defp sample_scalar(:f64, idx), do: idx * 0.25
  defp sample_scalar(:u8, idx), do: rem(idx + 1, 256)
  defp sample_scalar(:u16, idx), do: rem((idx + 1) * 7, 0x10000)
  defp sample_scalar(:u32, idx), do: (idx + 1) * 13
  defp sample_scalar(:u64, idx), do: (idx + 1) * 17
  defp sample_scalar(:bool, idx), do: rem(idx, 2) == 0

  defp actuators(ir), do: Enum.filter(ir, &(&1.dir == :in and &1.has_safe_action == true))

  # The node id of the ROOT hub (ADR-0006). Root-ness is DECLARED — the hub with
  # parent: :host is root — so the firmware no longer hand-sets a -DROOT_HUB flag:
  # the chassis derives IS_ROOT = (MY_NODE == ROOT_NODE) from this generated define.
  # A single-hub robot still declares parent: :host, so ROOT_NODE is ALWAYS emitted.
  defp root_node_define(ir) do
    case Enum.find(hub_models(ir), &(&1.parent == @host)) do
      nil ->
        "/* no root declared — topology verifier will have already failed */"

      root ->
        "#define ROOT_NODE 0x#{hex2(root.node)}"
    end
  end

  # Per-downlink transport defines for the ROOT hub (ADR-0006). Transport is a
  # property of a LINK: each root downlink k emits LINK<k>_TRANSPORT_UART = 1
  # (UART) or 0 (CAN), sourced from the child's DECLARED uplink — never inferred.
  # For the single-downlink example this is just LINK1_TRANSPORT_UART, equivalent
  # to the old robot-wide flag but per-link. A robot with no downlinks at the root
  # (a single-hub robot) emits none.
  defp root_link_transport_defines(ir) do
    case Enum.find(hub_models(ir), &(&1.parent == @host)) do
      nil ->
        "/* no root declared — topology verifier will have already failed */"

      root ->
        case downlinks(ir, root.name) do
          [] ->
            "/* the root has no downlinks (single-hub robot) — no per-link transport */"

          links ->
            Enum.map_join(links, "\n", fn link ->
              uart = if link.transport == :uart, do: 1, else: 0
              "#define LINK#{link.idx}_TRANSPORT_UART #{uart}"
            end)
        end
    end
  end

  defp tick_name(%{dir: :out, hub: _hub, port: port}), do: "#{port}_sample_tick"
  defp tick_name(%{dir: :in, port: port}), do: "#{port}_cmd_tick"

  defp c_type(:f32), do: "float"
  defp c_type(:f64), do: "double"
  defp c_type(:u8), do: "uint8_t"
  defp c_type(:u16), do: "uint16_t"
  defp c_type(:u32), do: "uint32_t"
  defp c_type(:u64), do: "uint64_t"
  defp c_type(:bool), do: "bool"

  # The C struct name for a value-type ref. A stock atom (`:imu`) camelizes
  # directly (`Imu`). A consumer's own value-type is named by MODULE
  # (`MyApp.ValueType.Scalar`); its dotted string is not a valid C identifier, so
  # take the module's LAST segment (`Scalar`) — the value-type is a standalone
  # unit, so its short name uniquely names its struct.
  defp c_struct_name(type) do
    str = Atom.to_string(type)

    if String.starts_with?(str, "Elixir.") do
      type |> Module.split() |> List.last()
    else
      Macro.camelize(str)
    end
  end

  defp header_banner(file, what) do
    "/* GENERATED by BBMCUHub.Gen.WireGen — do not edit. #{file}\n   #{what} */"
  end

  defp finalize(lines), do: (lines |> List.flatten() |> Enum.join("\n")) <> "\n"

  defp up(atom), do: atom |> Atom.to_string() |> String.upcase()
  defp hex2(n), do: n |> Integer.to_string(16) |> String.upcase() |> String.pad_leading(2, "0")
  defp hex4(n), do: n |> Integer.to_string(16) |> String.upcase() |> String.pad_leading(4, "0")
end
