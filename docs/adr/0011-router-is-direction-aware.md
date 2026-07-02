# The router is direction-aware: a frame from a downlink ascends; the route table serves only descending traffic

_Amends ADR-0006 (whose `node → link` table stands, but only for one direction).
Fixes issue #9, found on real hardware by the `segby_v1` two-board bring-up._

A hub routes a frame by **the link it arrived on**, not by its NODE alone. A
frame arriving on a **downlink** (a child link, index ≥ 1) is ascending
host-bound traffic and is forwarded **up** (link 0), unconditionally — the
route table is never consulted. A frame arriving on the **up-link** (index 0,
from the parent/host) is descending traffic, and only there does ADR-0006's
`route_table[node] → link` dispatch apply: local delivery for `my_node`, the
declared downlink for a descendant, and a **drop** (never a reflection) for a
node not in this hub's subtree.

This is recorded because it corrects a wrong model at the heart of the relay
path — the assumption that a frame's NODE is always a _destination_ — that
made every multi-hop tree silently unable to report upward, and because the
correction adds a new input (the arrival link) to a safety-adjacent seam.

## The problem

The wire format has **one NODE field**, and it names _the hub end of a
host↔hub conversation_ — not "the destination":

- a **command** descends from the host carrying the destination hub's node;
- a **sense/status** frame ascends toward the host carrying the _source_
  hub's node. The host end of the conversation is implicit — no frame ever
  names it.

ADR-0006's router ignored this duality. `router_route` looked up
`route_table[f->node]` for every frame, treating NODE as a destination
always. Both of a root's RX paths (the host UART and the backplane) fed the
same `hub_on_body(body, len)` with **no arrival-link information**, so the
router _could not_ have told the directions apart even if it had wanted to.

The consequence, observed on real ESP32 hardware (issue #9): a leaf's sensor
frame carries `f->node = <the leaf>`; the root's generated table maps that
node to the leaf's own **downlink**; the root therefore reflected every
leaf→host frame straight back down the link it arrived on. Host→leaf commands
worked (NODE really is a destination going down), so the failure was
perfectly one-sided and **silent** — no CRC errors, no drops, flat wire
stats; the leaf was simply invisible to the host. Any leaf with a sense or
status port behind a relay was unusable, which is to say: the multi-hop
topology ADR-0006 exists to enable did not work.

It survived every test layer because each one shared the same wrong model or
bypassed the seam: the C router harness only ever routed destination-addressed
frames; the VirtualHub e2e NIF compiled the floor and the wire path but not
`router.c`; the MuJoCo sim (ADR-0008) replaces the transport entirely, so no
C router runs at all.

## The decision

**Thread the arrival link through the RX path, and make direction the first
routing decision.**

- `link_esp32.cpp` tags every verified body with the LOCAL LINK INDEX it
  arrived on: the root's host UART is arrival 0; the one realized backplane
  is arrival 1 on a root and arrival 0 on a leaf (`IS_ROOT` folds it at
  preprocess time). The generated `hub_on_body(arrival_link, body, len)`
  passes it to `router_route(&router, &frame, arrival_link, &sinks)`.
- **Arrival ≥ 1 (a downlink): ascend.** Forward on link 0, unconditionally.
  No table lookup, no local delivery — a child can never command its
  parent's ports and can never inject traffic toward a sibling; everything a
  subtree emits can only travel toward the host. This preserves the
  one-writer discipline (ADR-0007) at the wire level: the parent side is the
  sole source of descending traffic.
- **Arrival 0 (the parent/host): descend by the table.** `my_node` /
  `LINK_LOCAL_IDX` delivers locally; a descendant's node forwards onto its
  declared downlink; the table default (0, "not in my subtree") **drops** the
  frame — split horizon: a frame is never sent back out the link it arrived
  on, in either direction.
- The route table's _contents_ are unchanged — ADR-0006's generation stands;
  the table simply answers only for descending frames now. The relay
  invariants are untouched: strict per-link FIFO, the body re-framed and
  re-CRC'd verbatim, `seq`/`t_dev` never modified. The router still never
  reads the payload — the arrival link is link-layer fact, not frame
  meaning, so "meaning-blind" holds.

## Considered options

- **Split horizon with an up-fallback (keep the table primary for all
  arrivals; when `route_table[node] == arrival_link`, send up instead).**
  Equivalent for every host↔hub flow, and it would additionally route a
  downlink-arrived frame addressed to a _sibling's_ node down that sibling's
  link. Rejected: hub↔hub traffic is not in the model (all conversations are
  host↔hub), and honoring it would let a malfunctioning or spoofing child
  inject descending traffic toward a sibling actuator — exactly the
  capability the one-writer discipline exists to exclude. More conditional
  logic to enable a hazard.
- **Real source/destination headers in the wire format.** The general
  networking answer. Rejected: it grows every frame, touches the codec on
  both strata, the floor, the parity vectors, and every harness — to buy
  point-to-point routing the host↔hub model deliberately does not have.
- **Do nothing in the library; require single-hop robots.** Rejected: it
  retracts ADR-0006's headline capability, and the segby hardware is already
  a two-hop tree.

## Consequences

- **Multi-hop trees actually work.** The two-hop upstream path (leaf sense →
  root → host) — broken since ADR-0006 — routes correctly at any depth: at
  every intermediate hub an ascending frame arrives on a downlink and keeps
  ascending.
- **The RX seam carries one more fact.** `link_set_on_body` /
  `hub_on_body` gain the `arrival_link` parameter; the generator emits the
  new signature; both generated trees (the fixture robot and `segby_v1`)
  regenerate. Consumers of the _host-side_ library see no change — the wire
  format and the host stack are untouched.
- **Anomalies die instead of wandering.** A descending frame for an unknown
  node used to reflect back up at every hub; now it drops at the first hub
  whose subtree excludes it. (A drop counter can ride the planned `link_stats`
  port — issue #8 — for observability.)
- **The coverage gap that hid this is closed at two layers.** The C router
  harness now routes by (arrival, node) — including the issue-#9 regression,
  the sibling-injection case, and the parent-spoof case — and the VirtualHub
  NIF compiles `router.c`, so a host-level e2e drives a leaf's frames through
  the real C router in both directions (`two_hop_routing_e2e_test.exs`). The
  MuJoCo sim still bypasses the router by design; the VirtualHub e2e is the
  seam that covers it.
- **What a child claims is still trusted.** A leaf that lies about its NODE
  in ascending traffic reaches the host under the lie (node identity remains
  trust-on-first-use, as before this ADR) — but it can no longer _steer_
  frames anywhere except up.
- **The traffic model becomes a named invariant.** "Every conversation is
  host↔hub; hubs never converse with each other" is promoted from an implicit
  assumption to a first-class design invariant (`docs/hub-design.html` ·
  Invariants; `CONTEXT.md` · Host↔hub conversations). The direction rule is
  its enforcement: the hub tree is a tree at the transport level but a star
  at the conversation level. Any future hub↔hub path must revisit this ADR
  and bring its own authentication story.
