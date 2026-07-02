/* Meaning-blind, direction-aware flat-NODE routing (§03/§04/§08, ADR-0006,
 * ADR-0011).
 *
 * NODE is a whole-tree-unique id, never a path — and it names the HUB END of
 * a host↔hub conversation, not "the destination": commands descend carrying
 * the destination hub's node; sense/status ascend carrying the SOURCE hub's
 * node. One field, two directions — so routing needs the frame's ARRIVAL
 * LINK to know which meaning applies (ADR-0011):
 *
 *   - arrived on a DOWNLINK (index >= 1): ascending, host-bound. Forward UP
 *     (link 0), unconditionally — the dest table does not apply to a source
 *     node, and nothing a child injects may ever be routed down (ADR-0007).
 *   - arrived on the UP-LINK (index 0, from the parent/host): descending.
 *     NODE is a destination — deliver local or forward onto the declared
 *     downlink via route_table[node]. A table answer equal to the arrival
 *     link (the default 0) means "not in my subtree": DROP, never reflect a
 *     frame back out the link it arrived on.
 *
 * Topology is DECLARED, not inferred (ADR-0006): the generated route table
 * maps each NODE to a per-hub-local LINK INDEX (link 0 is always the up-link
 * toward the parent/host; downlinks are 1..N). A shared CAN bus is ONE link;
 * each point-to-point UART child its own link. The router does NOT decode
 * the payload and does NOT touch seq/t_dev.
 *
 * Forwarding is strict FIFO, in arrival order, re-framing for the destination
 * link's transport and re-CRCing the SAME body — never reorder/hold/dedup. That
 * in-order guarantee is what makes the floor's seq!=last_seq test sound (§04).
 */
#ifndef BB_MCUHUB_ROUTER_H
#define BB_MCUHUB_ROUTER_H

#include "frame.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* The route-table sentinel for "this node is local to me" — delivered to a port
 * on THIS hub, never sent on a link. A real link index is 0..N (0 = up-link).
 */
#define LINK_LOCAL_IDX 0xFF

/* The arrival_link value meaning "this frame came DOWN from the parent/host"
 * — the up-link, index 0. Any other arrival is a downlink (ADR-0011). */
#define ROUTER_ARRIVAL_UP 0

/* A hub's view of the tree: which LOCAL LINK INDEX reaches each NODE. Generated
 * from the DECLARED topology (ADR-0006). Link 0 is the up-link (toward the
 * parent/host); downlinks are 1..N. MY_NODE → LINK_LOCAL_IDX; an ancestor or an
 * unknown node → 0 (the up-link); a descendant via downlink k → k. The table
 * only ever answers for DESCENDING frames (ADR-0011). */
typedef struct {
  uint8_t my_node;
  /* route_table[node] → a LOCAL LINK INDEX, or LINK_LOCAL_IDX for self/local */
  uint8_t route_table[256];
} Router;

/* Callbacks the firmware supplies to actually move bytes on each link. The link
 * index is per-hub-local (0 = up-link); send_on_link maps it to that hub's
 * peripheral. A board realizes the links it physically has and stubs the rest
 * (ADR-0006). */
typedef struct {
  void (*deliver_local)(const Frame *f,
                        void *ctx); /* an endpoint port on THIS hub */
  void (*send_on_link)(uint8_t link, const Frame *f,
                       void *ctx); /* a local link index → bytes on that link */
  void *ctx;
} RouterSinks;

/* Route one frame, in order, meaning-blind. `arrival_link` is the LOCAL LINK
 * INDEX the frame arrived on (0 = from the parent/host; >= 1 = from that
 * downlink) — it decides whether NODE is a source (ascend) or a destination
 * (table). seq/t_dev untouched. (ADR-0011) */
void router_route(const Router *r, const Frame *f, uint8_t arrival_link,
                  const RouterSinks *sinks);

#ifdef __cplusplus
}
#endif

#endif /* BB_MCUHUB_ROUTER_H */
