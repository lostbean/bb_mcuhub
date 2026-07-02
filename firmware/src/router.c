#include "router.h"

void router_route(const Router *r, const Frame *f, uint8_t arrival_link,
                  const RouterSinks *sinks) {
  /* Direction rule (ADR-0011): a frame arriving on a DOWNLINK is ascending
   * host-bound traffic — its NODE names the SOURCE hub, so the destination
   * table does not apply. Forward it up (link 0), unconditionally. This is
   * also the safety property: nothing a child injects can ever be routed
   * DOWN (to this hub's ports or a sibling's) — the parent/host side stays
   * the only source of descending traffic (ADR-0007).
   */
  if (arrival_link != ROUTER_ARRIVAL_UP) {
    if (sinks->send_on_link)
      sinks->send_on_link(0, f, sinks->ctx);
    return;
  }

  /* From the parent (link 0): descending traffic — NODE is a DESTINATION. */
  if (f->node == r->my_node) {
    if (sinks->deliver_local)
      sinks->deliver_local(f, sinks->ctx);
    return;
  }

  uint8_t link = r->route_table[f->node];

  if (link == LINK_LOCAL_IDX) {
    /* a node mapped local (e.g. a multi-port hub addressing itself) */
    if (sinks->deliver_local)
      sinks->deliver_local(f, sinks->ctx);
    return;
  }

  if (link == ROUTER_ARRIVAL_UP) {
    /* Split horizon (ADR-0011): the table answers "back where it came from" —
     * a descending frame for a node NOT in this subtree (the table default).
     * Never reflect a frame out its arrival link: drop it. */
    return;
  }

  /* meaning-blind: forward verbatim onto the local link that reaches this node
   * (downlinks 1..N). seq/t_dev are never touched. An unrealized link is a
   * no-op stub on the board (ADR-0006).
   */
  if (sinks->send_on_link)
    sinks->send_on_link(link, f, sinks->ctx);
}
