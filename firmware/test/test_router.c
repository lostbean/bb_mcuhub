/* Host-compiled test of the router (§04/§08, ADR-0006, ADR-0011):
 * meaning-blind, DIRECTION-AWARE flat-NODE dispatch onto a per-hub-local LINK
 * INDEX, and the relay invariant — a forwarded frame's body (incl. seq/t_dev)
 * is unchanged. NODE names the hub end of a host↔hub conversation: a
 * descending frame (arrival link 0, from the parent) carries a DESTINATION
 * and is dispatched by the declared route table; an ascending frame (arrival
 * link >= 1, from a downlink) carries a SOURCE and always forwards up — never
 * back down the link it arrived on (issue #9), and never down to a sibling.
 * Pure logic, no hardware. */
#include "frame.h"
#include "router.h"
#include <stdio.h>
#include <string.h>

static int g_fail = 0;
#define CHECK(cond, msg)                                                       \
  do {                                                                         \
    if (cond) {                                                                \
      printf("  ok   %s\n", msg);                                              \
    } else {                                                                   \
      g_fail++;                                                                \
      printf("  FAIL %s\n", msg);                                              \
    }                                                                          \
  } while (0)

/* capture which sink fired, on what link, and on what frame */
static int g_local, g_sent;
static uint8_t g_last_link;
static Frame g_last;

static void on_local(const Frame *f, void *ctx) {
  (void)ctx;
  g_local++;
  g_last = *f;
}
static void on_send(uint8_t link, const Frame *f, void *ctx) {
  (void)ctx;
  g_sent++;
  g_last_link = link;
  g_last = *f;
}

static void reset(void) {
  g_local = g_sent = 0;
  g_last_link = 0xEE;
}

static Frame make(uint8_t node, uint8_t port, uint16_t seq, uint64_t t) {
  Frame f;
  memset(&f, 0, sizeof(f));
  f.node = node;
  f.port = port;
  f.seq = seq;
  f.stamped =
      true; /* the router is meaning-blind; it forwards the struct as-is */
  f.t_dev = t;
  f.payload[0] = 0xAB;
  f.payload[1] = 0xCD;
  f.payload_len = 2;
  return f;
}

int main(void) {
  printf("== bb_mcuhub C router harness ==\n");

  /* A root hub at NODE 0x02: link 0 = up (host), link 1 = down to the child
   * 0x05. MY_NODE → LINK_LOCAL_IDX; everything else defaults to 0 (up). */
  Router r;
  memset(&r, 0, sizeof(r));
  r.my_node = 0x02;
  for (int i = 0; i < 256; i++)
    r.route_table[i] = 0; /* default: link 0 — "not in my subtree" */
  r.route_table[0x02] = LINK_LOCAL_IDX;
  r.route_table[0x05] = 1; /* the child hub reached over downlink 1 */

  RouterSinks sinks = {on_local, on_send, NULL};

  /* --- DESCENDING (arrival link 0, from the host): NODE is a destination ---
   */

  /* a frame addressed to THIS hub → delivered locally */
  reset();
  Frame f_local = make(0x02, 0x10, 7, 1234);
  router_route(&r, &f_local, 0, &sinks);
  CHECK(g_local == 1 && g_sent == 0,
        "descend: frame for my_node → deliver_local");

  /* a frame for a child node → forwarded on the child's downlink, body verbatim
   */
  reset();
  Frame f_child = make(0x05, 0x28, 99, 555000);
  router_route(&r, &f_child, 0, &sinks);
  CHECK(g_sent == 1 && g_local == 0 && g_last_link == 1,
        "descend: frame for a child node → send_on_link(1) (its downlink)");
  CHECK(g_last.node == 0x05 && g_last.port == 0x28 && g_last.seq == 99 &&
            g_last.t_dev == 555000,
        "forwarded frame: seq/t_dev/node/port VERBATIM (relay invariant §04)");
  CHECK(g_last.payload_len == 2 && g_last.payload[0] == 0xAB &&
            g_last.payload[1] == 0xCD,
        "forwarded frame: payload unchanged");

  /* a descending frame for a node NOT in this subtree (table default 0 = the
   * arrival link) → DROPPED, never reflected back at the parent (ADR-0011) */
  reset();
  Frame f_unknown = make(0x09, 0x11, 1, 1);
  router_route(&r, &f_unknown, 0, &sinks);
  CHECK(g_sent == 0 && g_local == 0,
        "descend: a frame for a node not in my subtree is DROPPED (split "
        "horizon — never reflected up)");

  /* --- ASCENDING (arrival link >= 1, from a downlink): NODE is a source --- */

  /* issue #9 — the two-hop upstream path. A leaf's sensor frame carries
   * f->node = ITS OWN node (a SOURCE, not a destination). When it arrives at
   * the root from downlink 1, it must be forwarded UP (link 0) toward the
   * host — NOT bounced back down route_table[0x05] = 1, the link it just
   * came from. */
  reset();
  Frame f_leaf_sense = make(0x05, 0x1C, 21, 700);
  router_route(&r, &f_leaf_sense, 1, &sinks);
  CHECK(g_sent == 1 && g_local == 0 && g_last_link == 0,
        "issue #9: a leaf's up-frame arriving from downlink 1 → "
        "send_on_link(0) (up), never reflected back down");
  CHECK(g_last.node == 0x05 && g_last.port == 0x1C && g_last.seq == 21 &&
            g_last.t_dev == 700,
        "ascending frame: seq/t_dev/node/port VERBATIM (relay invariant §04)");

  /* a downlink frame claiming MY node is still ascending traffic — a child can
   * never command its parent's local ports (one writer: the host, ADR-0007) */
  reset();
  Frame f_spoof_me = make(0x02, 0x10, 5, 42);
  router_route(&r, &f_spoof_me, 1, &sinks);
  CHECK(g_sent == 1 && g_local == 0 && g_last_link == 0,
        "ascend: a downlink frame with node == my_node goes UP, never "
        "deliver_local (a child cannot command its parent)");

  /* --- A LEAF hub: everything arrives from the parent (its only link, 0) ---
   */
  Router leaf;
  memset(&leaf, 0, sizeof(leaf));
  leaf.my_node = 0x05;
  for (int i = 0; i < 256; i++)
    leaf.route_table[i] = 0; /* default: "not in my subtree" */
  leaf.route_table[0x05] = LINK_LOCAL_IDX;
  RouterSinks leaf_sinks = {on_local, on_send, NULL};

  reset();
  Frame f_leaf = make(0x05, 0x28, 3, 9);
  router_route(&leaf, &f_leaf, 0, &leaf_sinks);
  CHECK(g_local == 1 && g_sent == 0,
        "leaf: a command for it is delivered local");

  /* a leaf has no subtree below it: a descending frame for any OTHER node is
   * a misroute → DROPPED, never bounced back up at the parent (ADR-0011) */
  reset();
  Frame f_leaf_other = make(0x03, 0x10, 4, 9);
  router_route(&leaf, &f_leaf_other, 0, &leaf_sinks);
  CHECK(g_sent == 0 && g_local == 0,
        "leaf: a descending frame for another node is DROPPED (nothing below "
        "me; never reflected)");

  /* --- A BRANCH with TWO downlinks (ADR-0006): link 1 → 0x03, link 2 → 0x05.
   * Generic node→link routing down; direction-aware ascent up. --- */
  Router branch;
  memset(&branch, 0, sizeof(branch));
  branch.my_node = 0x02;
  for (int i = 0; i < 256; i++)
    branch.route_table[i] = 0; /* default: "not in my subtree" */
  branch.route_table[0x02] = LINK_LOCAL_IDX;
  branch.route_table[0x03] = 1; /* downlink 1 */
  branch.route_table[0x05] = 2; /* downlink 2 */
  RouterSinks branch_sinks = {on_local, on_send, NULL};

  reset();
  Frame f_d1 = make(0x03, 0x20, 11, 100);
  router_route(&branch, &f_d1, 0, &branch_sinks);
  CHECK(g_sent == 1 && g_local == 0 && g_last_link == 1,
        "multi-downlink descend: node 0x03 → send_on_link(1)");

  reset();
  Frame f_d2 = make(0x05, 0x21, 12, 200);
  router_route(&branch, &f_d2, 0, &branch_sinks);
  CHECK(
      g_sent == 1 && g_local == 0 && g_last_link == 2,
      "multi-downlink descend: node 0x05 → send_on_link(2) (a DIFFERENT link)");

  reset();
  Frame f_dx = make(0x09, 0x22, 13, 300);
  router_route(&branch, &f_dx, 0, &branch_sinks);
  CHECK(g_sent == 0 && g_local == 0,
        "multi-downlink descend: an unknown node is DROPPED (split horizon)");

  /* each child's up-frame ascends, regardless of which downlink it uses */
  reset();
  Frame f_up1 = make(0x03, 0x20, 14, 400);
  router_route(&branch, &f_up1, 1, &branch_sinks);
  CHECK(g_sent == 1 && g_local == 0 && g_last_link == 0,
        "multi-downlink ascend: child 0x03's up-frame from downlink 1 → up");

  /* a downlink frame addressed to a SIBLING must ascend, never hop across:
   * only the parent/host side originates descending traffic (ADR-0007/0011) */
  reset();
  Frame f_sibling = make(0x05, 0x21, 15, 500);
  router_route(&branch, &f_sibling, 1, &branch_sinks);
  CHECK(g_sent == 1 && g_local == 0 && g_last_link == 0,
        "ascend: a downlink-1 frame naming downlink-2's node goes UP — a "
        "child can never inject traffic toward a sibling");

  /* a deeper descendant (grandchild, unknown to no one — table maps it to its
   * downlink) still ascends from that downlink */
  reset();
  Frame f_grand = make(0x09, 0x23, 16, 600);
  router_route(&branch, &f_grand, 2, &branch_sinks);
  CHECK(g_sent == 1 && g_local == 0 && g_last_link == 0,
        "ascend: ANY frame from a downlink forwards up (a grandchild's "
        "source node included)");

  if (g_fail == 0) {
    printf("\nALL C ROUTER CHECKS PASSED\n");
    return 0;
  }
  printf("\n%d C ROUTER CHECK(S) FAILED\n", g_fail);
  return 1;
}
