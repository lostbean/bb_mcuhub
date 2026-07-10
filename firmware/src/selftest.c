/* The link self-test core — see selftest.h; the boot-mode design and this
 * pinned interface are ADR-0012. Pure logic, host-tested; wrap arithmetic on
 * now_ms is (now - then) >= window, so millis() rollover is harmless. */
#include "selftest.h"
#include <string.h>

/* Test frames start here; frame k carries seq SELFTEST_SEQ_ORIGIN + k. */
#define SELFTEST_SEQ_ORIGIN 1u

/* Representative payload width — well under FRAME_MAX_PAYLOAD, wide enough
 * that a torn or shifted echo cannot collide with the expected pattern. */
#define SELFTEST_PAYLOAD_LEN 16u

/* The seq-derived pattern: every byte depends on BOTH the seq and its own
 * position, so a torn echo, a shifted byte, and an echo of the wrong frame
 * all fail the exact match. */
static void payload_for_seq(uint16_t seq, uint8_t *p) {
  for (size_t i = 0; i < SELFTEST_PAYLOAD_LEN; i++)
    p[i] =
        (uint8_t)((seq >> 8) ^ (uint8_t)(seq * 7u) ^ (uint8_t)(i * 29u + 13u));
}

void selftest_init(Selftest *s, uint8_t my_node, uint8_t link,
                   uint16_t n_frames, uint32_t gap_ms, uint32_t settle_ms) {
  memset(s, 0, sizeof(*s));
  s->my_node = my_node;
  s->link = link;
  s->n_frames = n_frames;
  s->gap_ms = gap_ms;
  s->settle_ms = settle_ms;
}

bool selftest_next(Selftest *s, uint32_t now_ms, Frame *out) {
  s->last_seen_ms = now_ms; /* receives get stamped with this reading */
  if (s->sent >= s->n_frames)
    return false; /* never past n_frames */
  if (s->sent_any && (uint32_t)(now_ms - s->last_send_ms) < s->gap_ms)
    return false; /* never more often than gap_ms */

  uint16_t seq = (uint16_t)(SELFTEST_SEQ_ORIGIN + s->sent);
  out->node = s->my_node;
  out->port = SELFTEST_PORT;
  out->seq = seq;
  out->stamped = false; /* no t_dev on a test frame */
  out->t_dev = 0;
  out->payload_len = SELFTEST_PAYLOAD_LEN;
  payload_for_seq(seq, out->payload);

  s->sent++;
  s->sent_any = true;
  s->last_send_ms = now_ms;
  s->last_activity_ms = now_ms;
  return true;
}

void selftest_on_body(Selftest *s, const uint8_t *body, size_t len) {
  /* Receive activity — stamped with the latest clock reading (the pinned
   * interface carries no time here; the runner polls selftest_next every
   * loop, so the reading is at most one iteration old). */
  s->last_activity_ms = s->last_seen_ms;

  /* Echoes come back in send order on a point-to-point loopback, so one
   * cursor suffices for exact duplicate detection: a matched seq advances it,
   * a seq behind it is a duplicate. A reordered genuine echo would land
   * behind the cursor and be ignored — an undercount, i.e. it fails toward
   * FAIL, never toward a false PASS. */
  Frame f;
  bool match = frame_decode_body(body, len, /* stamped: */ false, &f) &&
               f.node == s->my_node && f.port == SELFTEST_PORT &&
               f.payload_len == SELFTEST_PAYLOAD_LEN &&
               (uint16_t)(f.seq - SELFTEST_SEQ_ORIGIN) < s->sent;
  if (match) {
    uint8_t want[SELFTEST_PAYLOAD_LEN];
    payload_for_seq(f.seq, want);
    match = memcmp(f.payload, want, SELFTEST_PAYLOAD_LEN) == 0;
  }
  if (!match) {
    s->mismatched++; /* decodes-but-wrong and does-not-decode alike */
    return;
  }
  uint16_t idx = (uint16_t)(f.seq - SELFTEST_SEQ_ORIGIN);
  if (idx < s->match_cursor)
    return; /* duplicate of an already-matched seq: neither counter moves */
  s->echoed++;
  s->match_cursor = (uint16_t)(idx + 1);
}

bool selftest_done(const Selftest *s, uint32_t now_ms) {
  if (s->sent < s->n_frames)
    return false;
  if ((uint32_t)s->echoed + s->mismatched >= s->sent)
    return true; /* every sent frame accounted for — no settle wait */
  return (uint32_t)(now_ms - s->last_activity_ms) >= s->settle_ms;
}

SelftestStats selftest_stats(const Selftest *s) {
  SelftestStats st = {s->sent, s->echoed, s->mismatched};
  return st;
}
