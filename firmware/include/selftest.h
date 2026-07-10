/* The link self-test core (issue #8) — a boot-time loopback session in the
 * same shape as the floor: explicit time in, a plain caller-allocated state
 * struct, no hardware, no heap. ADR-0012 pins this interface and the boot-mode
 * design around it.
 *
 * One session tests ONE link: emit `n_frames` known frames paced by `gap_ms`,
 * feed every decoded body back in, match against expectation, and terminate
 * through a `settle_ms` silence timeout so a dead link can never hang the
 * session. The caller owns the loop:
 *
 *   selftest_next() → frame_encode_body → transport_encode → the link;
 *   link bytes → transport_decoder_feed → its body callback →
 *   selftest_on_body(); poll selftest_done(); read selftest_stats().
 *
 * The core deliberately does NOT count CRC failures: a corrupt frame dies
 * inside the transport decoder, which already counts it
 * (TransportDecoder.rx_drop) — the runner reports that counter's delta. */
#ifndef BB_MCUHUB_SELFTEST_H
#define BB_MCUHUB_SELFTEST_H

#include "frame.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* The self-test port, reserved by convention (not enforced by the generator):
 * self-test frames never coexist with a running router or floor on the same
 * board, and at a live host a mismatched payload fails the value-type decode.
 */
#define SELFTEST_PORT 0xFF

typedef struct {
  uint16_t sent, echoed, mismatched;
} SelftestStats;

typedef struct {
  /* -- session parameters (fixed at init) -- */
  uint8_t my_node;
  uint8_t link; /* local link index under test — the caller sends on it */
  uint16_t n_frames;
  uint32_t gap_ms;    /* minimum spacing between two emitted frames */
  uint32_t settle_ms; /* silence window that ends a session with losses */
  /* -- session state -- */
  uint16_t sent;
  uint16_t echoed;
  uint16_t mismatched;
  uint16_t match_cursor;     /* lowest sent-frame index not yet matched */
  bool sent_any;             /* the first frame goes out on the first next() */
  uint32_t last_send_ms;     /* pacing reference */
  uint32_t last_activity_ms; /* settle reference: last send OR receive */
  uint32_t last_seen_ms;     /* latest clock reading — stamps receives, since
                                selftest_on_body carries no time */
} Selftest;

/* Initialise a session for `link`: `n_frames` test frames from `my_node`,
 * paced `gap_ms` apart, settling after `settle_ms` of silence. */
void selftest_init(Selftest *s, uint8_t my_node, uint8_t link,
                   uint16_t n_frames, uint32_t gap_ms, uint32_t settle_ms);

/* Ask for the next test frame at monotonic time `now_ms`. Returns true and
 * fills `*out` when the pacing gap has elapsed and frames remain — never more
 * often, never past `n_frames`. On true the caller sends `*out` on s->link. */
bool selftest_next(Selftest *s, uint32_t now_ms, Frame *out);

/* Feed one decoded body (CRC-clean bytes, as the transport decoder hands up).
 * An exact echo of a sent frame counts `echoed`; a body that decodes but does
 * not match — or does not decode as a frame at all — counts `mismatched`; a
 * duplicate echo of an already-matched seq counts neither, so `echoed` can
 * never inflate past `sent`. */
void selftest_on_body(Selftest *s, const uint8_t *body, size_t len);

/* True once all `n_frames` are sent AND every sent frame is accounted for
 * (echoed or mismatched) — or `settle_ms` has elapsed since the last
 * send/receive activity, so total silence terminates in bounded time. */
bool selftest_done(const Selftest *s, uint32_t now_ms);

SelftestStats selftest_stats(const Selftest *s);

#ifdef __cplusplus
}
#endif

#endif /* BB_MCUHUB_SELFTEST_H */
