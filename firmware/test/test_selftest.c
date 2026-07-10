/* Host-compiled test of the link self-test core (issue #8; the boot-mode
 * design is ADR-0012). The harness drives the EXACT loop the ESP32 runner
 * will, minus hardware: selftest_next → frame_encode_body → transport_encode
 * → [optional byte corruption] → transport_decoder_feed → on_body →
 * selftest_on_body — the real codec collaborators (frame.c, transport.c,
 * crc16.c, cobs.c), no mocks. Time is driven explicitly; no sleeps. */
#include "selftest.h"
#include "transport.h"
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

/* --- the runner loop, minus hardware ------------------------------------ */

/* the decoder's body callback feeds the session — the runner's exact shape */
static void feed_session(const uint8_t *body, size_t len, void *ctx) {
  selftest_on_body((Selftest *)ctx, body, len);
}

/* Frame → CRC-covered body → COBS+CRC wire bytes. Returns the wire length. */
static size_t frame_to_wire(const Frame *f, uint8_t *wire, size_t cap) {
  uint8_t body[FRAME_MAX_BODY];
  size_t blen = frame_encode_body(f, body, sizeof(body));
  if (blen == 0)
    return 0;
  return transport_encode(body, blen, wire, cap);
}

int main(void) {
  printf("== bb_mcuhub C selftest harness ==\n");

  /* PACING + FRAME SHAPE: the session emits at most one frame per gap_ms,
   * never past n_frames; every test frame carries my_node, SELFTEST_PORT, an
   * incrementing seq, no t_dev, and a seq-derived payload. */
  {
    Selftest s;
    selftest_init(&s, 0x02, 1, 3, 50, 200);
    CHECK(s.link == 1, "the link under test is a readable struct member");

    Frame f0, f1, f2, junk;
    CHECK(selftest_next(&s, 1000, &f0), "the first frame emits immediately");
    CHECK(!selftest_next(&s, 1000, &junk),
          "a second frame at the same instant is refused (pacing)");
    CHECK(!selftest_next(&s, 1049, &junk),
          "gap_ms - 1 after a send is still too soon");
    CHECK(selftest_next(&s, 1050, &f1), "gap_ms after a send emits the next");
    CHECK(selftest_next(&s, 1100, &f2), "the third frame follows one gap on");
    CHECK(!selftest_next(&s, 999999, &junk),
          "past n_frames nothing more is ever emitted");

    CHECK(f0.node == 0x02 && f1.node == 0x02 && f2.node == 0x02,
          "test frames carry the board's own node");
    CHECK(f0.port == SELFTEST_PORT && f2.port == SELFTEST_PORT,
          "test frames carry the reserved SELFTEST_PORT (0xFF)");
    CHECK(!f0.stamped && !f1.stamped && !f2.stamped,
          "test frames are unstamped (no t_dev)");
    CHECK(f1.seq == (uint16_t)(f0.seq + 1) && f2.seq == (uint16_t)(f1.seq + 1),
          "seq increments by one per frame");
    CHECK(f0.payload_len > 0 && f0.payload_len < FRAME_MAX_PAYLOAD &&
              f0.payload_len == f1.payload_len,
          "the payload is non-empty and well under FRAME_MAX_PAYLOAD");
    CHECK(memcmp(f0.payload, f1.payload, f0.payload_len) != 0,
          "payloads differ per seq (a swapped echo cannot pass)");

    SelftestStats st = selftest_stats(&s);
    CHECK(st.sent == 3 && st.echoed == 0 && st.mismatched == 0,
          "stats after emit-only: sent=3 echoed=0 mismatched=0");
  }

  /* CLEAN LOOP: N frames round-trip through the real encode→decode path
   * (a perfect jumper); every frame is accounted for, so the session is done
   * the moment the last echo lands — no settle wait. */
  {
    enum { N = 8, GAP = 10, SETTLE = 500 };
    Selftest s;
    selftest_init(&s, 0x03, 0, N, GAP, SETTLE);
    TransportDecoder d;
    transport_decoder_init(&d);

    uint32_t now = 100; /* explicit simulated time */
    const uint32_t start = now;
    int guard = 0;
    while (!selftest_done(&s, now) && guard < 100000) {
      Frame f;
      if (selftest_next(&s, now, &f)) {
        uint8_t wire[FRAME_MAX_WIRE];
        size_t w = frame_to_wire(&f, wire, sizeof(wire));
        transport_decoder_feed(&d, wire, w, feed_session, &s);
      }
      now += 1;
      guard++;
    }

    SelftestStats st = selftest_stats(&s);
    CHECK(guard < 100000, "the clean-loop session terminates");
    CHECK(st.sent == N, "clean loop: sent == N");
    CHECK(st.echoed == N, "clean loop: echoed == sent == N");
    CHECK(st.mismatched == 0, "clean loop: mismatched == 0");
    CHECK(d.rx_drop == 0, "clean loop: the decoder drops nothing");
    CHECK(now - start < (uint32_t)(N - 1) * GAP + SETTLE,
          "clean loop: done WITHOUT waiting out the settle window");
  }

  /* CORRUPTED BYTES: some frames' wire bytes are damaged before the decoder.
   * The core never sees them — the transport decoder drops and counts them
   * (rx_drop), echoed falls short of sent, and the session still terminates,
   * via the settle window. */
  {
    enum { N = 10, GAP = 10, SETTLE = 300 };
    Selftest s;
    selftest_init(&s, 0x03, 0, N, GAP, SETTLE);
    TransportDecoder d;
    transport_decoder_init(&d);

    uint32_t now = 0;
    const uint32_t start = now;
    int guard = 0, n_sent = 0, n_corrupted = 0;
    while (!selftest_done(&s, now) && guard < 100000) {
      Frame f;
      if (selftest_next(&s, now, &f)) {
        uint8_t wire[FRAME_MAX_WIRE];
        size_t w = frame_to_wire(&f, wire, sizeof(wire));
        if (n_sent % 3 == 0) { /* frames 0,3,6,9 — incl. the LAST send */
          /* damage a COBS body byte to another non-zero value: the frame
           * stays one frame (no new delimiter) and fails its CRC */
          wire[2] = (wire[2] == 0x55) ? 0xAA : 0x55;
          n_corrupted++;
        }
        n_sent++;
        transport_decoder_feed(&d, wire, w, feed_session, &s);
      }
      now += 1;
      guard++;
    }

    SelftestStats st = selftest_stats(&s);
    CHECK(guard < 100000, "corruption: the session still terminates");
    CHECK(n_corrupted == 4 && d.rx_drop == 4,
          "corruption: the DECODER counts the damaged frames (rx_drop)");
    CHECK(st.sent == N && st.echoed == N - 4,
          "corruption: echoed < sent by exactly the damaged frames");
    CHECK(st.mismatched == 0,
          "corruption: the core never counts a CRC casualty (decoder's job)");
    CHECK(now - start >= (uint32_t)(N - 1) * GAP + SETTLE,
          "corruption: termination came via the settle window");
    CHECK(now - start < (uint32_t)(N - 1) * GAP + SETTLE + 5,
          "corruption: ... and promptly once it elapsed");
  }

  /* TOTAL SILENCE: nothing ever comes back (a dead link). The session sends
   * all N and terminates via settle in bounded simulated time; the boundary
   * is exact — one tick early is not done, at settle_ms it is. */
  {
    enum { N = 5, GAP = 10, SETTLE = 300 };
    Selftest s;
    selftest_init(&s, 0x07, 0, N, GAP, SETTLE);
    Frame f;
    uint32_t last_send = 0;
    for (int i = 0; i < N; i++) {
      uint32_t t = (uint32_t)(i * GAP);
      CHECK(selftest_next(&s, t, &f), "silence: frame emits on schedule");
      last_send = t;
    }
    CHECK(!selftest_done(&s, last_send + SETTLE - 1),
          "silence: one tick before the settle window elapses — not done");
    CHECK(selftest_done(&s, last_send + SETTLE),
          "silence: done exactly when settle_ms of silence has elapsed");
    SelftestStats st = selftest_stats(&s);
    CHECK(st.sent == N && st.echoed == 0 && st.mismatched == 0,
          "silence: sent=N echoed=0 mismatched=0");
  }

  /* WRONG BODIES: a CRC-clean body that is not the expected echo counts
   * mismatched — a tampered payload, a foreign port, and a body too short to
   * decode as a frame at all. */
  {
    Selftest s;
    selftest_init(&s, 0x03, 0, 2, 0, 100);
    TransportDecoder d;
    transport_decoder_init(&d);
    uint8_t wire[FRAME_MAX_WIRE];
    Frame f;

    CHECK(selftest_next(&s, 0, &f), "wrong bodies: frame 1 emits");
    f.payload[0] ^= 0xFF; /* tamper BEFORE framing: CRC-clean, wrong bytes */
    transport_decoder_feed(&d, wire, frame_to_wire(&f, wire, sizeof(wire)),
                           feed_session, &s);
    CHECK(selftest_next(&s, 0, &f), "wrong bodies: frame 2 emits");
    f.port = 0x10; /* a foreign port, payload intact */
    transport_decoder_feed(&d, wire, frame_to_wire(&f, wire, sizeof(wire)),
                           feed_session, &s);
    /* a 2-byte body: CRC-clean on the wire, but no frame can be this short */
    uint8_t stub[2] = {0xAB, 0xCD};
    transport_decoder_feed(&d, wire,
                           transport_encode(stub, 2, wire, sizeof(wire)),
                           feed_session, &s);

    SelftestStats st = selftest_stats(&s);
    CHECK(d.rx_drop == 0, "wrong bodies: all three pass the CRC seam");
    CHECK(st.echoed == 0, "wrong bodies: none counts as echoed");
    CHECK(st.mismatched == 3,
          "wrong bodies: tampered payload + foreign port + undecodable body "
          "all count mismatched");
  }

  /* DUPLICATE ECHOES: an already-matched seq arriving again moves no counter
   * — echoed can never inflate past sent. */
  {
    Selftest s;
    selftest_init(&s, 0x03, 0, 2, 0, 100);
    TransportDecoder d;
    transport_decoder_init(&d);
    uint8_t wire[FRAME_MAX_WIRE];
    Frame f;

    CHECK(selftest_next(&s, 0, &f), "duplicates: frame 1 emits");
    size_t w = frame_to_wire(&f, wire, sizeof(wire));
    transport_decoder_feed(&d, wire, w, feed_session, &s);
    CHECK(selftest_stats(&s).echoed == 1, "duplicates: the echo matched once");
    transport_decoder_feed(&d, wire, w, feed_session, &s); /* replay it */
    {
      SelftestStats st = selftest_stats(&s);
      CHECK(st.echoed == 1 && st.mismatched == 0,
            "duplicates: a replayed echo moves NEITHER counter");
      CHECK(st.echoed <= st.sent, "duplicates: echoed never exceeds sent");
    }
    CHECK(selftest_next(&s, 1, &f), "duplicates: frame 2 emits");
    transport_decoder_feed(&d, wire, frame_to_wire(&f, wire, sizeof(wire)),
                           feed_session, &s);
    {
      SelftestStats st = selftest_stats(&s);
      CHECK(st.sent == 2 && st.echoed == 2 && st.mismatched == 0,
            "duplicates: the session still completes cleanly");
      CHECK(selftest_done(&s, 2),
            "duplicates: done once every sent frame is accounted for");
    }
  }

  if (g_fail == 0) {
    printf("\nALL C SELFTEST CHECKS PASSED\n");
    return 0;
  }
  printf("\n%d C SELFTEST CHECK(S) FAILED\n", g_fail);
  return 1;
}
