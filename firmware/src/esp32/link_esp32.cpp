/* ESP32 link layer (§03, ADR-0006): the host↔root UART seam and the per-hub
 * LOCAL LINKS, all carrying the SAME COBS+CRC frame: one frame, big-endian
 * body, end-to-end CRC across the re-framing boundary.
 *
 * Links are per-hub-local INDICES (ADR-0006): link 0 is the up-link (the host
 * UART on the root, the parent backplane on a leaf); downlinks are 1..N. The
 * generated route table maps each NODE to one of these indices; the router
 * calls `link_send_on_link(idx, f)`. THIS BOARD realizes only the links it
 * physically has — for the single-downlink example boards that is link 0 (up)
 * and link 1 (the one backplane); link >= 2 is a no-op stub (no hardware/test
 * validates an N-peripheral driver — deliberately out of scope, see ADR-0006
 * Consequences).
 *
 * Transport is a property of a LINK, not a robot-wide flag (ADR-0006). The
 * generator emits LINK1_TRANSPORT_UART into wire_contract.h from the root's
 * downlink-1 child's DECLARED uplink; this file reads it at boot for the one
 * backplane it realizes:
 *
 *   LINK1_TRANSPORT_UART == 0  → CAN/TWAI backplane with §03 segmentation
 *                                 (a 29-bit id [NODE:8][PORT:8][rsv:13],
 *                                  a wide body splits across CAN frames).
 *   LINK1_TRANSPORT_UART == 1  → a plain UART (Serial2) backplane carrying
 *                                 the SAME COBS+CRC frame with NO segmentation
 * — a wide body (e.g. the 54-byte IMU) rides one COBS frame, exactly like the
 * host↔root UART.
 *
 * Role (the up-link, link 0) is DERIVED, not flagged (ADR-0006). The generator
 * emits ROOT_NODE (the declared parent: :host hub) into wire_contract.h, and
 * this file computes IS_ROOT = (MY_NODE == ROOT_NODE):
 *   IS_ROOT  → link 0 is UART to the host; link 1 (the backplane) reaches a
 *              child.
 *   (else)   → link 0 is the backplane to the parent (a leaf); no downlinks.
 *
 * A root hub with a UART backplane therefore has TWO UARTs: Serial1 (host, on
 * GPIO 16/17 — NOT UART0/USB) and Serial2 (backplane = link 1). The host↔root
 * seam is unchanged regardless of the backplane transport.
 *
 * This file only compiles under the Arduino/ESP32 framework (PlatformIO). The
 * pure codec it rides on (frame/transport/crc16/cobs) is the same host-tested
 * C. */
#if defined(ARDUINO)

#include <Arduino.h>
extern "C" {
#include "frame.h"
#include "link.h"
#include "segment.h"
#include "transport.h"
#include "wire_contract.h" /* LINK1_TRANSPORT_UART, ROOT_NODE — generated facts */
}

/* Root-ness is GENERATED, not a hand-set build flag (ADR-0006). The DSL
 * declares the root (the hub with parent: :host); the generator emits ROOT_NODE
 * into wire_contract.h; this board's MY_NODE (-DMY_NODE=0x..) and ROOT_NODE are
 * both integer literals, so IS_ROOT folds at preprocess time and every #if
 * IS_ROOT below resolves per env. Forgetting a flag can no longer silently make
 * a root compile as a leaf. */
#define IS_ROOT (MY_NODE == ROOT_NODE)

/* The root's downlink-1 transport (ADR-0006). A leaf has no downlinks, so this
 * define is only meaningful when IS_ROOT; default it for a clean compile of a
 * leaf (whose only link, 0, is the up-link). */
#ifndef LINK1_TRANSPORT_UART
#define LINK1_TRANSPORT_UART 1
#endif

#if LINK1_TRANSPORT_UART
#include "HardwareSerial.h" /* Serial2 — the COBS+CRC backplane */
#else
#include "driver/twai.h" /* the default CAN/TWAI backplane */
#endif

#ifndef HOST_UART_BAUD
#define HOST_UART_BAUD 1000000
#endif

/* The host↔root-hub seam is UART1 (Serial1) on dedicated GPIO pins — NOT Serial
 * (UART0), which on the ESP32 is the USB-bridge console (GPIO 1/3) and is not
 * wired to the host. Pins match the proven wiring: the Blaster
 * receives the host on RX 16 and transmits on TX 17 (the two ends crossed by
 * wiring). The Pi's PL011 (/dev/ttyAMA0) is the other end. Overridable per
 * board. */
#ifndef HOST_UART_RX_PIN
#define HOST_UART_RX_PIN 16
#endif
#ifndef HOST_UART_TX_PIN
#define HOST_UART_TX_PIN 17
#endif

#if LINK1_TRANSPORT_UART
/* The UART backplane (Serial2 = link 1). Same baud as the host seam by default;
 * the pins are overridable per board. Defaults: a root/Blaster hub uses TX 26 /
 * RX 27, a leaf uses TX 17 / RX 16 — the two ends are crossed by wiring, not by
 * code. */
#ifndef BACKPLANE_UART_BAUD
#define BACKPLANE_UART_BAUD 1000000
#endif
#ifndef BACKPLANE_UART_TX_PIN
#if IS_ROOT
#define BACKPLANE_UART_TX_PIN 26
#else
#define BACKPLANE_UART_TX_PIN 17
#endif
#endif
#ifndef BACKPLANE_UART_RX_PIN
#if IS_ROOT
#define BACKPLANE_UART_RX_PIN 27
#else
#define BACKPLANE_UART_RX_PIN 16
#endif
#endif
#else
#ifndef CAN_BITRATE
#define CAN_BITRATE 1000000
#endif
#ifndef CAN_TX_PIN
#define CAN_TX_PIN 16
#endif
#ifndef CAN_RX_PIN
#define CAN_RX_PIN 17
#endif
#endif

/* Callback the runtime sets: a verified body arrived, tagged with the LOCAL
 * LINK INDEX it came in on — the router routes by direction (ADR-0011). */
static void (*g_on_body)(uint8_t arrival_link, const uint8_t *body,
                         size_t len) = nullptr;

/* The one backplane this board realizes: downlink 1 on a root, the up-link
 * (0) on a leaf. IS_ROOT folds at preprocess time. */
#define BACKPLANE_LINK_IDX (IS_ROOT ? 1 : 0)

#if LINK1_TRANSPORT_UART
/* The UART backplane is a streaming COBS+CRC seam (transport.c), exactly like
 * the host↔root UART. No segmentation: a wide body rides one COBS frame. */
static TransportDecoder g_bp_rx;

static void bp_body_cb(const uint8_t *body, size_t len, void *) {
  if (g_on_body)
    g_on_body(BACKPLANE_LINK_IDX, body, len);
}

/* Legible failure counters (§03). On a UART backplane there is no
 * fragmentation, so the frag/oversize counters are vacuously zero; rx_drop is
 * the seam's analog of the CAN seam's drop (a corrupt/truncated COBS frame). */
uint32_t link_tx_oversize_drops(void) { return 0; }
uint32_t link_rx_frag_orphan(void) { return 0; }
uint32_t link_rx_frag_drop(void) { return g_bp_rx.rx_drop; }
uint32_t link_rx_crc_fail(void) { return g_bp_rx.rx_drop; }
#else
/* The CAN backplane is segmented (§03): a wide body splits across CAN frames on
 * TX and is reassembled, CRC-checked, in order on RX. The fragment metadata
 * rides the 13 reserved id bits; the data field stays 100% body bytes. */
static SegReasm g_can_rx;

/* Legible failure counters (§03), exposed for telemetry / a future status port:
 *  - tx_oversize: a body over the 512-byte ceiling — refused, never truncated
 *    (a should-never-happen belt to the §06 boot size-check), or a TWAI TX that
 *    failed mid-body so the body is abandoned (the RX side drops the partial).
 *  - rx_*: the CAN seam's analog of the UART seam's rx_drop. */
static uint32_t g_tx_oversize_drop = 0;
uint32_t link_tx_oversize_drops(void) { return g_tx_oversize_drop; }
uint32_t link_rx_frag_orphan(void) { return g_can_rx.rx_frag_orphan; }
uint32_t link_rx_frag_drop(void) { return g_can_rx.rx_frag_drop; }
uint32_t link_rx_crc_fail(void) { return g_can_rx.rx_crc_fail; }

/* Reassembler → runtime: a complete, CRC-clean CAN body (CRC already stripped).
 */
static void can_body_cb(const uint8_t *body, size_t len, void *) {
  if (g_on_body)
    g_on_body(BACKPLANE_LINK_IDX, body, len);
}
#endif

#if IS_ROOT
static TransportDecoder
    g_uart_rx; /* the host UART seam — only the root hub has one */

static void uart_body_cb(const uint8_t *body, size_t len, void *) {
  /* the host UART is the root's up-link: arrival link 0 (from the parent) */
  if (g_on_body)
    g_on_body(0, body, len);
}
#endif

void link_set_on_body(void (*cb)(uint8_t arrival_link, const uint8_t *body,
                                 size_t len)) {
  g_on_body = cb;
}

void link_begin(void) {
#if LINK1_TRANSPORT_UART
  transport_decoder_init(&g_bp_rx); /* the COBS+CRC backplane seam (link 1) */
#else
  seg_reasm_init(&g_can_rx); /* the CAN backplane reassembler (link 1) */
#endif

#if IS_ROOT
  transport_decoder_init(&g_uart_rx);
  /* UART1 to the host on dedicated GPIO pins (NOT UART0/USB) — independent of
   * the backplane transport. RX 16 / TX 17, crossed to the Pi's PL011 by
   * wiring. */
  Serial1.begin(HOST_UART_BAUD, SERIAL_8N1, HOST_UART_RX_PIN, HOST_UART_TX_PIN);
#endif

#if LINK1_TRANSPORT_UART
  /* the backplane (link 1): a second hardware serial carrying the same COBS+CRC
   * frame */
  Serial2.begin(BACKPLANE_UART_BAUD, SERIAL_8N1, BACKPLANE_UART_RX_PIN,
                BACKPLANE_UART_TX_PIN);
#else
  /* the backplane (link 1): CAN/TWAI (every role on an all-CAN robot has one)
   */
  twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT(
      (gpio_num_t)CAN_TX_PIN, (gpio_num_t)CAN_RX_PIN, TWAI_MODE_NORMAL);
  twai_timing_config_t t = TWAI_TIMING_CONFIG_1MBITS();
  twai_filter_config_t fcfg = TWAI_FILTER_CONFIG_ACCEPT_ALL();
  twai_driver_install(&g, &t, &fcfg);
  twai_start();
#endif
}

/* Send a frame on the UP-LINK (link 0). On the root hub that is the host UART;
 * on a leaf, it is the parent backplane. The body is identical; only the
 * transport differs. Kept under the legible name the generated sense/status
 * ticks call; link_send_on_link(0, f) routes here. */
void link_send_up(const Frame *f) {
  uint8_t body[FRAME_MAX_BODY];
  size_t body_len = frame_encode_body(f, body, sizeof(body));
  if (body_len == 0)
    return;

#if IS_ROOT
  /* UP from the root hub is the host UART (Serial1 on GPIO 16/17) — unchanged
   * by the backplane. */
  uint8_t wire[FRAME_MAX_WIRE];
  size_t w = transport_encode(body, body_len, wire, sizeof(wire));
  Serial1.write(wire, w);
#elif LINK1_TRANSPORT_UART
  /* UP from a leaf over a UART backplane: COBS+CRC the whole body into one
   * frame. No segmentation — a wide body rides one COBS frame, exactly like the
   * host seam (transport.c). */
  uint8_t wire[FRAME_MAX_WIRE];
  size_t w = transport_encode(body, body_len, wire, sizeof(wire));
  Serial2.write(wire, w);
#else
  /* UP from a leaf over a CAN backplane is segmented (§03): split body || CRC
   * into ordered frames, fragment metadata in the 13 reserved id bits. ESP32's
   * built-in TWAI is classic CAN (8-byte data field), so even a small body
   * fragments (an 8-byte body → 10 with CRC → 2 frames); a CAN-FD board would
   * carry up to 512 B with fewer frames. The whole body crosses byte-identical
   * — never truncated. */
  CanFrame frags[SEG_MAX_FRAGS];
  size_t n_frags = 0;
  if (!seg_split(f->node, f->port, f->seq, body, body_len, frags, SEG_MAX_FRAGS,
                 &n_frags)) {
    g_tx_oversize_drop++; /* over the 512-byte ceiling — the §06 size-check
                             should forbid this */
    return;
  }
  /* Emit all fragments of this body back-to-back, in index order, before
   * returning — strict per-(node,port) FIFO (§04). If a transmit fails
   * mid-body, abandon it: the receiver's FIRST-seeded reassembler drops the
   * now-incomplete body cleanly, exactly as a bus loss would (count it as
   * oversize/abandoned). */
  for (size_t i = 0; i < n_frags; i++) {
    twai_message_t m = {};
    m.identifier = frags[i].id;
    m.extd = 1;
    m.data_length_code = frags[i].len;
    for (size_t j = 0; j < frags[i].len; j++)
      m.data[j] = frags[i].data[j];
    if (twai_transmit(&m, pdMS_TO_TICKS(1)) != ESP_OK) {
      g_tx_oversize_drop++; /* abandoned mid-body — receiver drops the partial
                             */
      return;
    }
  }
#endif
}

/* Send a frame DOWN to a child over the realized backplane (link 1). Only a
 * root hub bridges host→child: it re-frames the SAME body onto its backplane
 * (Serial2 for a UART backplane, segmented CAN otherwise) — the mirror of a
 * leaf's link_send_up over the backplane. A non-root hub has no children, so
 * this is a no-op there. Reached via link_send_on_link(1, f). */
static void link_send_down(const Frame *f) {
#if IS_ROOT
  uint8_t body[FRAME_MAX_BODY];
  size_t body_len = frame_encode_body(f, body, sizeof(body));
  if (body_len == 0)
    return;

#if LINK1_TRANSPORT_UART
  /* DOWN over a UART backplane: COBS+CRC the whole body into one frame (no
   * segmentation), exactly like a leaf's up-send (transport.c). */
  uint8_t wire[FRAME_MAX_WIRE];
  size_t w = transport_encode(body, body_len, wire, sizeof(wire));
  Serial2.write(wire, w);
#else
  /* DOWN over a CAN backplane is segmented (§03), same as the leaf's up-send.
   */
  CanFrame frags[SEG_MAX_FRAGS];
  size_t n_frags = 0;
  if (!seg_split(f->node, f->port, f->seq, body, body_len, frags, SEG_MAX_FRAGS,
                 &n_frags)) {
    g_tx_oversize_drop++;
    return;
  }
  for (size_t i = 0; i < n_frags; i++) {
    twai_message_t m = {};
    m.identifier = frags[i].id;
    m.extd = 1;
    m.data_length_code = frags[i].len;
    for (size_t j = 0; j < frags[i].len; j++)
      m.data[j] = frags[i].data[j];
    if (twai_transmit(&m, pdMS_TO_TICKS(1)) != ESP_OK) {
      g_tx_oversize_drop++;
      return;
    }
  }
#endif
#else
  (void)f; /* a leaf has no children — nothing to send down */
#endif
}

/* Map a per-hub-local LINK INDEX to this board's peripheral (ADR-0006). This
 * board realizes ONLY the links its hardware has: link 0 (the up-link) and, on
 * a root, link 1 (the one backplane). link >= 2 is a no-op stub — no hardware
 * and no harness exercises an N-peripheral driver, so it is deliberately left
 * out of the safety-critical relay path (ADR-0006 Consequences). The router's
 * route table never produces an index this board lacks for a robot whose
 * firmware exists. */
void link_send_on_link(uint8_t link, const Frame *f) {
  switch (link) {
  case 0:
    link_send_up(
        f); /* the up-link: host UART (root) / parent backplane (leaf) */
    break;
  case 1:
    link_send_down(f); /* the one backplane this board realizes (root only) */
    break;
  default:
    (void)f; /* link >= 2: not realized on this board — see ADR-0006 */
    break;
  }
}

/* Pump inbound bytes/frames toward g_on_body. Call every loop. */
void link_pump(void) {
#if IS_ROOT
  while (Serial1.available() > 0) {
    uint8_t b = (uint8_t)Serial1.read();
    transport_decoder_feed(&g_uart_rx, &b, 1, uart_body_cb, nullptr);
  }
#endif

#if LINK1_TRANSPORT_UART
  /* The UART backplane: feed bytes into the COBS+CRC decoder. It calls
   * g_on_body only with a complete, CRC-clean body (CRC stripped) — a
   * corrupt/truncated frame is dropped and counted, never delivered partial
   * (§03/§04). */
  while (Serial2.available() > 0) {
    uint8_t b = (uint8_t)Serial2.read();
    transport_decoder_feed(&g_bp_rx, &b, 1, bp_body_cb, nullptr);
  }
#else
  /* CAN: feed each frame into the reassembler. It calls g_on_body only with a
   * complete, CRC-clean, in-order body (CRC stripped) — a
   * gap/reorder/orphan/CRC failure is dropped and counted, never delivered
   * partial (§03/§04). */
  twai_message_t m;
  while (twai_receive(&m, 0) == ESP_OK) {
    CanFrame cf;
    cf.id = m.identifier;
    cf.len =
        m.data_length_code > SEG_CAN_DATA ? SEG_CAN_DATA : m.data_length_code;
    for (size_t i = 0; i < cf.len; i++)
      cf.data[i] = m.data[i];
    seg_reasm_feed(&g_can_rx, &cf, can_body_cb, nullptr);
  }
#endif
}

#endif /* ARDUINO */
