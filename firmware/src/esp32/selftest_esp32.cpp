/* The link self-test boot mode, ESP32 runner (issue #8; the design and the
 * pinned output format are ADR-0012). Compiled ONLY into a `-DBB_SELFTEST=1`
 * build, where it provides setup()/loop() in place of hub_main.cpp (whose
 * guard excludes it there); production builds compile this file to nothing.
 *
 * It drives the pure host-tested core (selftest.c) through the PRODUCTION link
 * layer — the same link_begin(), the same generated wire_contract.h, the same
 * -D pin/baud overrides — so a PASS certifies THIS configuration, never a
 * parallel test one. Each link this board realizes is tested sequentially
 * (IS_ROOT folds at compile time: root = link 0 host UART + link 1 backplane;
 * leaf = link 0 backplane), one verdict line per link on UART0/USB — the
 * out-of-band diagnostic console no link may ever claim — then the whole
 * cycle repeats about every 2 s: move the loopback jumper and watch FAIL flip
 * to PASS live, no reflash. The router, the floor, and the scheduler never
 * run; the watchdog is configured and fed exactly as hub_main does. */
#if defined(ARDUINO) && defined(BB_SELFTEST)

#include "esp_task_wdt.h"
#include <Arduino.h>
extern "C" {
#include "frame.h"
#include "link.h" /* link_* — all C linkage; hub_* stays unreferenced here */
#include "selftest.h"
}

/* The pin/baud values printed below are the SAME macros link_begin() consumed
 * — one shim-internal source (link_pins_esp32.h), so the verdict line can
 * never report a second copy of the defaults that drifted (ADR-0012). */
#include "link_pins_esp32.h"

#ifndef WDT_TIMEOUT_MS
#define WDT_TIMEOUT_MS 1000
#endif

/* Session shape: 100 frames paced 5 ms apart (~500 ms of traffic per link),
 * a 250 ms silence window so a dead link terminates in bounded time, and a
 * cycle repainted every ~2 s. The session inner loop and the idle wait both
 * feed the WDT every iteration, so the 1000 ms watchdog can never fire. */
#define SELFTEST_N_FRAMES 100
#define SELFTEST_GAP_MS 5
#define SELFTEST_SETTLE_MS 250
#define SELFTEST_CYCLE_MS 2000

/* How the realized backplane link is described on its verdict line. On a CAN
 * backplane the UART self-test procedure cannot pass (a lone TWAI transmitter
 * dies on ACK error — the CAN self-test mode is deferred, ADR-0012), so such
 * a build honestly reports FAIL 0/N; these labels only keep the line truthful.
 */
#if LINK1_TRANSPORT_UART
#define BP_SERIAL_NAME "Serial2"
#define BP_RX_PIN BACKPLANE_UART_RX_PIN
#define BP_TX_PIN BACKPLANE_UART_TX_PIN
#define BP_BAUD BACKPLANE_UART_BAUD
#else
#define BP_SERIAL_NAME "TWAI"
#define BP_RX_PIN CAN_RX_PIN
#define BP_TX_PIN CAN_TX_PIN
#define BP_BAUD CAN_BITRATE
#endif

/* The body callback is registered once (link_set_on_body); it feeds the core
 * ONLY with bodies that arrived on the link under test — a body from any other
 * link (e.g. a live peer on the other UART) is ignored, never miscounted. */
static Selftest *g_session = nullptr;
static uint8_t g_link_under_test = 0;

static void selftest_body_cb(uint8_t arrival_link, const uint8_t *body,
                             size_t len) {
  if (g_session != nullptr && arrival_link == g_link_under_test)
    selftest_on_body(g_session, body, len);
}

/* What to print for one local link index: the human name, the peripheral, and
 * the very pin/baud macros link_begin() used (link_pins_esp32.h). */
struct LinkDesc {
  const char *name;
  const char *serial;
  unsigned rx, tx;
  unsigned long baud;
};

static LinkDesc link_desc(uint8_t link) {
#if IS_ROOT
  if (link == 0)
    return {"host-uart", "Serial1", HOST_UART_RX_PIN, HOST_UART_TX_PIN,
            HOST_UART_BAUD};
#else
  (void)link; /* a leaf realizes only link 0 — the backplane to its parent */
#endif
  return {"backplane", BP_SERIAL_NAME, BP_RX_PIN, BP_TX_PIN, BP_BAUD};
}

/* Idle until `deadline_ms`, feeding the WDT — wrap-safe signed comparison. */
static void idle_until(uint32_t deadline_ms) {
  while ((int32_t)(deadline_ms - millis()) > 0) {
    esp_task_wdt_reset();
    delay(10);
  }
}

/* Run one loopback session on `link` through the production TX/RX path and
 * print its pinned verdict line. */
static void run_session(uint8_t link) {
  uint32_t drop_before = link_rx_drop(link);

  Selftest s;
  selftest_init(&s, MY_NODE, link, SELFTEST_N_FRAMES, SELFTEST_GAP_MS,
                SELFTEST_SETTLE_MS);
  g_link_under_test = link;
  g_session = &s;

  Frame f;
  while (!selftest_done(&s, millis())) {
    esp_task_wdt_reset(); /* every iteration — the WDT never fires mid-session
                           */
    if (selftest_next(&s, millis(), &f))
      link_send_on_link(link, &f); /* the production TX path */
    link_pump(); /* the production RX path → selftest_body_cb */
  }
  g_session = nullptr;

  SelftestStats st = selftest_stats(&s);
  uint32_t drop_delta = link_rx_drop(link) - drop_before;
  bool pass = st.echoed == st.sent && st.mismatched == 0;

  /* The pinned, greppable verdict line (ADR-0012); the FAIL hint names this
   * board's ACTUAL rx/tx pins. */
  LinkDesc d = link_desc(link);
  Serial.printf(
      "link%u %s %s rx=%u tx=%u baud=%lu: %s echoed=%u/%u rx_drop=+%u",
      (unsigned)link, d.name, d.serial, d.rx, d.tx, d.baud,
      pass ? "PASS" : "FAIL", (unsigned)st.echoed, (unsigned)st.sent,
      (unsigned)drop_delta);
  if (!pass)
    Serial.printf("   <- jumper %u<->%u or pin dead", d.rx, d.tx);
  Serial.printf("\n");
}

void setup() {
  /* The task watchdog, configured and subscribed exactly as hub_main does
   * (ESP-IDF 5.x config-struct API; the TWDT is already inited by Arduino). */
  esp_task_wdt_config_t wdt = {
      .timeout_ms = WDT_TIMEOUT_MS,
      .idle_core_mask = 0,
      .trigger_panic = true,
  };
  esp_task_wdt_reconfigure(&wdt);

  /* The PRODUCTION link bring-up — the same generated wire_contract.h, the
   * same -D overrides. A PASS below certifies this configuration. */
  link_begin();
  link_set_on_body(selftest_body_cb);

  /* UART0/USB — the out-of-band diagnostic console (ADR-0012); production
   * firmware never touches it (the host seam is deliberately Serial1). 115200
   * matches the repo's monitor_speed. */
  Serial.begin(115200);

  esp_task_wdt_add(NULL);
}

void loop() {
  esp_task_wdt_reset(); /* fed at the top, and inside every wait below */
  uint32_t cycle_start = millis();

  /* One banner + one line per realized link, every cycle. */
  Serial.printf("BB_SELFTEST node=0x%02X root=%d\n", (unsigned)MY_NODE,
                IS_ROOT ? 1 : 0);
#if IS_ROOT
  run_session(0); /* the host UART (Serial1) */
  run_session(1); /* the backplane */
#else
  run_session(0); /* a leaf's only link: the backplane to its parent */
#endif

  /* Repaint roughly every 2 s — move the jumper, watch the verdict flip. */
  idle_until(cycle_start + SELFTEST_CYCLE_MS);
}

#endif /* ARDUINO && BB_SELFTEST */
