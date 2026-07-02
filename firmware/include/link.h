/* The link + board interface shared across the firmware (§03/§08).
 *
 * Declared once here with C linkage so the pure-C hub logic
 * (hubs/<hub>/mcu/*.c) and the C++ ESP32 glue (firmware/src/esp32/*.cpp) agree
 * on the symbols — a mismatched extern is the linker error this header exists
 * to prevent. */
#ifndef BB_MCUHUB_LINK_H
#define BB_MCUHUB_LINK_H

#include "frame.h"
#include "scheduler.h" /* for Task, used by hub_tasks() */
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* --- the link layer (firmware/src/esp32/link_esp32.cpp) --- */
void link_begin(void);
void link_pump(void);

/* A verified body arrived; `arrival_link` is the LOCAL LINK INDEX it came in
 * on (0 = the up-link from the parent/host; >= 1 = that downlink). The router
 * needs the direction to know whether the frame's NODE is a source or a
 * destination (ADR-0011). */
void link_set_on_body(void (*cb)(uint8_t arrival_link, const uint8_t *body,
                                 size_t len));

/* Send a frame on a per-hub-local LINK INDEX (ADR-0006). Link 0 is the up-link
 * (host UART on the root, parent backplane on a leaf); downlinks are 1..N. The
 * board realizes the links it physically has and stubs the rest. The router
 * calls this with route_table[node]. */
void link_send_on_link(uint8_t link, const Frame *f);

/* Direction-named helpers the per-hub glue (status/sense ticks) uses to send a
 * locally-produced frame UP toward the parent/host — link 0. Kept as the
 * legible names the generated ticks emit; link_send_on_link(0, f) is the same
 * path. */
void link_send_up(const Frame *f); /* toward parent/host (link 0); bridge
                                      re-frames UART/CAN */

/* Legible CAN-seam failure counters (§03), for telemetry / a future status
 * port. tx_oversize: a body over the 512-byte ceiling, or a TWAI TX abandoned
 * mid-body. rx_*: a fragment dropped at the CAN seam — never delivered partial
 * (§04). */
uint32_t link_tx_oversize_drops(void);
uint32_t link_rx_frag_orphan(void);
uint32_t link_rx_frag_drop(void);
uint32_t link_rx_crc_fail(void);

/* --- supplied by each board sketch (hubs/<hub>/mcu/main_*.cpp) --- */
void hub_setup(void);
void hub_on_body(uint8_t arrival_link, const uint8_t *body, size_t len);
Task *hub_tasks(size_t *n_tasks);

#ifdef __cplusplus
}
#endif

#endif /* BB_MCUHUB_LINK_H */
