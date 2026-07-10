/* ESP32 shim-internal pin/baud defaults + role derivation — the SINGLE source
 * of truth, shared by link_esp32.cpp (which configures the peripherals) and
 * selftest_esp32.cpp (which prints these values in its verdict lines,
 * ADR-0012). Extracted verbatim from link_esp32.cpp so the self-test can never
 * report a second copy of the defaults that drifts from what link_begin()
 * actually used. Semantics are unchanged: `-D` overrides still win (every
 * default is #ifndef-guarded), and the IS_ROOT-dependent defaults still fold
 * at preprocess time (MY_NODE and the generated ROOT_NODE are both integer
 * literals). Shim-internal: lives in src/esp32/, not include/ — consumers
 * override via -D, never by including this. */
#ifndef BB_MCUHUB_LINK_PINS_ESP32_H
#define BB_MCUHUB_LINK_PINS_ESP32_H

#include "wire_contract.h" /* LINK1_TRANSPORT_UART, ROOT_NODE — generated facts */

/* Root-ness is GENERATED, not a hand-set build flag (ADR-0006). The DSL
 * declares the root (the hub with parent: :host); the generator emits ROOT_NODE
 * into wire_contract.h; this board's MY_NODE (-DMY_NODE=0x..) and ROOT_NODE are
 * both integer literals, so IS_ROOT folds at preprocess time and every #if
 * IS_ROOT resolves per env. Forgetting a flag can no longer silently make
 * a root compile as a leaf. */
#define IS_ROOT (MY_NODE == ROOT_NODE)

/* The root's downlink-1 transport (ADR-0006). A leaf has no downlinks, so this
 * define is only meaningful when IS_ROOT; default it for a clean compile of a
 * leaf (whose only link, 0, is the up-link). */
#ifndef LINK1_TRANSPORT_UART
#define LINK1_TRANSPORT_UART 1
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

#endif /* BB_MCUHUB_LINK_PINS_ESP32_H */
