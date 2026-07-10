/* The generic hub main loop (§08), ESP32. Brings up the link, runs the
 * generated per-port schedule, pumps the router (meaning-blind), and feeds the
 * watchdog at the top of every loop so a wedged tick resets the chip into
 * born-disarmed (§05).
 *
 * The concrete hub (its tasks[] table from schedule.gen.h, its tick functions,
 * and on_command) is supplied by the board sketch, which #includes its hub's
 * mcu/*.c and schedule.gen.h and defines hub_setup()/the task table. */
#if defined(ARDUINO) && !defined(BB_SELFTEST)

#include "esp_task_wdt.h"
#include <Arduino.h>
extern "C" {
#include "frame.h"
#include "link.h" /* link_* and the board's hub_* — all C linkage */
#include "scheduler.h"
}

#ifndef WDT_TIMEOUT_MS
#define WDT_TIMEOUT_MS 1000
#endif

static Task *g_tasks = nullptr;
static size_t g_n_tasks = 0;

void setup() {
  /* ESP-IDF 5.x watchdog API: config struct, not (timeout, panic). A wedged
   * tick that misses the feed resets the chip → it comes up born-disarmed
   * (§05). */
  esp_task_wdt_config_t wdt = {
      .timeout_ms = WDT_TIMEOUT_MS,
      .idle_core_mask = 0,
      .trigger_panic = true,
  };
  esp_task_wdt_reconfigure(
      &wdt); /* the TWDT is already inited by Arduino; reconfigure it */

  /* Bring-up runs BEFORE the task subscribes to the watchdog. hub_setup() is a
   * one-time bounded init that can legitimately exceed WDT_TIMEOUT_MS (e.g. a
   * FOC initFOC() sensor-alignment that spins the motor, or an i2c-ng settle
   * delay) — the watchdog guards the steady-state LOOP, not bring-up (§08).
   * Subscribing the task before hub_setup() would reset the chip mid-alignment,
   * before loop() can ever feed it (a boot loop). Add the task only once setup
   * is complete; the first feed is at the top of loop(). */
  link_begin();
  link_set_on_body(hub_on_body);
  hub_setup();
  g_tasks = hub_tasks(&g_n_tasks);

  esp_task_wdt_add(NULL);
}

void loop() {
  esp_task_wdt_reset(); /* fed at the TOP, never from inside a tick (§08) */
  uint32_t now_us = micros();

  link_pump(); /* drain inbound; dispatch via hub_on_body → router */
  scheduler_run(g_tasks, g_n_tasks, now_us);
}

#endif /* ARDUINO */
