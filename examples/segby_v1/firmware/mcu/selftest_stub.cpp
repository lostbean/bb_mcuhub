/* Self-test build stub — the ONLY source the *_selftest envs compile. Its one
 * and only role is to #include a chassis header so PlatformIO's library
 * dependency finder sees the bb_mcuhub dependency and links the chassis into a
 * build that compiles no device file (setup()/loop() come from the chassis'
 * selftest_esp32.cpp; no device hooks, no floor, no router run here). */
extern "C" {
#include "link.h"
}
