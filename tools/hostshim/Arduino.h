// Minimal Arduino shim so src/link_owner.cpp -- and the real src/od_log.h it
// includes -- can be compiled and tested on the host.
//
// Deliberately the REAL od_log.h rather than a stub of it: that header is what
// link_owner.cpp actually includes, so shimming Arduino instead of shimming our own
// header keeps the test compiling the same code the firmware does. Only the two
// Arduino types od_log.h names are declared here, plus millis().
//
// Test-only: never on an include path for a firmware build, which always gets the
// real Arduino core from the PlatformIO framework.
#ifndef OD_HOSTSHIM_ARDUINO_H
#define OD_HOSTSHIM_ARDUINO_H

#include <stdint.h>
#include <stddef.h>

// The test drives this directly, so it can place the clock wherever a case needs
// -- including just below a wrap boundary, which is not reachable by waiting.
//
// Read atomically because the concurrency cases advance it from one thread while
// the code under test reads it from another; a plain access would be a data race
// in the HARNESS and would mask the race being hunted for in the code.
extern volatile uint32_t od_test_millis;
static inline uint32_t millis(void) {
    return __atomic_load_n(&od_test_millis, __ATOMIC_RELAXED);
}

// od_log.h names these in declarations the test never calls; they only have to
// exist for the header to parse.
class Stream;
typedef void* TaskHandle_t;

#endif  // OD_HOSTSHIM_ARDUINO_H
