#include "od_log.h"
#include <stdarg.h>
#include <stdio.h>

// Implemented in main.cpp: RTC-persisted wake cycle count on ESP32, always 0 on nRF52840.
uint32_t getDeepSleepCount();

// Log output destination, set once by od_log_init(). Stays NULL if
// od_log_init() is never called (e.g. DISABLE_USB_SERIAL builds), in which
// case all log calls become no-ops.
static Stream *s_port = NULL;

static const char level_chars[] = "EWID";

void od_log_init(Stream *port) {
    s_port = port;
}

void _od_log(int level, const char *fmt, ...) {
    if (s_port == NULL) {
        return;
    }

    char buf[256];
    unsigned long ms = millis();
    unsigned long cycleCount = (unsigned long)getDeepSleepCount();
    int pos = snprintf(buf, sizeof(buf), "[%04lu.%03lu|C%lu] %c: ",
                        ms / 1000, ms % 1000,
                        cycleCount,
                        level_chars[level]);
    if (pos < 0) {
        return;
    }

    va_list args;
    va_start(args, fmt);
    vsnprintf(buf + pos, sizeof(buf) - pos, fmt, args);
    va_end(args);

    s_port->println(buf);
}

void od_log_raw(const char *fmt, ...) {
    if (s_port == NULL) {
        return;
    }

    char buf[256];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    s_port->print(buf);
}

void od_log_flush(void) {
    if (s_port == NULL) {
        return;
    }

    s_port->flush();
    #ifdef TARGET_ESP32
    delay(5);
    #endif
}
