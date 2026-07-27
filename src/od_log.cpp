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

void od_log_hex_line(char *buf, size_t bufSize, const char *label,
                     const uint8_t *data, uint16_t len) {
    int pos = snprintf(buf, bufSize, "%s", label);
    if (pos < 0) {
        pos = 0;
        buf[0] = '\0';
    }
    int dumpLen = (len < 32) ? len : 32;
    for (int i = 0; i < dumpLen && pos < (int)bufSize; i++) {
        int n = snprintf(buf + pos, bufSize - pos, i > 0 ? " %02X" : "%02X", data[i]);
        if (n < 0) {
            break;
        }
        pos += n;
    }
    if (len > 32 && pos >= 0 && pos < (int)bufSize) {
        snprintf(buf + pos, bufSize - pos, " ...");
    }
}

void od_log_flush(void) {
    if (s_port == NULL) {
        return;
    }

    s_port->flush();
    // Settling pause after flush(), unconditional as of 2026-07-27. Stream::flush()
    // returns once the driver has accepted the bytes, which is not the same as the
    // host having seen them: on a USB CDC port the transfer still has to be polled
    // off the device, and both targets log over CDC by default (nRF always -- there
    // is no OPENDISPLAY_LOG_UART path there). This was TARGET_ESP32-only for reasons
    // never recorded; the mechanism it compensates for is not ESP32-specific, and
    // od_log_flush() is called only at boot/wake checkpoints and before a rail cut --
    // the places where losing the last line costs the most and 5 ms costs nothing.
    // 16 call sites, so at most ~80 ms across a boot.
    delay(5);
}
