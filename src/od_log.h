#pragma once

#include <Arduino.h>

// Timestamped, allocation-free logging.
//
// Call od_log_init() once in setup() right after the serial port's begin().
// Levels are filtered at compile time via OD_LOG_LEVEL, so disabled levels
// compile to nothing.

#define OD_LOG_ERROR 0
#define OD_LOG_WARN  1
#define OD_LOG_INFO  2
#define OD_LOG_DEBUG 3

#ifndef OD_LOG_LEVEL
#define OD_LOG_LEVEL OD_LOG_INFO
#endif

#define od_log_error(fmt, ...) do { if (OD_LOG_LEVEL >= OD_LOG_ERROR) \
    _od_log(OD_LOG_ERROR, fmt, ##__VA_ARGS__); } while (0)
#define od_log_warn(fmt, ...)  do { if (OD_LOG_LEVEL >= OD_LOG_WARN) \
    _od_log(OD_LOG_WARN, fmt, ##__VA_ARGS__); } while (0)
#define od_log_info(fmt, ...)  do { if (OD_LOG_LEVEL >= OD_LOG_INFO) \
    _od_log(OD_LOG_INFO, fmt, ##__VA_ARGS__); } while (0)
#define od_log_debug(fmt, ...) do { if (OD_LOG_LEVEL >= OD_LOG_DEBUG) \
    _od_log(OD_LOG_DEBUG, fmt, ##__VA_ARGS__); } while (0)

void od_log_init(Stream *port);
void _od_log(int level, const char *fmt, ...) __attribute__((format(printf, 2, 3)));
void od_log_raw(const char *fmt, ...)         __attribute__((format(printf, 1, 2)));
void od_log_flush(void);

// Delivery contract: best-effort, bounded, may drop.
//
// On nRF the bytes go out through tud_cdc_write(), NOT Adafruit_USBD_CDC::write(),
// whose send loop is bounded only by "DTR is high" and so spins forever when a host
// holds the port open but stops reading. Writing through TinyUSB directly makes a
// stalled host structurally incapable of blocking loop() -- which matters because
// this chip has no watchdog and every fault handler is `b .`.
//
// A record that will not fit waits up to 20 ms for space and is then discarded and
// counted. The count is reported on the next record that gets out, spliced in after
// the level as "[DROP: n] " -- one line, never an extra one, and absent when the
// count is zero. Records logged from a task other than loop() do not wait at all.
//
// ESP32 keeps Stream::write and never drops or counts: neither of its log ports can
// block on host backpressure, so its output is unchanged.
// See docs/PLAN_NONBLOCKING_LOG_2026-07-29.md.

// Tells the logger whether a host is listening, so a dark port is not counted as
// dropped records. Optional; NULL (the default) means "assume ready". nRF only --
// ESP32's HWCDC::isCDC_Connected() flaps on a healthy link and would discard good
// output.
void od_log_set_ready_hook(bool (*fn)(void));

// Identifies the loop task, so logging from anywhere else can skip the wait rather
// than invert priority above it. Call once in setup(), immediately after
// od_log_init(); until then everything gets the full budget.
void od_log_set_loop_task(TaskHandle_t task);

// Records dropped since boot.
uint32_t od_log_dropped_total(void);

// Builds "<label><space-separated %02X bytes, up to 32><' ...' if truncated>" into
// buf. Lives here rather than in one caller's translation unit because the RX line
// (command_queue.cpp, on the stack callback task) and the TX line
// (communication.cpp, on loop()) must render frames identically -- two copies of
// this loop is how the two directions drift apart.
void od_log_hex_line(char *buf, size_t bufSize, const char *label,
                     const uint8_t *data, uint16_t len);
