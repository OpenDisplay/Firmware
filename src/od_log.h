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
