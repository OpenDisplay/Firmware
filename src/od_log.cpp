#include "od_log.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#ifndef TARGET_ESP32
// The whole point of this file's nRF path: bypass Adafruit_USBD_CDC::write(), whose
// send loop is bounded only by tud_cdc_n_connected() -- literally "DTR is high" --
// and therefore spins forever when a host holds the port open but stops draining the
// IN endpoint. tud_cdc_write() underneath it is a bare tu_fifo_write_n that takes
// what fits and returns the count, so it cannot block no matter what the host does.
// See docs/PLAN_NONBLOCKING_LOG_2026-07-29.md.
#include <Adafruit_TinyUSB.h>
#endif

// Implemented in main.cpp: RTC-persisted wake cycle count on ESP32, always 0 on nRF52840.
uint32_t getDeepSleepCount();

// Log output destination, set once by od_log_init(). Stays NULL if
// od_log_init() is never called (e.g. DISABLE_USB_SERIAL builds), in which
// case all log calls become no-ops. On nRF this is still used for the readiness
// hook and flush; the record bytes go out through tud_cdc_write().
static Stream *s_port = NULL;

// Optional "is a host actually listening" predicate. With DTR low, TinyUSB flips the
// TX FIFO to overwritable (cdcd_init -> tu_fifo_set_overwritable(&tx_ff, !dtr)), so
// tud_cdc_write_available() can read 0 while a write would in fact succeed by
// discarding old bytes. Without this hook the all-or-nothing gate below would drop
// every line on an unattended tag and hand the first terminal to attach a
// "[DROP: 4102931]" -- a true number that says nothing. NULL means "assume ready".
static bool (*s_readyHook)(void) = NULL;

// The loop task, captured in setup(). Anything logging from another context --
// Bluefruit's "Callback" task at priority 2, the FreeRTOS timer task, or the BLE
// task at priority 3 when ada_callback()'s queue is full and the write callback
// runs inline -- gets a zero budget, because waiting above loop() (priority 1) is
// priority inversion. NULL means "not captured yet": everything then gets the full
// budget, which is the safe bring-up default. The NULL test is not optional -- see
// od_budget_ms().
static TaskHandle_t s_loopTask = NULL;

// Records discarded for want of TX space, reported on the next line that gets out.
// Written from three tasks, hence the atomics.
static uint32_t s_dropped = 0;
static uint32_t s_droppedTotal = 0;

// Consecutive drops taken ON THE LOOP TASK ONLY. Deliberately not a global count of
// every drop: this gates the loop budget and nothing else, and a global is poisoned
// across contexts on a perfectly healthy host -- a -debug frame burst drops a few
// off-loop hex lines (they run at budget 0 and the FIFO drains ~64 B per bulk-IN
// completion), and a loop-task ERROR arriving a millisecond later would then find
// the budget already zeroed when 20 ms would certainly have saved it. Loop-only also
// makes this single-writer, so it needs no atomics.
static uint32_t s_loopConsecutiveDrops = 0;

// Total wait budget per record, shared between the mutex take and the room wait.
// Not a safety bound -- nothing here can block -- but a latency one: without a wait
// at all, a single-shot capacity test fails constantly on a healthy host, because
// the FIFO is 256 bytes, the longest record is ~210, and one image push emits ~300
// records back to back. The wait is what keeps the frame dumps.
static const uint32_t OD_LOG_BUDGET_MS = 20;

// After this many consecutive loop-task drops the port is presumed stalled and the
// budget goes to zero until something gets through. Without it a stalled host costs
// the full budget on every record: ~300 records per push x 20 ms is ~6 s of added
// loop() latency, enough to starve BLE ACK draining and time out a transfer that
// would otherwise have completed. Resets on any successful write, so recovery needs
// no detection of its own.
static const uint32_t OD_LOG_STALL_DROPS = 3;

// Longest record handed to the port. Worst wire line is 232 + 19 ("[DROP: 4294967295] ")
// + 2 (CRLF) = 253, inside the 256-byte CFG_TUD_CDC_TX_BUFSIZE. Nothing currently
// emitted reaches it -- the longest is command_queue.cpp's 192-byte hex body plus a
// ~20-byte header -- so this is a guard, not a routine cost.
static const size_t OD_LOG_MAX_TEXT = 232;

// Serialises the capacity reservation and the writes that follow it.
//
// NOT a hang guard: TinyUSB's own FIFO is multi-writer safe (CFG_TUSB_OS ==
// OPT_OS_FREERTOS enables CFG_FIFO_MUTEX, and cdcd_init gives tx_ff a write mutex),
// so a single tud_cdc_write() is already atomic against other writers. What is not
// atomic is the span from "there is room for the whole record" to the last write of
// that record: without this lock another producer can consume the reserved space in
// between and truncate the line. A failure here costs a mangled or dropped line, not
// a wedged tag -- which is exactly the property the tud_cdc_write() rewrite buys.
//
// Static allocation only to avoid an allocation-failure branch.
static StaticSemaphore_t s_txLockStorage;
static SemaphoreHandle_t s_txLock = NULL;

static const char level_chars[] = "EWID";

void od_log_init(Stream *port) {
    if (s_txLock == NULL) {
        s_txLock = xSemaphoreCreateMutexStatic(&s_txLockStorage);
    }
    s_port = port;
}

void od_log_set_ready_hook(bool (*fn)(void)) {
    s_readyHook = fn;
}

void od_log_set_loop_task(TaskHandle_t task) {
    s_loopTask = task;
}

uint32_t od_log_dropped_total(void) {
    return __atomic_load_n(&s_droppedTotal, __ATOMIC_RELAXED);
}

// How long this record may wait. Zero means "try once, then discard".
static uint32_t od_budget_ms(void) {
    // The NULL test comes first and is load-bearing: xTaskGetCurrentTaskHandle()
    // never returns NULL under a running scheduler, so without it an uncaptured
    // s_loopTask would make the inequality true for every caller and silently put
    // the whole firmware into try-once-then-drop -- the exact inverse of the
    // intended bring-up default.
    if (s_loopTask != NULL && xTaskGetCurrentTaskHandle() != s_loopTask) {
        return 0;
    }
    if (s_loopConsecutiveDrops >= OD_LOG_STALL_DROPS) {
        return 0;
    }
    return OD_LOG_BUDGET_MS;
}

static bool od_on_loop_task(void) {
    return (s_loopTask == NULL) || (xTaskGetCurrentTaskHandle() == s_loopTask);
}

static void od_count_drop(bool feedBackoff) {
    __atomic_fetch_add(&s_dropped, 1u, __ATOMIC_RELAXED);
    __atomic_fetch_add(&s_droppedTotal, 1u, __ATOMIC_RELAXED);
    // A lock-timeout drop passes feedBackoff=false: losing a record to another
    // producer holding the mutex says nothing about the state of the port.
    if (feedBackoff && od_on_loop_task()) {
        s_loopConsecutiveDrops++;
    }
}

// Free space in the port's TX buffer, without blocking on either target.
static int od_port_room(void) {
#ifdef TARGET_ESP32
    return (s_port != NULL) ? s_port->availableForWrite() : 0;
#else
    return (int)tud_cdc_write_available();
#endif
}

// The only place the two targets differ. All-or-nothing by contract: the caller has
// already reserved `n` bytes and holds the lock, so on nRF this always takes the lot.
//
// On nRF this CANNOT block -- tud_cdc_write() is tu_fifo_write_n plus a conditional
// flush, and tud_cdc_write_flush() bails through usbd_edpt_claim() rather than
// waiting. On ESP32 it stays on Stream::write, which is bounded (HWCDC caps at
// max_consec_timeouts x tx_timeout_ms; the log UART has flow control hardwired off
// so its FIFO always drains at the baud rate) and whose short-write behaviour is
// deliberately left exactly as it is today -- see od_emit().
static bool od_port_write(const uint8_t *b, size_t n) {
#ifdef TARGET_ESP32
    return s_port->write(b, n) == n;
#else
    return tud_cdc_write(b, n) == n;
#endif
}

// True once the port can take `need` bytes, false if the budget ran out first.
static bool od_port_wait_ready(int need, TickType_t start, TickType_t budget) {
#ifdef TARGET_ESP32
    // Short-circuit by design. Neither ESP32 log port can block on host
    // backpressure, so there is nothing to wait for and nothing to protect
    // against: always write.
    (void)need; (void)start; (void)budget;
    return true;
#else
    for (;;) {
        // The room test MUST precede the expiry test. With a zero budget this
        // degenerates to "try once, then discard"; reversed, it becomes "discard
        // without trying" and every off-loop record is lost unconditionally.
        if (od_port_room() >= need) {
            return true;
        }
        if ((TickType_t)(xTaskGetTickCount() - start) >= budget) {
            return false;
        }
        // vTaskDelay, NOT delay(): the core's delay() flushes CDC first and returns
        // early without ever calling vTaskDelay when that flush spans a tick
        // (cores/nRF5/delay.c, `if (flush_tick >= ticks) return;`, and ms2tick(1) is
        // one tick at 1024 Hz). That turns this into a busy-spin at priority 2 that
        // never yields to loop(), and time slicing is disabled.
        vTaskDelay(1);
    }
#endif
}

// The single write choke point. Emits one log record as a run of writes covered by a
// single capacity reservation, so no write in the run can come up short.
//
// `text` is the whole record without its line ending. `tagAt` is the offset at which
// a "[DROP: n] " tag may be spliced (the first byte after "] L: "), or -1 for a
// partial line that must never carry one.
static void od_emit(const char *text, int tagAt, bool newline) {
    if (s_port == NULL) {
        return;
    }
    // Dark port: discard without counting. Nobody is listening, so there is no gap
    // for a drop report to explain.
    if (s_readyHook != NULL && !s_readyHook()) {
        return;
    }

    const TickType_t start  = xTaskGetTickCount();
    const TickType_t budget = pdMS_TO_TICKS(od_budget_ms());

    if (s_txLock != NULL) {
        // Stamped once, before the take, and the take gets only what is left --
        // giving it a fresh full budget would make the worst case per record 2x.
        const TickType_t used = (TickType_t)(xTaskGetTickCount() - start);
        const TickType_t left = (used >= budget) ? 0 : (TickType_t)(budget - used);
        if (xSemaphoreTake(s_txLock, left) != pdTRUE) {
            od_count_drop(false);
            return;
        }
    }

    size_t len = strlen(text);
    if (len > OD_LOG_MAX_TEXT) {
        len = OD_LOG_MAX_TEXT;
    }
    if (tagAt > (int)len) {
        tagAt = -1;   // truncation ate the splice point
    }

    // Read without clearing: the count is only consumed once the record is out.
    uint32_t reported = (tagAt >= 0) ? __atomic_load_n(&s_dropped, __ATOMIC_RELAXED) : 0u;
    char tag[24];
    size_t tagLen = 0;
    if (reported > 0) {
        int n = snprintf(tag, sizeof(tag), "[DROP: %lu] ", (unsigned long)reported);
        if (n > 0 && n < (int)sizeof(tag)) {
            tagLen = (size_t)n;
        } else {
            reported = 0;
        }
    }

    const int need = (int)(len + tagLen + (newline ? 2 : 0));
    if (!od_port_wait_ready(need, start, budget)) {
        od_count_drop(true);
        if (s_txLock != NULL) {
            xSemaphoreGive(s_txLock);
        }
        return;
    }

    // Every write below is covered by the reservation above and runs under the lock,
    // so on nRF each one takes its bytes in full and the record cannot be truncated
    // or interleaved. On ESP32 the returns are deliberately ignored: a short write
    // there would otherwise set s_dropped, which would put the next record on the
    // tagged path -- a path this target must never take, and which would break the
    // "ESP32 output is byte-identical to before" guarantee. ESP32 loss stays silent,
    // exactly as it is today.
    bool ok = true;
    if (tagLen > 0) {
        ok = od_port_write((const uint8_t *)text, (size_t)tagAt) && ok;
        ok = od_port_write((const uint8_t *)tag, tagLen) && ok;
        ok = od_port_write((const uint8_t *)text + tagAt, len - (size_t)tagAt) && ok;
    } else {
        ok = od_port_write((const uint8_t *)text, len) && ok;
    }
    if (newline) {
        ok = od_port_write((const uint8_t *)"\r\n", 2) && ok;
    }

#ifdef TARGET_ESP32
    (void)ok;                 // see the comment above; ESP32 never counts a drop
    s_loopConsecutiveDrops = 0;
    if (reported > 0) {
        __atomic_fetch_sub(&s_dropped, reported, __ATOMIC_RELAXED);
    }
#else
    // Push whatever is queued at the record boundary rather than waiting for a full
    // bulk packet to accumulate, so a lone line is not held back by a quiet link.
    tud_cdc_write_flush();
    if (ok) {
        s_loopConsecutiveDrops = 0;
        if (reported > 0) {
            // Subtract what was reported rather than zeroing, so a drop taken by
            // another task between the load above and here is not swallowed.
            __atomic_fetch_sub(&s_dropped, reported, __ATOMIC_RELAXED);
        }
    } else {
        // Cannot happen while the reservation and the lock both hold; if it ever
        // does, the tag bytes reached the host, so consume the report (re-reporting
        // would double-count for a reader summing them) and count the lost record.
        if (reported > 0) {
            __atomic_fetch_sub(&s_dropped, reported, __ATOMIC_RELAXED);
        }
        od_count_drop(true);
    }
#endif

    if (s_txLock != NULL) {
        xSemaphoreGive(s_txLock);
    }
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

    // pos is the offset of the first message byte, i.e. just past "] L: ", which is
    // where a drop tag goes so the timestamp keeps column 1. The header is ~29 chars
    // at worst, so it can never be the thing snprintf truncates.
    od_emit(buf, pos, true);
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

    // tagAt = -1: this emits partial lines (waitforrefresh()'s progress dots), and a
    // "[DROP: n]" spliced into a run of dots is noise rather than information. Its
    // drops are still counted; the total surfaces on the next complete _od_log line.
    od_emit(buf, -1, false);
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

    // Taken so this cannot land between the writes of a record another task is
    // mid-way through emitting. Bounded, and skipping the flush on contention costs
    // nothing that the next flush will not fix.
    const bool locked = (s_txLock != NULL) &&
                        (xSemaphoreTake(s_txLock, pdMS_TO_TICKS(OD_LOG_BUDGET_MS)) == pdTRUE);
#ifdef TARGET_ESP32
    s_port->flush();
#else
    tud_cdc_write_flush();
#endif
    if (locked) {
        xSemaphoreGive(s_txLock);
    }

    // Settling pause after flush(), unconditional as of 2026-07-27. flush() returns
    // once the driver has accepted the bytes, which is not the same as the host
    // having seen them: on a USB CDC port the transfer still has to be polled off the
    // device, and both targets log over CDC by default (nRF always -- there is no
    // OPENDISPLAY_LOG_UART path there). od_log_flush() is called only at boot/wake
    // checkpoints and before a rail cut -- the places where losing the last line
    // costs the most and 5 ms costs nothing. 16 call sites, so at most ~80 ms across
    // a boot. Deliberately OUTSIDE the lock: 16 x 5 ms of hold is not worth adding.
    delay(5);
}
